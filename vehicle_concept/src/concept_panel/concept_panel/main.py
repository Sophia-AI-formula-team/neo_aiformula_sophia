"""ROS 2 Foxy panel. ROS callbacks and widgets share the Qt thread."""
import json
import signal
import sys
import time

from .model import Action, stamp_seconds
from .qt import QtCore, QtGui, QtWidgets
from .window import MODE, STATUS, Window, reason

LABELS={'enable':'启用电机输出','disable':'禁用电机输出','pause':'暂停自动驾驶',
        'resume':'继续自动驾驶','restart':'重启驾驶节点','target':'目标速度','limit':'自动驾驶限速'}

BAG_STATES = {
    'idle': ('未录制', '开始录包', True, '#52685e'),
    'starting': ('正在启动', '停止录包', True, '#8b630d'),
    'recording': ('录制中', '停止录包', True, '#9f342e'),
    'stopping': ('正在保存', '正在保存', False, '#8b630d'),
    'saved': ('已保存', '开始录包', True, '#205944'),
    'error': ('录制失败', '重新录包', True, '#9f342e'),
}


class RecordingControls:
    def __init__(self, recorder, window):
        self.recorder = recorder
        self.window = window
        self.last = None
        window.bag_action = self.toggle
        self.refresh()

    def toggle(self):
        state = self.recorder.status().state
        if state in ('starting', 'recording'):
            self.render(self.recorder.stop())
        elif state in ('idle', 'saved', 'error'):
            self.render(self.recorder.start())

    def refresh(self):
        self.render(self.recorder.status())

    def render(self, snapshot):
        if snapshot == self.last:
            return
        title, button, enabled, color = BAG_STATES[snapshot.state]
        path = str(snapshot.path or self.recorder.root)
        message = title
        if snapshot.state == 'saved' and snapshot.messages is not None:
            message += ' · {} 条消息'.format(snapshot.messages)
        self.window.show_bag(message, path, button, enabled, color)
        self.window.bag_status.setToolTip(snapshot.error or message)
        if (self.last is None or self.last.state != snapshot.state) and snapshot.state != 'idle':
            self.window.log_event('录包', message + ' · ' + (snapshot.error or path))
        self.last = snapshot


class LapMapUpdates:
    def __init__(self, node, dialog):
        from std_srvs.srv import Trigger

        self.dialog = dialog
        self.client = node.create_client(Trigger, '/concept/drive/lap_map')
        self.request = Trigger.Request
        self.pending = None
        self.sent_at = float('-inf')

    def refresh(self, steady):
        if self.pending is not None:
            if self.pending.done():
                try:
                    response = self.pending.result()
                    if self.dialog.isVisible():
                        if response.success:
                            self.dialog.show_map(response.message)
                        else:
                            self.dialog.connection.setText(response.message)
                except (TypeError, ValueError, KeyError, RuntimeError):
                    self.dialog.connection.setText('地图读取失败 · 保留上次地图')
                self.pending = None
            elif steady-self.sent_at > 3.:
                self.pending.cancel()
                self.pending = None
                self.dialog.connection.setText('地图服务超时 · 保留上次地图')
        if self.pending is not None or not self.dialog.isVisible() or steady-self.sent_at < 1.:
            return
        self.sent_at = steady
        if not self.client.service_is_ready():
            self.dialog.connection.setText('未连接地图服务 · 保留上次地图')
            return
        try:
            self.pending = self.client.call_async(self.request())
        except RuntimeError:
            self.dialog.connection.setText('地图请求失败 · 保留上次地图')


def image_from_ros(msg):
    encoding=msg.encoding.lower()
    formats={'rgb8':(3,QtGui.QImage.Format_RGB888),'bgr8':(3,QtGui.QImage.Format_RGB888),
             'rgba8':(4,QtGui.QImage.Format_RGBA8888),'mono8':(1,QtGui.QImage.Format_Grayscale8)}
    if encoding not in formats:
        raise ValueError('不支持的图像编码 '+encoding)
    channels,fmt=formats[encoding]
    if min(msg.width,msg.height)<=0 or msg.step<channels*msg.width or len(msg.data)<msg.step*msg.height:
        raise ValueError('图像尺寸与缓冲区不符')
    image=QtGui.QImage(bytes(msg.data),msg.width,msg.height,msg.step,fmt).copy()
    return image.rgbSwapped() if encoding=='bgr8' else image


class Controller:
    def __init__(self, node, window):
        from rcl_interfaces.srv import SetParameters
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from std_srvs.srv import SetBool,Trigger
        from .recorder import BagRecorder

        self.node,self.window=node,window
        self.recording = RecordingControls(BagRecorder(), window)
        self.lap_map = LapMapUpdates(node, window.lap_map)
        self.pending=self.future=None
        self.clients={}
        for key,path,kind in [
            ('enable','/concept/gate/enable_output',SetBool),('pause','/concept/gate/pause',Trigger),
            ('resume','/concept/gate/resume',Trigger),('restart','/concept/drive/restart',Trigger),
            ('target','/concept/drive/set_parameters',SetParameters),('limit','/concept/gate/set_parameters',SetParameters)]:
            self.clients[key]=(node.create_client(kind,path),kind)
        self.clients['disable']=self.clients['enable']
        node.create_subscription(String,'/concept/gate/status',lambda m:self.message('门控',m),1)
        node.create_subscription(String,'/concept/decision',lambda m:self.message('决策',m),1)
        node.create_subscription(Image,'/concept/view/camera',self.camera,qos_profile_sensor_data)
        window.dispatch=self.start
        self.previous_live=None

    def ros(self):
        return self.node.get_clock().now().nanoseconds*1e-9

    def message(self, channel, msg):
        try:
            d=json.loads(msg.data)
            if channel=='门控':
                stamp=stamp_seconds(d['t'])
                self.window.gate.put(d,stamp,time.monotonic())
                detail=MODE.get(d['mode'],d['mode'])+' · '+reason(d.get('reason','—'))
            else:
                stamp=stamp_seconds(d['stamp']) if d.get('stamp') is not None else None
                self.window.decision.put(d,stamp,time.monotonic())
                detail=STATUS.get(d['status'],d['status'])+' · '+reason(d.get('reason','—'))
            self.window.log_event(channel,detail)
        except (ValueError,TypeError,KeyError) as exc:
            feed=self.window.gate if channel=='门控' else self.window.decision
            feed.put({},None,time.monotonic())
            self.window.log_event(channel,'消息无效 · '+str(exc))

    def camera(self,msg):
        try:
            stamp=msg.header.stamp
            self.window.image(image_from_ros(msg),stamp.sec+stamp.nanosec*1e-9,time.monotonic())
        except (TypeError,ValueError) as exc:
            self.window.log_event('摄像头',str(exc))

    def finish(self,text):
        self.pending=self.future=None
        self.window.busy=False
        self.window.pending_text=text
        self.window.log_event('控制',text)

    def start(self,command,value=None):
        if self.pending:
            if command not in ('disable','pause'):
                return
            # Supersede future local steps, especially queued restart. The ROS service
            # call already sent still needs authoritative gate telemetry for confirmation.
            if self.future is not None:
                self.future.cancel()
            self.finish('之前的操作已被中断，以门控回读为准')
        self.pending=Action(command,value)
        self.window.busy=True
        self.issue()

    def issue(self):
        from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
        key=self.pending.steps[0]
        client,kind=self.clients[key]
        if not client.service_is_ready():
            self.finish(LABELS[key]+'失败 · 服务不可用')
            return
        req=kind.Request()
        if key in ('enable','disable'):
            req.data=key=='enable'
        if key in ('target','limit'):
            name='target_speed_mps' if key=='target' else 'max_auto_speed_mps'
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=float(self.pending.value))
            req.parameters=[Parameter(name=name,value=value)]
        try:
            self.future=client.call_async(req)
            self.deadline=time.monotonic()+3.
            self.window.pending_text='等待确认 · '+LABELS[key]
        except Exception as exc:
            self.finish('请求失败 · '+str(exc))

    def poll(self):
        if self.pending is None:
            return
        key=self.pending.steps[0]
        if not self.future.done():
            if time.monotonic()>self.deadline:
                self.future.cancel()
                self.finish(LABELS[key]+'响应超时，执行状态未知')
            return
        try:
            reply=self.future.result()
            if key in ('target','limit'):
                ok=len(reply.results)==1 and reply.results[0].successful
                why=reply.results[0].reason if reply.results else '缺少参数结果'
            else:
                ok,why=reply.success,reply.message
            next_key=self.pending.completed(ok)
            text=LABELS[key]+('已确认' if ok else '未执行')+(' · '+why if why else '')
            if next_key:
                self.window.log_event('控制',text)
                self.issue()
            else:
                self.finish(text)
        except Exception as exc:
            self.finish('响应无效 · '+str(exc))

    def update(self):
        self.recording.refresh()
        self.poll()
        now=self.ros()
        steady=time.monotonic()
        self.lap_map.refresh(steady)
        age=self.window.gate.age(now,steady)
        live=age is not None and age<=.5
        if self.previous_live is not None and self.previous_live!=live:
            self.window.log_event('连接','门控已连接' if live else '门控数据失联')
        self.previous_live=live
        self.window.refresh(now,steady)


def main(args=None):
    import rclpy
    from rclpy.node import Node
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    rclpy.init(args=args)
    node=Node('concept_panel')
    window=Window()
    controller=Controller(node,window)
    shutting_down = False

    def request_exit(signum, frame):
        nonlocal shutting_down
        shutting_down = True

    previous_signals = {sig: signal.signal(sig, request_exit) for sig in (signal.SIGINT, signal.SIGTERM)}

    def spin():
        if shutting_down or not rclpy.ok():
            app.quit()
        else:
            # Bounded work keeps buttons and service timeouts responsive under camera load.
            for _ in range(4):
                rclpy.spin_once(node,timeout_sec=0.)

    io=QtCore.QTimer()
    io.timeout.connect(spin)
    io.start(10)
    refresh=QtCore.QTimer()
    refresh.timeout.connect(controller.update)
    refresh.start(100)
    window.show()
    try:
        app.exec_() if hasattr(app,'exec_') else app.exec()
    finally:
        io.stop()
        refresh.stop()
        recorder = controller.recording.recorder
        result = recorder.close()
        # Qt has exited. Keep our SIGINT/SIGTERM handlers installed until the
        # independent recorder process group is reaped and SQLite is verified.
        while result.state in ('starting', 'recording', 'stopping'):
            time.sleep(.05)
            result = recorder.status()
        if result.state == 'error':
            print('录包收尾失败 · {} · {}'.format(result.path, result.error), file=sys.stderr, flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous_signals.items():
            signal.signal(sig, handler)
