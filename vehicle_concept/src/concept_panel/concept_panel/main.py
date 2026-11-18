"""ROS 2 Foxy panel. ROS callbacks and widgets share the Qt thread."""
import json
import sys
import time

from .model import Action, stamp_seconds
from .qt import QtCore, QtGui, QtWidgets
from .window import MODE, STATUS, Window, reason

LABELS={'enable':'启用电机输出','disable':'禁用电机输出','pause':'暂停自动驾驶',
        'resume':'继续自动驾驶','restart':'重启驾驶节点','target':'规划目标','limit':'自动上限'}


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

        self.node,self.window=node,window
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
        self.poll()
        now=self.ros()
        steady=time.monotonic()
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

    def spin():
        if rclpy.ok():
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
