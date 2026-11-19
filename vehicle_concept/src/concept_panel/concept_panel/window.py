from datetime import datetime
import math

from .lap_map import LapMapPanel

from .model import Feed, Journal
from .qt import QtCore, QtGui, QtWidgets as W

MODE = {'disabled':'输出禁用', 'paused':'自动已暂停', 'manual':'手柄接管', 'auto':'自动驾驶', 'estop':'软件急停'}
STATUS = {'ok':'有效', 'invalid':'无效', 'blocked':'输出受阻', 'shadow':'影子模式', 'waiting':'等待输入', 'late':'超时'}
REASONS={'startup':'初始化','output_enabled':'等待恢复自动驾驶','output_disabled':'电机输出已禁用',
         'operator_pause':'操作员已暂停','manual_takeover':'手柄已接管','manual_released':'手柄已松开，等待继续',
         'joy_timeout':'手柄数据超时','graph_conflict':'控制链存在冲突','motor_timeout':'电机转换输出超时',
         'waiting_fresh_auto':'当前指令已到期，零输出等待新帧',
         'image gap over 0.5 s; pause the gate and restart the driver':'图像采集间隔超过 0.5 秒',
         'planning gap over 0.5 s after missing synchronized wheel/gyro feedback':'图像已到达，轮速与陀螺反馈未配对，规划中断超过 0.5 秒',
         'planning gap over 0.5 s between completed frames':'图像已到达，两次完成规划的间隔超过 0.5 秒',
         'vehicle_state_timeout':'车辆状态超时','speed_limit_changed':'速度上限已更改，等待继续',
         'auto_timeout':'自动指令超时','auto_speed_limit':'自动指令超过速度上限',
         'waiting_new_auto':'等待恢复后的新指令','auto_command_received':'已收到新的自动指令','feasible':'已选择可行轨迹',
         'no_feasible_trajectory':'当前无可行轨迹','unobserved_road':'缺少道路观测',
         'executing_prior_contingency':'沿上一帧已检查的备用路线行驶',
         'checked_slow_reverse':'后方路线已检查，准备低速倒车',
         'checked_reverse_release':'结束倒车，等待车停稳',
         'executing_checked_reverse_release':'按已检查的方案结束倒车',
         'recovery_unknown_road':'后方路界未知',
         'recovery_no_checked_motion':'前后暂时都没有通过检查的方案',
         'camera geometry has no validated driving profile':'相机安装标定尚未通过',
         'Honda mount projection active; ground error unmeasured':'本田安装参数投影已启用 · 地面误差未实测',
         'reverse wheel-feedback sign unverified':'轮速反馈尚未确认能识别倒车方向',
         'decision exceeded 0.5 s from acquisition':'拍摄到出指令超过 0.5 秒'}

def reason(value):
    return REASONS.get(value,value) if isinstance(value,str) else '—'

STYLE = """
QWidget { background:#f2f4f1; color:#243831; font-family:'Microsoft YaHei UI','Noto Sans CJK SC',sans-serif; font-size:13px; }
QLabel {background:transparent;}
QFrame#card {background:#ffffff;border:1px solid #d4ded7;border-radius:10px;}
QLabel#title {font-size:26px;font-weight:700;color:#173c30;}
QLabel#heading {font-size:17px;font-weight:700;}
QLabel#muted {color:#52685e;}
QLabel#badge {background:#dcebe3;color:#205944;border-radius:5px;padding:5px 10px;font-weight:600;}
QLabel#metric {font-size:23px;font-weight:650;color:#174a38;}
QLabel#camera {background:#172c25;color:#b4cbc0;border-radius:6px;}
QLabel#motionAction {font-size:43px;font-weight:750;}
QLabel#decisionAction {font-size:19px;font-weight:700;}
QLabel#decisionReason {font-size:14px;}
QLabel#decisionMode {font-size:14px;font-weight:650;padding:5px 9px;border:1px solid #e7efec;border-radius:5px;}
QLineEdit#bag_path {background:#f2f5f1;border:1px solid #d4ded7;border-radius:4px;padding:4px;}
QPushButton {background:#f4f7f4;border:1px solid #c3d0c7;border-radius:5px;padding:6px 10px;font-weight:550;}
QPushButton:hover {background:#e6efe9;}
QPushButton#primary {background:#245e49;color:white;border-color:#245e49;}
QPushButton#stop {background:#f9e8df;color:#783817;border-color:#e2c7b7;}
QPushButton#primary:disabled,QPushButton#stop:disabled,QPushButton:disabled {background:#edf0ed;color:#7d8980;border-color:#e0e4e0;}
QDoubleSpinBox {background:white;border:1px solid #becfc3;border-radius:4px;padding:4px;}
QTreeWidget {background:white;border:0;alternate-background-color:#f3f6f3;}
QHeaderView::section {background:#edf2ed;color:#456051;border:0;padding:4px;text-align:left;}
"""


def label(text='', style=None):
    item = W.QLabel(text)
    if style:
        item.setObjectName(style)
    return item


def card():
    box = W.QFrame()
    box.setObjectName('card')
    layout = W.QVBoxLayout(box)
    layout.setContentsMargins(15, 12, 15, 12)
    layout.setSpacing(7)
    return box, layout


def number(value, digits=2):
    return format(value, '.'+str(digits)+'f') if isinstance(value,(float,int)) else '—'


AUTO_YAW_LIMIT = 1.55  # V6.26 candidate yaw limit, used only to scale the display.


def output_motion(gate):
    speed, yaw = gate.get('linear_mps'), gate.get('yaw_rps')
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (speed, yaw)):
        return '等待门控', 'unknown', None
    if speed < 0:
        return '倒车', 'reverse', 180.
    if speed > 0:
        limit = (gate.get('manual_yaw_limit_rps') or 1.) if gate.get('mode') == 'manual' else AUTO_YAW_LIMIT
        ratio = max(-1., min(1., yaw / limit))
        angle = -90. * math.copysign(math.sqrt(abs(ratio)), ratio)
        return '前进', 'forward', angle
    if yaw:
        return '原地转向', 'turn', -90. if yaw > 0 else 90.
    return '停止', 'stop', None


class DirectionArrow(W.QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(95)
        self.setMaximumHeight(180)
        self.motion, self.degrees = 'unknown', None
        self.setAccessibleName('门控指令方向')

    def show_motion(self, motion, degrees):
        if (motion, degrees) != (self.motion, self.degrees):
            self.motion, self.degrees = motion, degrees
            self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.translate(self.width()/2, self.height()/2)
        scale = min(self.width(), self.height()) / 126.
        painter.scale(scale, scale)
        painter.setBrush(QtGui.QColor('#f7faf8'))
        painter.setPen(QtCore.Qt.NoPen)
        if self.degrees is not None:
            painter.rotate(self.degrees)
            outline = [(-12, 48), (12, 48), (12, -7), (32, -7), (0, -50), (-32, -7), (-12, -7)]
            painter.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(x, y) for x, y in outline]))
        elif self.motion == 'stop':
            painter.drawRoundedRect(QtCore.QRectF(-25, -25, 50, 50), 5, 5)
        else:
            painter.drawRoundedRect(QtCore.QRectF(-22, -4, 44, 8), 3, 3)


class Window(W.QWidget):
    def __init__(self, offline=False):
        super().__init__()
        self.offline = offline
        self.dispatch = None
        self.bag_action = None
        self.busy = False
        self.pending_text = ''
        self.gate, self.decision, self.image_feed = Feed(), Feed(), Feed()
        self.journal = Journal()
        self.pix = None
        self.setWindowTitle('Concept · 驾驶控制台')
        self.setStyleSheet(STYLE)
        self.resize(1120, 720)
        root = W.QVBoxLayout(self)
        root.setContentsMargins(18,12,18,12)
        root.setSpacing(8)
        title = W.QHBoxLayout()
        title.addWidget(label('驾驶控制台','title'))
        title.addWidget(label('V6.27.5 · 驾驶策略 V6.26','muted'))
        title.addStretch()
        self.connection = label('等待门控','badge')
        title.addWidget(self.connection)
        root.addLayout(title)
        columns = W.QHBoxLayout()
        columns.setSpacing(12)
        root.addLayout(columns, 1)
        self.decision_surface = W.QFrame()
        self.decision_surface.setObjectName('decisionSurface')
        decision_layout = W.QVBoxLayout(self.decision_surface)
        decision_layout.setContentsMargins(18,16,18,16)
        decision_layout.setSpacing(8)
        decision_bar = W.QHBoxLayout()
        decision_bar.addWidget(label('门控输出','heading'))
        decision_bar.addStretch()
        self.mode = label('电机状态未知','decisionMode')
        decision_bar.addWidget(self.mode)
        decision_layout.addLayout(decision_bar)
        decision_layout.addStretch()
        self.motion = label('等待门控','motionAction')
        self.motion.setAlignment(QtCore.Qt.AlignCenter)
        decision_layout.addWidget(self.motion)
        self.arrow = DirectionArrow()
        decision_layout.addWidget(self.arrow, 1)
        self.output = label('—')
        self.output.setAlignment(QtCore.Qt.AlignCenter)
        decision_layout.addWidget(self.output)
        decision_layout.addStretch()
        self.reason = label('等待门控数据')
        self.reason.setWordWrap(True)
        decision_layout.addWidget(self.reason)
        self.action = label('规划 · 等待决策','decisionAction')
        self.action.setToolTip('自动驾驶规划器建议的动作')
        decision_layout.addWidget(self.action)
        self.detail = label('等待有效决策','decisionReason')
        self.detail.setWordWrap(True)
        decision_layout.addWidget(self.detail)
        self.command = label('')
        self.command.setWordWrap(True)
        decision_layout.addWidget(self.command)
        self.output_color = None
        self.decision_surface.setMinimumWidth(290)

        camera, cam = card()
        cam.setContentsMargins(14, 9, 14, 9)
        columns.addWidget(camera, 5)
        self.lap_map = LapMapPanel(self)
        columns.addWidget(self.lap_map, 4)
        columns.addWidget(self.decision_surface, 3)
        bar = W.QHBoxLayout()
        bar.addWidget(label('摄像头与规划叠图','heading'))
        bar.addStretch()
        self.camera_time = label('等待图像','muted')
        bar.addWidget(self.camera_time)
        cam.addLayout(bar)
        self.camera = label('等待摄像头叠图','camera')
        self.camera.setMinimumSize(380,230)
        self.camera.setSizePolicy(W.QSizePolicy.Ignored, W.QSizePolicy.Ignored)
        self.camera.setAlignment(QtCore.Qt.AlignCenter)
        cam.addWidget(self.camera,1)
        legend = W.QHBoxLayout()
        for text, color in (('可行候选', '#1e6a54'), ('淘汰候选', '#59646b'), ('最终路线', '#a52770')):
            item = label('━  ' + text)
            item.setStyleSheet(f'color:{color};font-weight:650;')
            legend.addWidget(item)
        legend.addStretch()
        cam.addLayout(legend)
        recording = W.QHBoxLayout()
        recording.addWidget(label('ROS 数据包','heading'))
        self.bag_status = label('未录制')
        self.bag_status.setObjectName('bag_status')
        recording.addWidget(self.bag_status)
        recording.addStretch()
        self.bag_toggle = W.QPushButton('开始录包')
        self.bag_toggle.setObjectName('bag_toggle')
        self.bag_toggle.setToolTip('保存决策、车况和 640×360 JPEG 识别画面。画面跟随识别帧率。')
        self.bag_toggle.setEnabled(False)
        self.bag_toggle.clicked.connect(self.request_bag)
        recording.addWidget(self.bag_toggle)
        cam.addLayout(recording)
        path_line = W.QHBoxLayout()
        path_line.addWidget(label('保存位置','muted'))
        self.bag_path = W.QLineEdit()
        self.bag_path.setObjectName('bag_path')
        self.bag_path.setReadOnly(True)
        self.bag_path.setPlaceholderText('等待录制目录')
        self.bag_path.setAccessibleName('录包保存位置')
        path_line.addWidget(self.bag_path, 1)
        cam.addLayout(path_line)

        controls, stack = card()
        root.addWidget(controls)
        toolbar = W.QHBoxLayout()
        stack.addLayout(toolbar)
        self.buttons = {}
        for key,text,style in [
                ('enable','启用电机输出','primary'),('disable','禁用电机输出','stop'),
                ('pause','暂停自动驾驶',None),('resume','继续自动驾驶',None),
                ('restart','暂停并重启驾驶节点',None)]:
            button=W.QPushButton(text)
            if style: button.setObjectName(style)
            button.clicked.connect(lambda checked=False, key=key: self.request(key))
            self.buttons[key]=button
            toolbar.addWidget(button)
        toolbar.addStretch()

        self.settings_dialog = W.QDialog(self)
        self.settings_dialog.setWindowTitle('Concept · 速度设置')
        self.settings_dialog.setMinimumWidth(420)
        settings = W.QVBoxLayout(self.settings_dialog)
        settings.setContentsMargins(16, 16, 16, 16)
        settings.setSpacing(9)
        settings.addWidget(label('速度设置','heading'))
        settings_button = W.QPushButton('调参…')
        settings_button.clicked.connect(self.settings_dialog.show)
        toolbar.addWidget(settings_button)
        self.spins={}
        self.readbacks={}
        self.edited_speeds = set()
        self.confirmed_speeds = {}
        self.parameter_reads = {}
        for key,text,maximum in [('target','目标速度',6.),('limit','自动驾驶限速',5.)]:
            line=W.QHBoxLayout()
            field=label(text)
            field.setMinimumWidth(94)
            line.addWidget(field)
            spin=W.QDoubleSpinBox()
            spin.setRange(.05,maximum)
            spin.setSingleStep(.1)
            spin.setSuffix(' m/s')
            spin.setValue(4.)
            spin.valueChanged.connect(lambda value, key=key: self.edited_speeds.add(key))
            spin.lineEdit().textEdited.connect(lambda text, key=key: self.edited_speeds.add(key))
            line.addWidget(spin)
            self.spins[key]=spin
            btn=W.QPushButton('保存')
            btn.clicked.connect(lambda checked=False,key=key:self.request(key))
            self.buttons[key]=btn
            line.addWidget(btn)
            feedback=label('当前 —','muted')
            self.readbacks[key]=feedback
            line.addWidget(feedback)
            settings.addLayout(line)
        self.pace = label('规划按目标和限速中较低的值计算。','muted')
        self.pace.setWordWrap(True)
        settings.addWidget(self.pace)
        limit_tip = '限制自动驾驶发送的速度指令。修改后自动驾驶暂停，需要手动继续。手柄控制使用自己的设置。'
        self.spins['limit'].setToolTip(limit_tip)
        self.buttons['limit'].setToolTip(limit_tip)
        settings.addWidget(label('修改限速后，需手动继续自动驾驶。','muted'))
        self.speed_feedback = label('', 'muted')
        self.speed_feedback.setWordWrap(True)
        settings.addWidget(self.speed_feedback)
        self.action_feedback=label('','muted')
        self.action_feedback.setWordWrap(True)
        stack.addWidget(self.action_feedback)
        self.gap_notice = label('', 'muted')
        self.gap_notice.setWordWrap(True)
        self.gap_notice.hide()
        stack.addWidget(self.gap_notice)

        metrics=W.QHBoxLayout()
        self.metrics={}
        for key,text in [('speed','轮速估计 · m/s'),('cmd','门控输出 · m/s'),('legal','可行 / 候选'),('lag','距拍摄 · ms')]:
            box,col=card()
            col.addWidget(label(text,'muted'))
            v=label('—','metric')
            col.addWidget(v)
            self.metrics[key]=v
            metrics.addWidget(box)
        root.insertLayout(2, metrics)

        self.log_dialog = W.QDialog(self)
        self.log_dialog.setWindowTitle('Concept · 事件记录')
        self.log_dialog.resize(880, 430)
        log_button = W.QPushButton('事件记录…')
        log_button.clicked.connect(self.log_dialog.show)
        toolbar.addWidget(log_button)
        logs = W.QVBoxLayout(self.log_dialog)
        logs.addWidget(label('事件记录','heading'))
        self.table=W.QTreeWidget()
        self.table.setHeaderLabels(['来源','事件','首次时间','末次时间','次数'])
        self.table.setRootIsDecorated(False)
        self.table.setAlternatingRowColors(True)
        self.table.header().setStretchLastSection(False)
        self.table.header().setSectionResizeMode(1,W.QHeaderView.Stretch)
        for i,w in [(0,75),(2,96),(3,96),(4,52)]: self.table.setColumnWidth(i,w)
        logs.addWidget(self.table)
        self.refresh(0.,0.)

    def request(self, key):
        if self.dispatch is not None:
            self.dispatch(key,self.spins[key].value() if key in self.spins else None)

    def show_speed(self, key, value):
        self.readbacks[key].setText('当前 '+number(value, 2))
        spin = self.spins[key]
        spin.setEnabled(not self.busy and not self.offline)
        if value is None:
            return
        if self.confirmed_speeds.get(key) == value:
            self.confirmed_speeds.pop(key)
            self.edited_speeds.discard(key)
        if key not in self.edited_speeds:
            previous = spin.blockSignals(True)
            spin.setValue(value)
            spin.blockSignals(previous)

    def request_bag(self):
        if self.bag_action and not self.offline:
            self.bag_action()

    def show_bag(self, status, path, button, enabled, color):
        self.bag_status.setText(status)
        self.bag_status.setStyleSheet('color:'+color+';font-weight:600;')
        self.bag_status.setToolTip(status)
        if self.bag_path.text() != path:
            self.bag_path.setText(path)
            self.bag_path.setCursorPosition(len(path))
            self.bag_path.setToolTip(path)
        self.bag_toggle.setText(button)
        self.bag_toggle.setEnabled(enabled and not self.offline)

    def log_event(self, channel, text):
        self.journal.add(channel,text,datetime.now().strftime('%H:%M:%S'))
        self.table.clear()
        for r in self.journal.rows:
            W.QTreeWidgetItem(self.table,[r['channel'],r['message'],r['first'],r['last'],str(r['count'])])

    def image(self, image, source, receipt):
        self.pix=QtGui.QPixmap.fromImage(image)
        self.image_feed.put({},source,receipt)
        self.draw_image()

    def draw_image(self):
        if self.pix is not None and not self.pix.isNull():
            self.camera.setPixmap(self.pix.scaled(self.camera.size(),QtCore.Qt.KeepAspectRatio,QtCore.Qt.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self,'camera'):
            self.draw_image()

    def show_decision(self, decision, age):
        status = decision.get('status')
        cmd = decision.get('command') or {}
        speed = cmd.get('linear_x_mps')
        yaw = cmd.get('angular_z_radps')
        detail = reason(decision.get('reason') or '等待驾驶节点给出路线')
        description = ''
        if age is not None and age > 1.:
            action, detail = '决策已过期', '超过 1 秒没有收到新鲜决策'
        elif status == 'blocked':
            action = '控制输出受阻'
        elif status in ('invalid', 'late'):
            action = '决策无效' if status == 'invalid' else '决策超时'
        elif status in ('ok', 'shadow') and speed is not None and yaw is not None:
            if speed > 0:
                action = '前进'
            elif speed < 0:
                action = '慢速倒车'
            else:
                action = '仅转向' if yaw else '停车等待'
            if status == 'shadow':
                action = '试算 ' + action
            direction = '左转' if yaw > 0 else '右转'
            steer = f'{direction} {abs(yaw):.2f} rad/s' if abs(yaw) >= .005 else '转向 0.00 rad/s'
            description = f'请求速度 {speed:.2f} m/s   ·   {steer}'
            if decision.get('selected_index') is not None:
                description += f"   ·   候选 #{decision['selected_index']}"
            elif decision.get('selected_path'):
                description += '   ·   恢复路线'
            elif 'selected_path' in decision:
                description += '   ·   当前无选中路径'
            planner_reason = decision.get('planner_reason')
            if planner_reason and planner_reason != decision.get('reason'):
                detail += ' · ' + reason(planner_reason)
        else:
            action = '等待决策'
        self.action.setText('规划 · ' + action)
        self.detail.setText(detail)
        self.command.setText(description)

    def show_output(self, gate, steady):
        action, motion, angle = output_motion(gate)
        color = {'forward': '#185c48', 'turn': '#795012', 'stop': '#882c30',
                 'unknown': '#42524e'}.get(motion)
        if motion == 'reverse':
            color = '#b1282e' if int(steady * 2) % 2 else '#741b24'
        if color != self.output_color:
            self.decision_surface.setStyleSheet(
                f'QFrame#decisionSurface {{background:{color};border-radius:12px;}}'
                'QFrame#decisionSurface QLabel {color:#f7faf8;}'
                'QFrame#decisionSurface QWidget {background:transparent;}')
            self.output_color = color
        self.motion.setText(action)
        self.arrow.show_motion(motion, angle)
        self.output.setText(f"{number(gate.get('linear_mps'))} m/s  ·  ω {number(gate.get('yaw_rps'))} rad/s")

    def refresh(self, ros, steady):
        ga=self.gate.age(ros,steady)
        da=self.decision.age(ros,steady)
        ca=self.image_feed.age(ros,steady)
        live=ga is not None and ga<=.5
        valid=da is not None and da<=1.
        g=self.gate.value if live else {}
        unstamped_error = (self.decision.stamp is None and self.decision.received is not None
                           and 0. <= steady-self.decision.received <= 1.
                           and self.decision.value.get('status') == 'invalid')
        d=self.decision.value if valid or unstamped_error else {}
        self.connection.setText('离线预览' if self.offline else '门控在线' if live else '门控失联' if self.gate.received is not None else '等待门控')
        self.mode.setText(MODE.get(g.get('mode'),'电机状态未知'))
        self.connection.setStyleSheet('' if live or self.offline else 'background:#f5dfd7;color:#894328;')
        self.reason.setText(('门控原因 · '+reason(g.get('reason','—'))) if live else '无法确认电机输出状态')
        self.camera_time.setText('等待图像' if ca is None else ('图像已过期 · ' if ca>1. else '图像 · ')+f'{ca:.2f} s')
        self.show_output(g, steady)
        self.show_decision(d, da)
        self.metrics['speed'].setText(number(d.get('speed_estimate_mps')))
        self.metrics['cmd'].setText(number(g.get('linear_mps')))
        self.metrics['legal'].setText(f"{d['feasible_count']} / {len(d.get('candidates',[]))}" if 'feasible_count' in d else '—')
        self.metrics['lag'].setText(number(da*1000,0) if da is not None else '—')
        for key, feed, value in (('target', self.decision, d.get('target_speed_mps')),
                                 ('limit', self.gate, g.get('max_auto_speed_mps'))):
            if key in self.parameter_reads:
                receipt, actual = self.parameter_reads[key]
                if steady - receipt <= 2. and (value is None or receipt >= feed.received):
                    value = actual
            self.show_speed(key, value)
        effective = d.get('effective_speed_mps') if live else None
        self.pace.setText(f'本帧规划参考 {effective:.2f} m/s · 目标和限速取较低值'
                          if effective is not None else '规划按目标和限速中较低的值计算。')
        self.action_feedback.setText(self.pending_text)
        self.speed_feedback.setText(self.pending_text)
        if self.image_gap():
            gap = self.decision.value.get('gap_reason') or 'image gap over 0.5 s; pause the gate and restart the driver'
            self.gap_notice.setText(reason(gap) + '。点继续自动驾驶会先清空累计地图并重启规划。')
        self.gap_notice.setVisible(self.image_gap())
        for key,button in self.buttons.items():
            ok=live and not self.busy and not self.offline
            if key=='enable': ok=ok and not g.get('output_enabled') and not g.get('manual_held') and not g.get('estop')
            if key=='resume': ok=ok and g.get('output_enabled') and g.get('paused') and not g.get('manual_held') and not g.get('estop')
            if key=='disable': ok=not self.offline
            if key=='pause': ok=not self.offline
            button.setEnabled(bool(ok))

    def image_gap(self):
        return self.decision.value.get('image_gap_reset_required') is True
