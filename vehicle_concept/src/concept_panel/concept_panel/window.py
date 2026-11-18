from datetime import datetime

from .model import Feed, Journal
from .qt import QtCore, QtGui, QtWidgets as W

MODE = {'disabled':'输出禁用', 'paused':'自动已暂停', 'manual':'手柄接管', 'auto':'自动驾驶', 'estop':'软件急停'}
STATUS = {'ok':'有效', 'invalid':'无效', 'shadow':'影子模式', 'waiting':'等待输入', 'late':'超时'}
REASONS={'startup':'初始化','output_enabled':'等待恢复自动驾驶','output_disabled':'电机输出已禁用',
         'operator_pause':'操作员已暂停','manual_takeover':'手柄已接管','manual_released':'手柄已松开，等待继续',
         'joy_timeout':'手柄数据超时','graph_conflict':'控制链存在冲突','motor_timeout':'电机转换输出超时',
         'vehicle_state_timeout':'车辆状态超时','speed_limit_changed':'速度上限已更改，等待继续',
         'auto_timeout':'自动指令超时','auto_speed_limit':'自动指令超过速度上限',
         'waiting_new_auto':'等待恢复后的新指令','auto_command_received':'已收到新的自动指令','feasible':'已选择可行轨迹',
         'no_feasible_trajectory':'当前无可行轨迹','unobserved_road':'缺少道路观测'}

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
        self.resize(1120, 765)
        root = W.QVBoxLayout(self)
        root.setContentsMargins(22,16,22,16)
        root.setSpacing(10)
        title = W.QHBoxLayout()
        title.addWidget(label('驾驶控制台','title'))
        title.addWidget(label('驾驶策略  /  V6.26','muted'))
        title.addStretch()
        self.connection = label('等待门控','badge')
        title.addWidget(self.connection)
        root.addLayout(title)
        columns = W.QHBoxLayout()
        columns.setSpacing(12)
        root.addLayout(columns, 3)
        camera, cam = card()
        columns.addWidget(camera, 3)
        bar = W.QHBoxLayout()
        bar.addWidget(label('摄像头与规划叠图','heading'))
        bar.addStretch()
        self.camera_time = label('等待图像','muted')
        bar.addWidget(self.camera_time)
        cam.addLayout(bar)
        self.camera = label('等待摄像头叠图','camera')
        self.camera.setMinimumSize(260,160)
        self.camera.setSizePolicy(W.QSizePolicy.Ignored, W.QSizePolicy.Ignored)
        self.camera.setAlignment(QtCore.Qt.AlignCenter)
        cam.addWidget(self.camera,1)
        self.detail = label('决策状态未知','muted')
        self.detail.setWordWrap(True)
        cam.addWidget(self.detail)
        recording = W.QHBoxLayout()
        recording.addWidget(label('ROS 数据包','heading'))
        self.bag_status = label('未录制')
        self.bag_status.setObjectName('bag_status')
        recording.addWidget(self.bag_status)
        recording.addStretch()
        self.bag_toggle = W.QPushButton('开始录包')
        self.bag_toggle.setObjectName('bag_toggle')
        self.bag_toggle.setToolTip('保存原生决策和车况。motor_can 是电机换算后的内部 CAN；录包排除原始相机图像和门控后的物理 CAN 指令。')
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
        controls.setMinimumWidth(330)
        columns.addWidget(controls, 2)
        row = W.QHBoxLayout()
        row.addWidget(label('控制权限','heading'))
        row.addStretch()
        self.mode = label('未知','badge')
        row.addWidget(self.mode)
        stack.addLayout(row)
        self.reason = label('等待门控数据','muted')
        self.reason.setWordWrap(True)
        stack.addWidget(self.reason)
        grid = W.QGridLayout()
        self.buttons = {}
        for i,(key,text,style) in enumerate([
                ('enable','启用电机输出','primary'),('disable','禁用电机输出','stop'),
                ('pause','暂停自动驾驶',None),('resume','继续自动驾驶',None),
                ('restart','暂停并重启驾驶节点',None)]):
            button=W.QPushButton(text)
            if style: button.setObjectName(style)
            button.clicked.connect(lambda checked=False, key=key: self.request(key))
            self.buttons[key]=button
            grid.addWidget(button,i//2,i%2,1,2 if key=='restart' else 1)
        stack.addLayout(grid)
        stack.addWidget(label('速度设置','heading'))
        self.spins={}
        self.readbacks={}
        for key,text,maximum in [('target','规划目标',6.),('limit','自动上限',5.)]:
            line=W.QHBoxLayout()
            field=label(text)
            line.addWidget(field)
            spin=W.QDoubleSpinBox()
            spin.setRange(.05,maximum)
            spin.setSingleStep(.1)
            spin.setSuffix(' m/s')
            spin.setValue(3.4 if key=='target' else 1.)
            line.addWidget(spin)
            self.spins[key]=spin
            btn=W.QPushButton('应用')
            btn.clicked.connect(lambda checked=False,key=key:self.request(key))
            self.buttons[key]=btn
            line.addWidget(btn)
            feedback=label('当前 —','muted')
            self.readbacks[key]=feedback
            line.addWidget(feedback)
            stack.addLayout(line)
        self.action_feedback=label('','muted')
        self.action_feedback.setWordWrap(True)
        stack.addWidget(self.action_feedback)
        stack.addStretch()

        metrics=W.QHBoxLayout()
        self.metrics={}
        for key,text in [('speed','轮速估计 · m/s'),('cmd','门控线速度 · m/s'),('legal','可行轨迹 / 候选'),('lag','决策数据龄 · ms')]:
            box,col=card()
            col.addWidget(label(text,'muted'))
            v=label('—','metric')
            col.addWidget(v)
            self.metrics[key]=v
            metrics.addWidget(box)
        root.addLayout(metrics)
        box,logs=card()
        logs.addWidget(label('事件记录','heading'))
        self.table=W.QTreeWidget()
        self.table.setHeaderLabels(['来源','事件','首次时间','末次时间','次数'])
        self.table.setRootIsDecorated(False)
        self.table.setAlternatingRowColors(True)
        self.table.setMinimumHeight(104)
        self.table.header().setStretchLastSection(False)
        self.table.header().setSectionResizeMode(1,W.QHeaderView.Stretch)
        for i,w in [(0,75),(2,96),(3,96),(4,52)]: self.table.setColumnWidth(i,w)
        logs.addWidget(self.table)
        root.addWidget(box,1)
        self.refresh(0.,0.)

    def request(self, key):
        if self.dispatch is not None:
            self.dispatch(key,self.spins[key].value() if key in self.spins else None)

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

    def refresh(self, ros, steady):
        ga=self.gate.age(ros,steady)
        da=self.decision.age(ros,steady)
        ca=self.image_feed.age(ros,steady)
        live=ga is not None and ga<=.5
        valid=da is not None and da<=1.
        g=self.gate.value if live else {}
        d=self.decision.value if valid else {}
        self.connection.setText('离线预览' if self.offline else '门控在线' if live else '门控失联' if self.gate.received is not None else '等待门控')
        self.mode.setText(MODE.get(g.get('mode'),'未知'))
        self.connection.setStyleSheet('' if live or self.offline else 'background:#f5dfd7;color:#894328;')
        self.reason.setText(('门控原因 · '+reason(g.get('reason','—'))) if live else '无法确认电机输出状态')
        self.camera_time.setText('等待图像' if ca is None else ('图像已过期 · ' if ca>1. else '图像 · ')+f'{ca:.2f} s')
        self.detail.setText('决策 · '+STATUS.get(d.get('status'),d.get('status','未知'))+'   /   '+reason(d.get('reason') or '等待有效决策'))
        self.metrics['speed'].setText(number(d.get('speed_estimate_mps')))
        self.metrics['cmd'].setText(number(g.get('linear_mps')))
        self.metrics['legal'].setText(f"{d['feasible_count']} / {len(d.get('candidates',[]))}" if 'feasible_count' in d else '—')
        self.metrics['lag'].setText(number(da*1000,0) if da is not None else '—')
        self.readbacks['target'].setText('当前 '+number(d.get('target_speed_mps'),1))
        self.readbacks['limit'].setText('当前 '+number(g.get('max_auto_speed_mps'),1))
        self.action_feedback.setText(self.pending_text)
        for key,button in self.buttons.items():
            ok=live and not self.busy and not self.offline
            if key=='enable': ok=ok and not g.get('output_enabled') and not g.get('manual_held') and not g.get('estop')
            if key=='resume': ok=ok and g.get('output_enabled') and g.get('paused') and not g.get('manual_held') and not g.get('estop')
            if key=='disable': ok=not self.offline
            if key=='pause': ok=not self.offline
            button.setEnabled(bool(ok))
