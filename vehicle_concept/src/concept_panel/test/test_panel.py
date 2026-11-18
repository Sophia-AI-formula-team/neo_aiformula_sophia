from types import SimpleNamespace
from concept_panel.model import Feed, Journal, Action, stamp_seconds


def test_restart_requires_acknowledged_pause():
    a=Action('restart')
    assert list(a.steps)==['pause','restart'] and not a.completed(False)
    a=Action('restart')
    assert a.completed(True)=='restart' and a.completed(True) is None


def test_causal_and_receipt_freshness():
    f=Feed()
    assert f.age(0.,0.) is None
    f.put({},10.,20.)
    assert f.age(9.,20.) is None
    assert f.age(10.,21.)==1.
    assert f.age(12.,20.)==2.
    assert stamp_seconds(dict(sec=3,nanosec=500000000))==3.5


def test_interleaved_repeat_events_preserve_first_last_and_occurrences():
    j=Journal()
    j.add('门控','paused','09:00:00')
    j.add('决策','等待','09:00:02')
    j.add('门控','paused','09:00:05')
    assert j.rows[1]==dict(channel='门控',message='paused',first='09:00:00',last='09:00:05',count=2)
    j.add('门控','manual','09:00:06')
    j.add('门控','paused','09:00:07')
    assert j.rows[0]['count']==1


def test_offline_qt_render_flags_and_bgr_with_stride(monkeypatch):
    import os
    os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
    from concept_panel.main import image_from_ros
    from concept_panel.qt import QtWidgets
    from concept_panel.window import Window
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    w=Window()
    w.gate.put(dict(mode='auto',output_enabled=True,linear_mps=.87),10.,20.)
    w.refresh(10.,20.)
    assert w.connection.text()=='门控在线' and w.metrics['cmd'].text()=='0.87'
    w.refresh(10.,20.6)
    assert w.connection.text()=='门控失联' and w.metrics['cmd'].text()=='—'
    assert not w.buttons['enable'].isEnabled()
    assert w.buttons['disable'].isEnabled()
    image=image_from_ros(SimpleNamespace(encoding='bgr8',width=1,height=2,step=4,
                        data=bytes([1,2,3,99,11,22,33,99])))
    assert image.pixelColor(0,1).getRgb()[:3]==(33,22,11)
    w.close()
    assert app is not None


def test_controller_restarts_only_after_successful_pause():
    from concept_panel.main import Controller
    for outcome in (False, True):
        controller=Controller.__new__(Controller)
        sent=[]
        controller.window=SimpleNamespace(log_event=lambda *args: None,busy=True,pending_text='')
        controller.pending=Action('restart')
        controller.future=SimpleNamespace(done=lambda:True,
            result=lambda:SimpleNamespace(success=outcome,message=''))
        controller.issue=lambda:sent.append(controller.pending.steps[0])
        controller.poll()
        assert sent==(['restart'] if outcome else [])


def test_controller_pause_timeout_never_restarts():
    from concept_panel.main import Controller
    controller=Controller.__new__(Controller)
    controller.window=SimpleNamespace(log_event=lambda *args: None,busy=True,pending_text='')
    controller.pending=Action('restart')
    controller.future=SimpleNamespace(done=lambda:False,cancel=lambda:None)
    controller.deadline=0
    controller.poll()
    assert controller.pending is None and '超时' in controller.window.pending_text
