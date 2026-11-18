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


def test_primary_panel_shows_plan_separately_from_motor_permission(monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM', 'offscreen')
    from concept_panel.qt import QtWidgets
    from concept_panel.window import Window

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    w = Window()
    w.gate.put(dict(mode='disabled', output_enabled=False, linear_mps=0., yaw_rps=0.,
                    max_auto_speed_mps=1.), 10., 20.)
    decision = dict(status='ok', reason='feasible', selected_index=0, target_speed_mps=3.4,
                    effective_speed_mps=1., command=dict(linear_x_mps=.8, angular_z_radps=.2))
    w.decision.put(decision, 10., 20.)
    w.refresh(10., 20.)
    assert w.action.text() == '规划 · 前进' and w.motion.text() == '停止'
    assert w.mode.text() == '输出禁用' and w.metrics['cmd'].text() == '0.00'
    assert w.arrow.degrees is None and w.arrow.motion == 'stop'
    assert '左转' in w.command.text() and '#0' in w.command.text()
    assert '1.00 m/s' in w.pace.text()

    w.decision.put(dict(decision, status='shadow', command=dict(linear_x_mps=-.2, angular_z_radps=0.)),
                   10., 20.)
    w.refresh(10., 20.)
    assert w.action.text() == '规划 · 试算 慢速倒车' and w.arrow.motion == 'stop'
    w.refresh(11.1, 21.1)
    assert w.action.text() == '规划 · 决策已过期' and not w.command.text()
    w.decision.put(dict(status='invalid', reason='missing frame stamp'), None, 21.1)
    w.refresh(11.1, 21.1)
    assert w.action.text() == '规划 · 决策无效' and 'missing frame stamp' in w.detail.text()
    w.close()
    assert app is not None


def test_output_arrow_uses_manual_and_automatic_commands(monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM', 'offscreen')
    from concept_panel.qt import QtWidgets
    from concept_panel.window import Window

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    w = Window()
    w.show_output(dict(mode='auto', linear_mps=1., yaw_rps=1.55), 0.)
    assert w.motion.text() == '前进' and w.arrow.degrees == -90.
    w.show_output(dict(mode='manual', linear_mps=1., yaw_rps=-.5, manual_yaw_limit_rps=.5), 0.)
    assert w.arrow.degrees == 90.
    reverse = dict(mode='manual', linear_mps=-.1, yaw_rps=.2)
    w.show_output(reverse, 0.)
    color = w.output_color
    assert w.motion.text() == '倒车' and w.arrow.degrees == 180.
    w.show_output(reverse, .5)
    assert w.output_color != color
    w.show_output(dict(mode='manual', linear_mps=0., yaw_rps=-.2), 1.)
    assert w.motion.text() == '原地转向' and w.arrow.degrees == 90.
    w.show_output({}, 2.)
    assert w.motion.text() == '等待门控' and w.arrow.degrees is None
    assert not w.settings_dialog.isVisible()
    next(button for button in w.findChildren(QtWidgets.QPushButton) if button.text() == '调参…').click()
    assert w.settings_dialog.isVisible()
    w.settings_dialog.close()
    w.close()
    assert app is not None


def test_controller_pause_timeout_never_restarts():
    from concept_panel.main import Controller
    controller=Controller.__new__(Controller)
    controller.window=SimpleNamespace(log_event=lambda *args: None,busy=True,pending_text='')
    controller.pending=Action('restart')
    controller.future=SimpleNamespace(done=lambda:False,cancel=lambda:None)
    controller.deadline=0
    controller.poll()
    assert controller.pending is None and '超时' in controller.window.pending_text
