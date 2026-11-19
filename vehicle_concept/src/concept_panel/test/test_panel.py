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
    w.show()
    w.gate.put(dict(mode='paused', output_enabled=True, paused=True, linear_mps=0.),10.,20.)
    w.decision.put(dict(status='invalid', image_gap_reset_required=True),10.,20.)
    w.refresh(10.,20.)
    assert w.buttons['resume'].text()=='继续自动驾驶'
    assert w.buttons['resume'].isEnabled() and w.gap_notice.isVisible()
    assert '图像采集间隔' in w.gap_notice.text()
    w.decision.put(dict(status='invalid', image_gap_reset_required=True,
                        gap_reason='planning gap over 0.5 s after missing synchronized wheel/gyro feedback'),10.,20.)
    w.refresh(10.,20.)
    assert '轮速与陀螺反馈未配对' in w.gap_notice.text()
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
    w.show_output(dict(mode='auto', linear_mps=1., yaw_rps=.1), 0.)
    assert -25. < w.arrow.degrees < -20.
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


def test_embedded_map_resets_on_restart_ack_and_discards_old_snapshot(monkeypatch, tmp_path):
    import base64
    from concurrent.futures import Future
    import json
    import zlib
    monkeypatch.setenv('QT_QPA_PLATFORM', 'offscreen')
    from concept_panel.main import Controller, LapMapUpdates
    from concept_panel.qt import QtGui, QtWidgets
    from concept_panel.window import Window

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = Window()
    window.show()
    app.processEvents()
    panel = window.lap_map
    assert panel.isVisible() and panel.window() is window
    empty = base64.b64encode(zlib.compress(b'')).decode('ascii')
    snapshot = dict(schema='concept.lap-map.v1', session='old', stamp=1, cell_m=.1, distance_m=0.,
                    road=empty, lines=empty, track=empty, pose=[0., 0., 0.],
                    static=[], moving=[], openings=[])
    panel.show_map(json.dumps(snapshot))
    path = tmp_path / 'map.png'
    monkeypatch.setattr(QtWidgets.QFileDialog, 'getSaveFileName', lambda *args: (str(path), 'PNG'))
    panel.save_button.click()
    assert QtGui.QImage(str(path)).width() >= 800

    updates = LapMapUpdates.__new__(LapMapUpdates)
    updates.panel = panel
    old_response = Future()
    old_response.set_result(SimpleNamespace(success=True, message=json.dumps(snapshot)))
    updates.pending = old_response
    updates.sent_at = 0.
    new_response = Future()
    updates.client = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda _: new_response)
    updates.request = object
    controller = Controller.__new__(Controller)
    controller.window, controller.lap_map = window, updates
    for success in (False, True):
        controller.pending = Action('restart')
        controller.pending.completed(True)  # pause acknowledged
        controller.future = Future()
        controller.future.set_result(SimpleNamespace(success=success, message=''))
        controller.poll()
        assert panel.save_button.isEnabled() == (not success)
    assert panel.canvas.raster is None and panel.canvas.data is None and panel.key is None
    updates.refresh(.1)
    assert panel.key is None and updates.pending is new_response
    snapshot.update(session='new', stamp=2)
    new_response.set_result(SimpleNamespace(success=True, message=json.dumps(snapshot)))
    updates.refresh(.2)
    assert panel.key == ('new', 2) and panel.save_button.isEnabled()
    window.close()


def test_speed_fields_follow_readbacks_without_discarding_edits(monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM', 'offscreen')
    from concept_panel.qt import QtWidgets
    from concept_panel.window import Window
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    w = Window()
    spin = w.spins['target']
    assert spin.value() == w.spins['limit'].value() == 4.
    w.show_speed('target', 2.25)
    assert spin.value() == 2.25
    spin.setValue(2.75)
    w.busy = True
    w.show_speed('target', 2.25)
    assert spin.value() == 2.75 and not spin.isEnabled()
    w.confirmed_speeds['target'] = 2.75
    w.busy = False
    w.show_speed('target', 2.75)
    w.show_speed('target', 3.)
    assert spin.isEnabled() and spin.value() == 3.
    assert w.readbacks['target'].text() == '当前 3.00'
    w.close()
    assert app
