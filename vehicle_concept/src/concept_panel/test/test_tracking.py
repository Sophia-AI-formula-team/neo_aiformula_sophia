from types import SimpleNamespace

from concept_panel.tracking import TrackingFrames, used_points


def decision(mode, sec, ns):
    points = [[3., -.6]] if mode == 'pure_pursuit' else [[2., -.6], [3., 0.], [4., .6]]
    return dict(stamp=dict(sec=sec,nanosec=ns), session='test', selected_index=0, status='ok',
                command=dict(linear_x_mps=.8,angular_z_radps=.2),
                tracking_controller=dict(mode=mode, active=True, owner='candidate',
                                         checked_speed_mps=.8,checked_yaw_right_radps=-.2,
                                         points_zx_m=points,target_zx_m=points[0]))


def test_only_recognized_tracking_points_and_visibility():
    t=decision('lya',1,0)['tracking_controller']
    points=used_points(t)
    assert [p[0] for p in points]==['A','B','C']
    assert points[0][1][0] < 330 <= points[2][1][0]
    assert points[0][1][1] > points[2][1][1]
    clipped=dict(t,points_zx_m=[t['target_zx_m'],[.3,0],[2,99]])
    assert used_points(clipped)[1:]==(('B',None),('C',None))
    assert not used_points(dict(t,active=False))
    assert not used_points(dict(t,owner='edge_recovery'))
    assert not used_points(dict(t,target_zx_m=[5., 1.]))
    assert not used_points(dict(t,points_zx_m=[t['target_zx_m'],[1., float('nan')],[2.,3.]]))


def test_no_transfer_of_points_across_modes_or_failed_decisions():
    buf=TrackingFrames()
    a=decision('lya',10,1)
    buf.put(a)
    assert buf.frames[(10,1)][0][0]=='A'
    b=decision('pure_pursuit',10,2)
    buf.put(b)
    assert (10,1) not in buf.frames and buf.frames[(10,2)][0][0]=='P'
    buf.put(dict(b,status='late'))
    assert not buf.frames
    buf.put(b)
    buf.put(dict(b,command=dict(linear_x_mps=0.)))
    assert not buf.frames
    buf.put(dict(b,command=dict(linear_x_mps=.8,angular_z_radps=-.2)))
    assert not buf.frames


def test_panel_matches_ns_and_immediately_hides_takeover_stop_and_reset(monkeypatch):
    import json
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen')
    from concept_panel.main import Controller
    from concept_panel.qt import QtGui, QtWidgets
    from concept_panel.window import Window
    import concept_panel.main as main
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    w=Window()
    control=Controller.__new__(Controller)
    control.window=w
    # Adjacent nanoseconds at a Unix timestamp round to the same float.
    sec=1795080000
    control.ros=lambda:float(sec)
    monkeypatch.setattr(main.time,'monotonic',lambda:20.)
    gate=dict(t=sec,mode='auto',output_enabled=True,linear_mps=.8,yaw_rps=.1)
    emit=lambda channel, data:control.message(channel,SimpleNamespace(data=json.dumps(data)))
    emit('门控',gate)
    image=QtGui.QImage(640,360,QtGui.QImage.Format_RGB32)
    image.fill(QtGui.QColor('gray'))
    w.image(image,float(sec),20.,key=(sec,1),ros=float(sec))
    emit('决策',decision('lya',sec,2))
    assert not w.tracking_marks
    w.image(image,float(sec),20.,key=(sec,2),ros=float(sec))
    assert [p[0] for p in w.tracking_marks]==['A','B','C']
    assert '目标点' in w.tracking_label.text()
    marked=w.camera.pixmap().toImage()
    emit('门控',dict(gate,mode='manual'))
    assert not w.tracking_marks and '手柄' in w.tracking_label.text()
    assert marked != w.camera.pixmap().toImage()
    emit('门控',gate)
    assert not w.tracking_marks
    w.image(image,float(sec),20.,key=(sec,3),ros=float(sec))
    emit('决策',decision('pure_pursuit',sec,3))
    assert [p[0] for p in w.tracking_marks]==['P']
    emit('决策',dict(decision('pure_pursuit',sec,4),tracking_controller=dict(
            mode='pure_pursuit',active=False,owner='stop'),command=dict(linear_x_mps=0.)))
    assert not w.tracking_marks and '停车' in w.tracking_label.text()
    w.image(image,float(sec),20.,key=(sec,5),ros=float(sec))
    emit('决策',decision('lya',sec,5))
    assert w.tracking_marks
    w.show_tracking(float(sec),20.6)
    assert not w.tracking_marks
    emit('门控',gate)
    w.reset_tracking()
    assert not w.tracking.frames and w.camera_key is None
    w.close()
    assert app
