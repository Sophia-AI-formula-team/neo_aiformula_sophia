from pathlib import Path
from types import SimpleNamespace


def test_recording_control_stays_available_without_a_gate(monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM', 'offscreen')
    from concept_panel.main import RecordingControls
    from concept_panel.qt import QtWidgets
    from concept_panel.window import Window

    class Recorder:
        root = Path('/bags')

        def set_status(self, state, path='', error='', messages=None):
            self.snapshot = SimpleNamespace(state=state, path=path, error=error, messages=messages)
            return self.snapshot

        def status(self):
            return self.snapshot

        def start(self):
            return self.set_status('starting', '/bags/run')

        def stop(self):
            return self.set_status('stopping', '/bags/run')

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = Window()
    recorder = Recorder()
    recorder.set_status('idle')
    control = RecordingControls(recorder, window)
    window.busy = True
    window.refresh(1., 1.)
    assert window.bag_toggle.isEnabled() and not window.buttons['enable'].isEnabled()
    window.bag_toggle.click()
    assert window.bag_status.text() == '正在启动'
    recorder.set_status('recording', '/bags/run')
    control.refresh()
    assert window.bag_toggle.text() == '停止录包' and window.bag_path.text() == '/bags/run'
    window.bag_toggle.click()
    assert not window.bag_toggle.isEnabled() and window.buttons['pause'].isEnabled()
    recorder.set_status('saved', '/bags/run', messages=17)
    control.refresh()
    assert '17' in window.bag_status.text() and window.bag_path.text() == '/bags/run'
    recorder.set_status('error', '/bags/run', '磁盘已满')
    control.refresh()
    assert window.bag_status.toolTip() == '磁盘已满' and window.bag_toggle.isEnabled()
    assert '磁盘已满' in window.table.topLevelItem(0).text(1)
    offline = Window(offline=True)
    RecordingControls(recorder, offline)
    assert not offline.bag_toggle.isEnabled()
    window.close()
    offline.close()
    assert app is not None
