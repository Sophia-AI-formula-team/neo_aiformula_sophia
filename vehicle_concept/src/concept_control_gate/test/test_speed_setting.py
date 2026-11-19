from types import SimpleNamespace

import pytest

from concept_control_gate.speed_setting import SpeedSetting


class Node:
    def __init__(self, directory='', **overrides):
        self.params = dict(speed_settings_directory=str(directory), **overrides)

    def declare_parameter(self, name, default):
        return SimpleNamespace(value=self.params.get(name, default))


def test_independent_settings_restart_recovery_and_replay(tmp_path):
    target, limit = 'target_speed_mps', 'max_auto_speed_mps'
    driver = SpeedSetting(Node(tmp_path), target, 6.)
    gate = SpeedSetting(Node(tmp_path), limit, 5.)
    assert driver.initial == gate.initial == 4.
    driver.save(2.35)
    gate.save(1.75)
    assert SpeedSetting(Node(tmp_path), target, 6.).initial == 2.35
    assert SpeedSetting(Node(tmp_path), limit, 5.).initial == 1.75
    with pytest.raises(ValueError):
        gate.save(float('nan'))
    assert SpeedSetting(Node(tmp_path), limit, 5.).initial == 1.75
    gate.path.write_text('{')
    with pytest.raises(ValueError, match='Invalid'):
        SpeedSetting(Node(tmp_path), limit, 5.)
    recovered = SpeedSetting(Node(tmp_path, max_auto_speed_mps=3.), limit, 5.)
    assert recovered.initial == 3.
    recovered.save(4.)
    assert SpeedSetting(Node(tmp_path), limit, 5.).initial == 4.
    isolated = SpeedSetting(Node(), target, 6.)
    assert isolated.initial == 4.
    isolated.save(.5)
    assert SpeedSetting(Node(tmp_path), target, 6.).initial == 2.35


def test_failed_write_keeps_last_setting(tmp_path, monkeypatch):
    import os
    setting = SpeedSetting(Node(tmp_path), 'target_speed_mps', 6.)
    setting.save(2.)
    def unavailable(source, destination):
        raise OSError('disk is read only')
    monkeypatch.setattr(os, 'replace', unavailable)
    with pytest.raises(OSError, match='read only'):
        setting.save(4.)
    assert SpeedSetting(Node(tmp_path), 'target_speed_mps', 6.).initial == 2.
    assert list(tmp_path.iterdir()) == [setting.path]
