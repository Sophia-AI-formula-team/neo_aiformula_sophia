"""Installer transactions in a temporary fake workspace; colcon is mocked."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('bundle_installer', Path(__file__).with_name('install.py'))
installer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(installer)


class InstallerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.bundle, self.ws, self.backup = root/'bundle', root/'ws', root/'backup'
        self.names = ('control/motor_controller/motor_controller/gate.py',
            'perception/semantic_planning_system/semantic_local_planner_ros/semantic_local_planner_ros/map_assist.py')
        self.manifest = dict(files={}, guards={}, versions={'motor_controller':'0.4.1','semantic_local_planner_ros':'0.2.2'})
        for name in self.names:
            for path, content in ((self.ws/'src/aiformula'/name, b'old'), (self.bundle/'payload'/name, b'new')):
                path.parent.mkdir(parents=True, exist_ok=True);path.write_bytes(content)
            self.manifest['files'][name] = dict(before=hashlib.sha256(b'old').hexdigest(),after=hashlib.sha256(b'new').hexdigest())
        (self.bundle/'manifest.json').write_text(json.dumps(self.manifest))
        for section in ('build','install'):
            for package in installer.PACKAGES:
                path=self.ws/section/package;path.mkdir(parents=True);(path/'old_marker').write_text('preserve')
        self.idle=patch.object(installer,'live_processes',return_value=[]);self.idle.start();self.addCleanup(self.idle.stop)

    def test_source_drift_refuses_without_writing(self):
        path=self.ws/'src/aiformula'/self.names[0];path.write_text('user edit')
        with self.assertRaisesRegex(RuntimeError,'source changed'):
            installer.preflight(self.bundle,self.ws)
        self.assertEqual(path.read_text(),'user edit');self.assertFalse(self.backup.exists())

    def test_running_stack_refuses_without_stopping_it(self):
        with patch.object(installer,'live_processes',return_value=['motor running']):
            with self.assertRaisesRegex(RuntimeError,'Exit the driving'):
                installer.preflight(self.bundle,self.ws)
        self.assertFalse(self.backup.exists())

    def test_build_failure_restores_source_build_and_install(self):
        with patch.object(installer,'build',side_effect=RuntimeError('compile failed')):
            with self.assertRaisesRegex(RuntimeError,'compile failed'):
                installer.apply(self.bundle,self.ws,self.backup,self.manifest,{})
        for name in self.names:self.assertEqual((self.ws/'src/aiformula'/name).read_bytes(),b'old')
        for section in ('build','install'):
            for package in installer.PACKAGES:
                self.assertEqual((self.ws/section/package/'old_marker').read_text(),'preserve')
        self.assertEqual(json.loads((self.backup/'receipt.json').read_text())['status'],'rolled_back_after_failure')

    def test_success_verifies_modules_and_keeps_backups(self):
        def fake_build(ws,env,log):
            for package,name in zip(installer.PACKAGES,self.names):
                (ws/'build'/package).mkdir(parents=True)
                dest=ws/'install'/package/'lib/python3.8/site-packages'/package/Path(name).name
                dest.parent.mkdir(parents=True);dest.symlink_to(ws/'src/aiformula'/name)
        with patch.object(installer,'build',side_effect=fake_build):
            receipt=installer.apply(self.bundle,self.ws,self.backup,self.manifest,{})
        self.assertFalse(receipt['nodes_started']);self.assertFalse(receipt['hardware_commands_sent'])
        for name in self.names:
            self.assertEqual((self.ws/'src/aiformula'/name).read_bytes(),b'new')
            self.assertEqual((self.backup/'source'/name).read_bytes(),b'old')


    def test_egg_link_install_resolves_fresh_python_import(self):
        def fake_build(ws,env,log):
            for package in installer.PACKAGES:
                (ws/'build'/package).mkdir(parents=True)
                (ws/'install'/package/'lib/python3.8/site-packages').mkdir(parents=True)
        resolved=[json.dumps(str(self.ws/'src/aiformula'/name)).encode() for name in self.names]
        with patch.object(installer,'build',side_effect=fake_build), patch.object(installer.subprocess,'check_output',side_effect=resolved):
            receipt=installer.apply(self.bundle,self.ws,self.backup,self.manifest,{})
        self.assertEqual(receipt['status'],'installed')


if __name__ == '__main__':
    unittest.main(verbosity=2)
