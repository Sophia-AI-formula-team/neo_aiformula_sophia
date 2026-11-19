"""Offscreen panel, actual Qt signals, installed ROS parameter servers, no sensors or motor."""
import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node

from concept_panel.qt import QtWidgets
from concept_panel.main import Controller
from concept_panel.window import Window


def event(stage, **details):
    print(json.dumps(dict(time=datetime.now().astimezone().isoformat(timespec='milliseconds'),
                          stage=stage, **details),ensure_ascii=False),flush=True)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('scenario')
    parser.add_argument('expected',nargs=2,type=float)
    parser.add_argument('--save',nargs=2,type=float)
    parser.add_argument('--persistent',action='store_true')
    args=parser.parse_args()
    names=('target_speed_mps','max_auto_speed_mps')
    keys=('target','limit')
    directory=Path(os.environ['XDG_CONFIG_HOME'])/'concept'/'speed'
    assert os.environ['ROS_LOCALHOST_ONLY']=='1' and int(os.environ['ROS_DOMAIN_ID'])>=200
    assert '/tmp/' in str(directory) and os.environ['QT_QPA_PLATFORM']=='offscreen'
    rclpy.init(args=[])
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication(['concept-settings-test'])
    node=Node('concept_panel')
    window=Window()
    control=Controller(node,window)
    window.show()
    app.processEvents()
    clients=[node.create_client(GetParameters,'/concept/'+service+'/get_parameters') for service in ('drive','gate')]
    def wait(test, condition):
        end=time.monotonic()+16.
        while time.monotonic()<end:
            for _ in range(5):
                rclpy.spin_once(node,timeout_sec=.003)
            app.processEvents()
            control.update()
            if test(): return
        raise AssertionError(condition+'; panel='+window.pending_text+'; spins='+str(values()))
    def values():
        return [window.spins[k].value() for k in keys]
    def params():
        fs=[c.call_async(GetParameters.Request(names=[name,'speed_settings_directory'])) for c,name in zip(clients,names)]
        wait(lambda: all(f.done() for f in fs),'GET both ROS parameters')
        replies=[f.result().values for f in fs]
        actual_directory=str(directory) if args.persistent else ''
        assert [v[1].string_value for v in replies] == [actual_directory]*2
        return [v[0].double_value for v in replies]
    def show_settings():
        button=next(b for b in window.findChildren(QtWidgets.QPushButton) if b.text().startswith('调参'))
        button.click()
        wait(window.settings_dialog.isVisible,'Open settings dialog via Qt')
    try:
        wait(lambda: all(c.service_is_ready() for c in clients) and window.gate.value.get('output_enabled') is False,
             'Discover disabled gate & real parameter servers')
        assert not node.count_publishers('/aiformula_control/motor_controller/reference_signal')
        assert not any(name in ('socket_can_sender','socket_can_receiver','motor_controller','zed_node','joy_node')
                       for name,ns in node.get_node_names_and_namespaces())
        show_settings()
        def shows(wanted):
            return (values()==wanted and [window.readbacks[k].text() for k in keys]==
                    ['当前 %.2f' % v for v in wanted])
        wait(lambda: shows(args.expected),'Initial editor readback from ROS')
        assert params()==args.expected
        event(args.scenario+' open',editor=values(),service=params(),file_setting_enabled=args.persistent,
              physical_gate=False)
        if args.save:
            for key,wanted in zip(keys,args.save):
                window.spins[key].setValue(wanted)
                wait(window.buttons[key].isEnabled,'Save button enabled')
                assert window.buttons[key].text()=='保存'
                window.buttons[key].click()
                wait(lambda: control.pending is None and '已确认' in window.pending_text,'Qt apply to '+key)
            wait(lambda: shows(args.save),'Both displayed current values confirmed')
            assert params()==args.save
            if args.persistent:
                assert [json.loads((directory/(n+'.json')).read_text()) for n in names] == args.save
            event(args.scenario+' clicked',editor=values(),service=params())
            window.settings_dialog.close()
            app.processEvents()
            assert not window.settings_dialog.isVisible()
            show_settings()
            wait(lambda: values()==args.save,'Dialog reopened with applied settings')
            event(args.scenario+' reopened',editor=values(),service=params())
        if args.scenario=='first-save':
            out=Path(os.environ['SETTINGS_PROBE_LOG'])/'saved-speeds.png'
            assert window.settings_dialog.grab().save(str(out))
    finally:
        window.settings_dialog.close()
        window.close()
        app.processEvents()
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
