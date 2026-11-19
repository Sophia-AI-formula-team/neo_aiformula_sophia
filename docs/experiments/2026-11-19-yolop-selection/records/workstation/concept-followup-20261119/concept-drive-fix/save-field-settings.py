from pathlib import Path
import os, json, time
from datetime import datetime
import rclpy
from rcl_interfaces.srv import GetParameters,SetParameters
from rcl_interfaces.msg import Parameter,ParameterType,ParameterValue
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

assert os.environ['ROS_DOMAIN_ID']=='206' and os.environ['ROS_LOCALHOST_ONLY']=='1'
rclpy.init()
n=rclpy.create_node('verify_saved_field_parameters')
states=[]
n.create_subscription(String,'/concept/gate/status',lambda m: states.append(json.loads(m.data)),qos_profile_sensor_data)
def wait(predicate):
    end=time.monotonic()+12
    while not predicate() and time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.08)
    assert predicate()
def call(kind,node,name,req):
    c=n.create_client(kind,'/concept/'+node+'/'+name)
    assert c.wait_for_service(timeout_sec=6.)
    f=c.call_async(req)
    wait(f.done)
    return f.result()
try:
    wait(lambda: states and states[-1]['mode']=='disabled')
    assert n.count_publishers('/aiformula_control/motor_controller/reference_signal')==0
    assert not any(name in ('motor_controller','socket_can_sender') for name,ns in n.get_node_names_and_namespaces())
    rows=[]
    for node,key in [('drive','target_speed_mps'),('gate','max_auto_speed_mps')]:
        req=GetParameters.Request(names=[key,'speed_settings_directory'])
        before=call(GetParameters,node,'get_parameters',req).values
        folder=Path(before[1].string_value)
        assert folder==Path.home()/'.config/concept/speed'
        assert before[0].double_value==4.
        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=4.)
        reply=call(SetParameters,node,'set_parameters',SetParameters.Request(parameters=[Parameter(name=key,value=value)]))
        assert reply.results[0].successful
        after=call(GetParameters,node,'get_parameters',req).values[0].double_value
        saved=json.loads((folder/(key+'.json')).read_text())
        assert saved==after==4.
        rows.append(dict(node=node,parameter=key,ros_value=after,saved_value=saved))
    print(json.dumps(dict(time=datetime.now().astimezone().isoformat(timespec='milliseconds'),gate=states[-1]['mode'],parameters=rows)))
finally:
    n.destroy_node()
    rclpy.shutdown()
