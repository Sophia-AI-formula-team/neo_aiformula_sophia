import json
import threading
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from concept_drive.node import DriverNode
rclpy.init()
driver = DriverNode()
peer = Node('restart_service_probe')
executor = MultiThreadedExecutor(num_threads=3)
executor.add_node(driver)
executor.add_node(peer)
frames = 0
def work():
    global frames
    if not driver.restart_requested.is_set():
        with driver.planning_state:
            frames += 1
            time.sleep(.12)
driver.create_timer(.001,work)
client = peer.create_client(Trigger,'/concept/drive/restart')
thread = threading.Thread(target=executor.spin)
thread.start()
try:
    assert client.wait_for_service(timeout_sec=3)
    results=[]
    for _ in range(8):
        time.sleep(.16)
        before = driver.session
        sent=time.monotonic()
        f=client.call_async(Trigger.Request())
        done=threading.Event()
        f.add_done_callback(lambda _:done.set())
        assert done.wait(2)
        assert f.result().success and driver.session != before
        results.append(round((time.monotonic()-sent)*1000))
    print(json.dumps(dict(restart_ms=results,frames=frames)))
finally:
    executor.shutdown()
    thread.join()
    peer.destroy_node()
    driver.destroy_node()
    rclpy.shutdown()
