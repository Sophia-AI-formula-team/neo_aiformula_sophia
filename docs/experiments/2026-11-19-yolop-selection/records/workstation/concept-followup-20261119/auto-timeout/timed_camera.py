import json, os, time
from pathlib import Path
import rclpy
from concept_yolop import node as v
rows={}
stage={}
def wrap(name, fn):
    def call(*args, **kwargs):
        t=time.perf_counter()
        result=fn(*args, **kwargs)
        stage[name]=(time.perf_counter()-t)*1e3
        return result
    return call
class Timed(v.CameraNode):
    def on_image(self,msg):
        k=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        stage.clear()
        stage['start']=time.time_ns()
        super().on_image(msg)
        rows[k]=stage.copy()
rclpy.init()
n=Timed()
v.encode_frame=wrap('encode',v.encode_frame)
n.calibration.rectify=wrap('rectify',n.calibration.rectify)
n.calibration.restrict_predictions=wrap('visibility',n.calibration.restrict_predictions)
n.predictor.predict=wrap('predict',n.predictor.predict)
publish=n.frames.publish
def mark(msg):
    publish(msg)
    stage['pub']=time.time_ns()
n.frames.publish=mark
try:
    rclpy.spin(n)
except (KeyboardInterrupt,Exception) as e:
    print(str(e),flush=True)
finally:
    Path(os.environ['RUN_DIR'],'camera.json').write_text(json.dumps(rows))
    n.destroy_node()
