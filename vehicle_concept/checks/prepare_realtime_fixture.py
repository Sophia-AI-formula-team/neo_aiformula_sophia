"""Prepare only Unity camera recordings; the synthetic receipt cannot register a vehicle."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def main():
    p=argparse.ArgumentParser()
    p.add_argument('run',type=Path,help='Private Unity run with runs/ID.json and ID/decisions.jsonl')
    p.add_argument('output',type=Path)
    p.add_argument('--frames',type=int,default=300)
    a=p.parse_args()
    name=a.run.name
    raw=json.loads((a.run/'runs'/(name+'.json')).read_bytes())
    measured=raw['cameraMountCalibrationStart'][2]
    if abs(measured-.655)>.001 or abs(raw['cameraMountCalibrationEnd'][2]-measured)>.001:
        raise ValueError('Unexpected Unity camera mount')
    dst=a.output
    dst.mkdir(parents=True,exist_ok=False)
    rows=[]
    with (a.run/name/'decisions.jsonl').open(encoding='utf8') as f:
        for line in f:
            rec=json.loads(line)
            i=rec['frame']
            sensor=rec['decision']['sensor']
            if i!=len(rows) or abs(sensor['t']-i*.2)>.005:
                raise ValueError('Non-contiguous 5 Hz recording')
            source=a.run/name/'frames'/f'{i:05d}_rgb.png'
            shutil.copyfile(source,dst/source.name)
            rows.append(dict(file=source.name,archive_frame=i,sensor=sensor,
                             sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
            if len(rows)==a.frames:break
    if len(rows)!=a.frames:raise ValueError('Insufficient source captures')
    (dst/'fixture.json').write_text(json.dumps(dict(lap=name,archive_hz=5,frames=rows)))
    # This is a virtual pinhole camera in a rendered world. Generate exact ground-plane
    # correspondences from its renderer, never use this receipt for a vehicle camera.
    f,cx,cy=254.39162,330.01303,181.14964
    marks=[[cx+f*x/z,cy+f*measured/z,x,z] for x,z in [(-1,2),(1,2),(-2,9),(2,9),(0,4)]]
    receipt=dict(image_topic='/aiformula_sensing/zed_node/left_image/undistorted',
        frame_id='unity_fixed_receiver_optical',mount_id='Unity-fixed-frustum-bench-only',
        width=640,height=360,projection=[f,0.,cx,0.,0.,f,cy,0.,0.,0.,1.,0.],ground_marks=marks,
        provenance='Unity AIFormulaRgbCameraTopic.RepositoryProjection and MotorControllerValidation scene mount')
    (dst/'virtual-camera.json').write_text(json.dumps(receipt))


if __name__=='__main__':
    main()
