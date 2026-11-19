"""Create a same-clock comparison from two real Unity runs and the matched ROS decisions."""
import argparse
from pathlib import Path
import sys

import av
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'unity1120/host'))
sys.path.insert(0, str(HERE.parents[1] / 'src/concept_visualization'))
from compare_tracking import load_projection, read_run
from concept_visualization.geometry import pixel


def draw_frame(run, i, projection, fonts):
    host = run['host'][i]
    decision = run['ros'][i]['decision']
    frame = projection.rectify(cv2.imread(str(run['folder'] / host['camera_file'])))
    if frame.shape[:2] != (360, 640):
        raise ValueError('Unexpected camera resolution')
    image = Image.new('RGB', (640, 508), '#17242e')
    image.paste(Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)), (0, 48))
    art = ImageDraw.Draw(image)
    mode = run['mode']
    art.text((15, 8), 'Pure Pursuit' if mode == 'pure_pursuit' else 'LYA', font=fonts[0], fill='white')
    art.text((436, 14), f"{host['frame']['simulation_time_s']:4.1f} s   车载相机", font=fonts[1], fill='#d7e4ed')
    track = decision['tracking_controller']
    uv = [pixel(p) for p in decision['selected_path']]
    previous = None
    for pt in uv:
        if pt is not None and 0 <= pt[0] < 640 and 0 <= pt[1] < 360:
            pt = (pt[0], pt[1]+48)
            if previous is not None:
                art.line([previous, pt], fill='#fd78b4', width=4)
            previous = pt
        else:
            previous = None
    if track['active']:
        letters = 'P' if mode == 'pure_pursuit' else 'ABC'
        for i, (letter, p) in enumerate(zip(letters, track['points_zx_m'])):
            pt = pixel(p)
            if pt is None or not (0 <= pt[0] < 640 and 0 <= pt[1] < 360):
                continue
            x, y = pt[0], pt[1]+48
            ink = '#ffe25c' if i == 0 else '#71efff'
            art.ellipse((x-6,y-6,x+6,y+6), outline='black', width=4)
            art.ellipse((x-5,y-5,x+5,y+5), outline=ink, width=2)
            art.text((x+6, y-17), letter, font=fonts[1], fill=ink, stroke_width=2, stroke_fill='#14222a')
    names = {'candidate': '追踪选中的路线', 'edge_recovery': '外缘保护接管转向', 'reverse': '倒车',
             'stop': '发出停车', 'contingency': '执行备用动作', 'unavailable': '本帧没有追踪点'}
    text = names.get(track['owner'], track['owner'])
    art.text((15, 420), text, font=fonts[0], fill='#ffdf7d')
    art.text((15, 466), f"车速 {host['body_forward_mps']:.2f} m/s     左 / 右轮指令 {host['set_left_can']} / {host['set_right_can']} RPM",
             font=fonts[1], fill='white')
    return image


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path, help='Folder with pure_pursuit, lya, ros-pure_pursuit and ros-lya')
    parser.add_argument('out', type=Path)
    parser.add_argument('--font', required=True)
    args = parser.parse_args()
    fonts = [ImageFont.truetype(args.font, size) for size in (23,18)]
    projection = load_projection(HERE.parent/'unity1120/foxy/honda.json')
    modes = ('pure_pursuit','lya')
    runs = [read_run(args.root/m/'long-bend', [args.root/('ros-'+m)], m) for m in modes]
    with av.open(str(args.out), 'w', options={'movflags': 'faststart'}) as output:
        video = output.add_stream('libx264', rate=5)
        video.width, video.height, video.pix_fmt = 1284, 508, 'yuv420p'
        for i in range(len(runs[0]['host'])):
            times = [r['host'][i]['frame']['simulation_time_s'] for r in runs]
            if abs(times[0]-times[1]) > .001:
                raise ValueError('Clips have different simulation times')
            frame = Image.new('RGB', (1284,508), '#e4edf2')
            for j, run in enumerate(runs):
                frame.paste(draw_frame(run,i,projection,fonts), (j*644,0))
            for packet in video.encode(av.VideoFrame.from_ndarray(np.asarray(frame), format='rgb24')):
                output.mux(packet)
        for packet in video.encode():
            output.mux(packet)
