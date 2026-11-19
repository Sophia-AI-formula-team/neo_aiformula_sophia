"""Join the actual Unity camera, Foxy response, local wheel measurement and separate gate replay."""
import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


CASES = [('straight01', '沿直道起步'), ('outer_edge01', '靠外缘，朝外偏 12°'),
         ('inner_edge01', '靠内缘，朝内偏 12°'), ('outer_hard01', '靠外缘，朝外偏 23°')]


def read_json(path):
    return json.loads(path.read_bytes())


def read_lines(path):
    return [json.loads(line) for line in path.read_bytes().splitlines() if line.strip()]


def summarize(name, host, ros):
    exchange = read_lines(host / 'exchange.jsonl')
    # Match the whole command train, including exact per-frame wall times, across the two clocks.
    commands = [r['command'] for r in exchange]
    matches = [rows for rows in ros if [r['reply'] for r in rows] == commands]
    if len(matches) != 1:
        raise ValueError('No unique ROS session for ' + name)
    frames = []
    for i, (row, reply) in enumerate(zip(exchange, matches[0])):
        c, h, d = row['command'], row['frame'], reply['decision']
        if h != reply['input'] or c['seq'] != i or h['applied_seq'] != i-1:
            raise ValueError('Unmatched sensor or frame at ' + str(i))
        words = [c['right_can_rpm'], c['left_can_rpm']]
        if words != [row['set_right_can'], row['set_left_can']] or (i and
                [h['applied_right_can_rpm'], h['applied_left_can_rpm']] != frames[-1]['can_right_left']):
            raise ValueError('Unity command readback mismatch at ' + str(i))
        frames.append(dict(t=c['simulation_time_s'], target=c['linear_mps'], yaw_ros=c['angular_z_radps'],
                           count=c['feasible_count'], selected=c['selected_index'], ranking=d['ranking_active'],
                           reason=d['reason'], measured_mps=row['body_forward_mps'], can_right_left=words,
                           capture_to_can_ms=(row['can_set_utc_us']-row['capture_start_utc_us'])/1000.))
    result = read_json(host / 'result.json')
    return dict(name=name, duration=result['seconds'], progress_m=result['progress'],
                final_measured_mps=result['rows'][-1][4], first_out_s=result['firstCompletelyOut'],
                nominal_front_center_gaps=[[r[0], min(r[1:3])] for r in result['wheelRows']],
                zero_requests=sum(abs(r['target']) < 1e-9 for r in frames),
                applied_feedback_checks=len(frames)-1, frames=frames)


def draw_figure(cases, host, font, output):
    image = Image.new('RGB', (1320, 858), '#f3f6f8')
    draw = ImageDraw.Draw(image)
    fonts = {s: ImageFont.truetype(str(font), s) for s in (16, 18, 20, 23, 32)}
    def label(x, y, message, size=18, color='#193849'):
        draw.text((x,y), message, font=fonts[size], fill=color)
    label(30,18,'同一段赛道，四种起步姿态',32)
    label(32,67,'每场 5 秒，目标上限 2 m/s。照片取自车辆起步时的前视相机。',20)
    for i, ((key, title), run) in enumerate(zip(CASES, cases)):
        x, y = 26 + (i%2)*645, 112 + (i//2)*327
        draw.rounded_rectangle((x,y,x+626,y+306),radius=12,fill='white',outline='#d9e2e8')
        label(x+16,y+9,title,23)
        with Image.open(host / key / 'camera_0000.jpg') as src:
            image.paste(src.convert('RGB').resize((282,159),Image.Resampling.LANCZOS),(x+16,y+56))
        gx,gy,gw,gh=x+339,y+65,263,123
        for speed in (0,1,2,3):
            yy=gy+gh-speed*gh/3
            draw.line((gx,yy,gx+gw,yy),fill='#dbe4e9')
            label(gx-20, yy-10, str(speed),16)
        for key_y, color in [('target','#bf6b0c'),('measured_mps','#006a91')]:
            trace=[(gx+r['t']/5*gw, gy+gh-r[key_y]/3*gh) for r in run['frames']]
            draw.line(trace,fill=color,width=4)
        label(gx, y+191,'0 s',16);label(gx+gw-29,y+191,'5 s',16)
        label(x+17,y+226,'沿赛道前进 %.2f m    ·    请求停车 %d / 25 帧' % (run['progress_m'],run['zero_requests']),20)
        label(x+17,y+262,'5 秒末车速 %.2f m/s    ·    %s' % (round(run['final_measured_mps'],2)+0.0,
              '轮胎未完全越线' if run['first_out_s'] < 0 else '发生出界'),18)
    label(32,776,'橙线  速度指令',20,'#a35904');label(244,776,'蓝线  实测车身速度（m/s）',20,'#006a91')
    label(32,816,'出界按一侧前轮完全越过涂漆边界判定。内缘场名义轮心余量与涂漆裁判有分歧，详见报告。',18)
    image.save(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('host','ros','out','font'):
        parser.add_argument('--'+key,type=Path,required=True)
    args = parser.parse_args()
    runs = [read_lines(path) for path in args.ros.glob('*/frames.jsonl')]
    result = []
    for name, _ in CASES:
        case = summarize(name, args.host / name, runs)
        gate = read_json(args.ros / (name+'-gate.json'))
        case['gate_replay'] = {k:gate[k] for k in ('matches','commands','watchdog_stop','zero_frames_after_timeout')}
        result.append(case)
    args.out.mkdir(exist_ok=True, parents=True)
    (args.out/'unity-runs.json').write_text(json.dumps(result, indent=2)+'\n',encoding='utf8')
    draw_figure(result,args.host,args.font,args.out/'unity-comparison.png')


if __name__ == '__main__':
    main()
