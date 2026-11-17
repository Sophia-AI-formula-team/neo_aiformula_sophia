"""Post-run rendering only. Requires completed recordings; no controller imports."""
import json, math, bisect, importlib.util, argparse
from pathlib import Path
from fractions import Fraction
import av
from PIL import Image, ImageDraw, ImageFont
ROOT=Path(__file__).parent
spec=importlib.util.spec_from_file_location('media',r'E:/Mess/Projects/Programming/aiformula/workspace/semantic_joint_trial_20261116/staging/media/joint_media.py')
media=importlib.util.module_from_spec(spec);spec.loader.exec_module(media)
media.EVENTS.update({'first_lap_panel_red','first_lap_red_stopped','first_lap_panel_green'})
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def rows(p):
 with p.open(encoding='utf-8-sig') as f:return [json.loads(l) for l in f if l.strip()]
fonts={n:ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',n) for n in (14,16,18,22)}
def text(d,p,s,n=16,c='white'):d.text(p,str(s),font=fonts[n],fill=c)
def project(x,z):return (28+(x+23)*6.6,650-(z+45)*6.6)
def line(d,ps,c,w=2):
 if len(ps)>1:d.line(ps,fill=c,width=w)
def geom(d,ps,c,w=2):line(d,[project(-p[1],p[0]) for p in ps if len(p)>=2],c,w)
def main():
 parser=argparse.ArgumentParser();parser.add_argument('--cases',nargs='+',default=['inner_1118_fix05','outer_1118_fix05']);parser.add_argument('--output',default='1118_inner_outer_topology.mp4');args=parser.parse_args()
 out=ROOT/'media_1118';out.mkdir(exist_ok=True);dest=out/args.output
 if dest.exists():raise FileExistsError(dest)
 surface=read(Path(r'E:/Mess/Projects/Programming/aiformula/workspace/semantic_joint_trial_20261116/audit/evaluation_surface.json'))
 bg=Image.new('RGB',(1280,720),(17,23,31));d=ImageDraw.Draw(bg)
 for mesh in surface['meshes']:
  if mesh['category'] in ('green','paint'):
   for ring in mesh['boundaries']:line(d,[project(x,z) for x,z in ring],(40,78,56) if mesh['category']=='green' else (86,97,110))
 container=av.open(str(dest),'w');stream=container.add_stream('libx264',rate=15);stream.width=1280;stream.height=720;stream.pix_fmt='yuv420p';stream.options={'crf':'22','preset':'fast'};stream.codec_context.thread_count=2
 idx=0;manifest=[]
 for name in args.cases:
  lane='内圈' if name.startswith('inner') else '外圈'
  root=ROOT/'runs'/name;metric=read(root/'map_path_error.json') if (root/'map_path_error.json').exists() else {};data=rows(root/'motion.jsonl');events=rows(root/'events.jsonl');times=[r['wall'] for r in data];result=read(root/'result.json')
  segs=media.schedule(times[0],times[-1],events,data,30);walls=[]
  for s in segs:
   count=max(1,round((s['end']-s['start'])/s['speed']*15));walls.extend(s['start']+(s['end']-s['start'])*i/count for i in range(count))
  manifest.append({'case':name,'start_frame':idx,'frames':len(walls),'result':result,'timeline':segs})
  history=[project(r['x'],r['z']) for r in data];cache={}
  for k,wall in enumerate(walls):
   mi=max(0,bisect.bisect_right(times,wall)-1);r=data[mi];sem=r.get('semantic') or {};m=sem.get('map_data') or {};status=sem.get('status') or {};canvas=bg.copy();d=ImageDraw.Draw(canvas)
   geom(d,m.get('odom_reference_points') or [],(120,130,140),3);geom(d,m.get('reference_points') or [],(100,220,140),3)
   for p in (m.get('learned_samples') or [])[::3]:
    x,y=project(-p[1],p[0]);d.ellipse((x-1,y-1,x+1,y+1),fill=(180,140,230))
   if m.get('graph_frame')=='odom':
    nodes={str(p['id']):p for p in m.get('graph_nodes') or []}
    for a,b in m.get('graph_edges') or []:
     if str(a) in nodes and str(b) in nodes:
      a,b=nodes[str(a)],nodes[str(b)];line(d,[project(-a['y'],a['x']),project(-b['y'],b['x'])],(245,173,80))
    for key,p in nodes.items():
     x,y=project(-p['y'],p['x']);d.ellipse((x-4,y-4,x+4,y+4),fill='white');text(d,(x+5,y-15),key[-2:],14)
   geom(d,m.get('detour_points') or [],(255,120,180),4);line(d,history[:mi+1],(65,210,250))
   x,y=history[mi];yaw=math.radians(r['yaw']);d.ellipse((x-5,y-5,x+5,y+5),fill='white');d.line((x,y,x+math.sin(yaw)*12,y-math.cos(yaw)*12),fill='white',width=3)
   d.rectangle((0,0,850,45),fill=(17,23,31));text(d,(16,9),f'1118 / recorded  {lane} | 鸟瞰车辆轨迹 + semantic 建图',22)
   text(d,(16,670),'青：实际轨迹  紫：建图采样  绿：参考线  橙：拓扑  粉：规划绕行')
   text(d,(16,696),'底图：Unity 道路，仅供评价；地图节点在匹配后显示',14,(170,180,190))
   d.rectangle((728,480,840,650),fill=(24,31,42),outline=(80,90,100));text(d,(734,483),'ABC / LYA',14)
   def localpoint(p):return (784-p[1]*8,638-p[0]*14)
   candidate=r.get('candidate') or {}
   for seq,color in ((candidate.get('selected_points') or [],(150,160,170)),(sem.get('selected_points') or [],(180,140,230))):
    for p in seq:
     if len(p)>=2 and 0<=p[0]<=9 and abs(p[1])<=6:
      a,b=localpoint(p);d.ellipse((a-3,b-3,a+3,b+3),fill=color)
   target=sem.get('target') or []
   if len(target)>=2 and 0<=target[0]<=9 and abs(target[1])<=6:
    a,b=localpoint(target);d.ellipse((a-5,b-5,a+5,b+5),outline=(255,120,180),width=2)
   d.rectangle((850,0,1280,720),fill=(24,31,42));image=r.get('overlay') or r.get('rgb')
   if image and (root/image).is_file():
    if image not in cache:
     with Image.open(root/image) as im:cache={image:im.convert('RGB').resize((416,234))}
    canvas.paste(cache[image],(858,38))
   text(d,(862,8),'实际前视识别 overlay',18)
   fusion=status.get('map_fusion') or {};alignment=status.get('topology_alignment') or {};rmse=alignment.get('rmse_m')
   fields=[f"记录 {r.get('t',0):.1f}s / 已完成圈数 {r.get('completed_laps',0)}",f"实际速度 {r.get('actual_v',0):.2f} m/s",f"识别灯色：{r.get('detected_light')}",f"交通阶段：{status.get('traffic_mode')}",str(status.get('traffic_reason')),f"建图：{status.get('phase')} / {fusion.get('reason')}",f"参考线就绪：{status.get('odom_reference_ready')}",f"匹配拟合 RMSE：{rmse if rmse is not None else '尚无输出'}",'路口真实点误差：缺少权威真值，未验证',f"LYA v/w：{sem.get('controller_v',0):.2f} / {sem.get('controller_w',0):.2f}",f"semantic v/w：{sem.get('semantic_v',0):.2f} / {sem.get('semantic_w',0):.2f}"]
   for j,s in enumerate(fields):text(d,(862,285+j*27),s[:48])
   if m.get('graph_frame')=='odom' and metric.get('available'):
    ds=[n['distance_to_recorded_vehicle_center_path_m'] for n in metric['nodes'] if n.get('kind')=='outer_anchor']
    if ds:text(d,(862,578),f'锚点到实跑轨迹均值 {sum(ds)/len(ds):.3f}m',14,(100,220,140))
   ev=[e for e in events if e['wall']<=wall];text(d,(862,598),str(ev[-1].get('event','') if ev else '')[:43],14,(255,185,90))
   text(d,(862,628),'仿真理想里程计；进程内消息适配',14,(180,190,205));text(d,(862,651),'避障/分支动态输入缺失，采用原默认值',14,(255,185,90))
   if k==len(walls)-1:text(d,(862,680),'试验通过' if result.get('accepted') else '试验未通过：详见日志',18,(255,185,90))
   frame=av.VideoFrame.from_image(canvas);frame.pts=idx;frame.time_base=Fraction(1,15)
   for packet in stream.encode(frame):container.mux(packet)
   if k in (0,len(walls)//2,len(walls)-1):canvas.save(out/f'{name}_{k}.png')
   idx+=1
  print(name,len(walls),flush=True)
 for packet in stream.encode():container.mux(packet)
 container.close();(out/(dest.stem+'_timeline.json')).write_text(json.dumps({'frames':idx,'fps':15,'duration_s':idx/15,'cases':manifest},ensure_ascii=False,indent=2),encoding='utf8');print(dest,flush=True)
if __name__=='__main__':main()
