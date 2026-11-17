"""Post-run geometry diagnostic; never imported by controller.
Compares emitted mapped nodes with recorded Unity vehicle-centre positions.
This is not a ground-truth junction-centre error or a localization accuracy test.
"""
import argparse,json,math
from pathlib import Path

def nearest(p,trajectory):
    best=None
    for a,b in zip(trajectory,trajectory[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1]; d=dx*dx+dy*dy
        t=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/d)) if d else 0
        q=(a[0]+t*dx,a[1]+t*dy); distance=math.hypot(p[0]-q[0],p[1]-q[1])
        if best is None or distance<best[0]: best=(distance,q)
    return best

def compute(path):
    trajectory=[]; snapshot=None
    with path.open(encoding='utf-8-sig') as f:
      for line in f:
        row=json.loads(line); x,z=row.get('x'),row.get('z')
        if isinstance(x,(int,float)) and isinstance(z,(int,float)) and math.isfinite(x) and math.isfinite(z): trajectory.append((z,-x))
        s=row.get('semantic') or {}; m=s.get('map_data') or {}; status=s.get('status') or {}; fusion=status.get('map_fusion') or {}
        if snapshot is None and len(trajectory)>1 and m.get('graph_frame')=='odom' and fusion.get('available') is True and m.get('graph_nodes'):
          snapshot=(row['wall'],list(trajectory),m['graph_nodes'],status.get('topology_alignment'),fusion)
    if snapshot is None: return {'available':False,'reason':'no accepted mapped graph in odom frame'}
    wall,track,nodes,alignment,fusion=snapshot; out=[]
    for n in nodes:
      if not isinstance(n,dict) or not all(isinstance(n.get(k),(int,float)) and math.isfinite(n[k]) for k in ('x','y')): continue
      dist,q=nearest((n['x'],n['y']),track)
      out.append({'node_id':str(n.get('id')),'kind':n.get('kind'),'mapped_odom_m':[n['x'],n['y']], 'nearest_recorded_vehicle_center_odom_m':list(q),'distance_to_recorded_vehicle_center_path_m':dist})
    return {'available':True,'metric_definition':'Euclidean distance from mapped graph node to closest segment of recorded Unity vehicle-centre trajectory up to first accepted match; NOT node-to-true-junction error','control_input':False,'frame':'odom = (Unity world z, -Unity world x)','source_motion':str(path.resolve()),'first_match_wall_s':wall,'trajectory_points':len(track),'fusion':fusion,'alignment':alignment,'nodes':out}

def main():
 p=argparse.ArgumentParser();p.add_argument('motion',type=Path);p.add_argument('output',type=Path);args=p.parse_args()
 data=compute(args.motion)
 args.output.parent.mkdir(parents=True,exist_ok=True)
 with args.output.open('x',encoding='utf8') as f:json.dump(data,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
 print(json.dumps({'available':data['available'],'nodes':len(data.get('nodes',[])),'output':str(args.output)},ensure_ascii=False))
if __name__=='__main__':main()
