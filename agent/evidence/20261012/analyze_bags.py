from pathlib import Path
import json,collections,math,statistics,struct
from datetime import datetime,timezone,timedelta
import yaml
ROOT=Path(__file__).resolve().parent;TZ=timezone(timedelta(hours=9))
def iso(ns):return datetime.fromtimestamp(ns/1e9,TZ).isoformat(timespec='milliseconds')
def count(rows,key):return dict(collections.Counter(str(x['data'].get(key)) for x in rows))
def gap(rows):
 ds=[(b['t_ns']-a['t_ns'])/1e9 for a,b in zip(rows,rows[1:])]
 return dict(count=len(rows),median_gap_sec=statistics.median(ds) if ds else None,max_gap_sec=max(ds) if ds else None)
reports=[];events=[]
for p in sorted((ROOT/'bags').glob('*/selected_messages.jsonl')):
 rows=[json.loads(x) for x in p.read_text().splitlines()];by=collections.defaultdict(list)
 for x in rows:by[x['topic']].append(x)
 meta=yaml.safe_load((p.parent/'metadata.yaml').read_text())['rosbag2_bagfile_information']
 start=meta['starting_time']['nanoseconds_since_epoch'];duration=meta['duration']['nanoseconds']/1e9
 maps=by['/semantic_drive/map_status'];motors=by['/aiformula_control/motor_controller/gate_status'];odom=by['/aiformula_sensing/gyro_odometry_publisher/odom'];cmd=by['/aiformula_control/game_pad/cmd_vel'];a3=by['/aiformula_perception/yolop_a3/detections'];session=by['/semantic_drive/motor_session']
 r=dict(bag=p.parent.name,start_jst=iso(start),duration_sec=round(duration,3),recorded_messages=meta['message_count'],topic_recording={k:gap(v) for k,v in by.items()},a3_colors=count(a3,'traffic_light_state'))
 if odom:
  r['odometry']=dict(first_xy=[odom[0]['data']['x'],odom[0]['data']['y']],last_xy=[odom[-1]['data']['x'],odom[-1]['data']['y']],sampled_path_length_m=sum(math.hypot(b['data']['x']-a['data']['x'],b['data']['y']-a['data']['y']) for a,b in zip(odom,odom[1:])),max_abs_v=max(abs(x['data']['v']) for x in odom),nonzero_v_samples=sum(abs(x['data']['v'])>.02 for x in odom))
 if maps:
  r['semantic']={key:count(maps,key) for key in ['phase','traffic_mode','traffic_reason','traffic_light','traffic_light_raw','traffic_stop_required','reason','velocity_gate_reason']}
  r['semantic'].update(max_samples=max(x['data']['recorded_samples'] for x in maps),max_distance_m=max(x['data']['recorded_distance_m'] for x in maps),reference_ever_available=any(x['data']['reference_length_m'] is not None for x in maps),input_never_received_samples=sum(x['data']['odom_age_sec'] is None and all(v is None for v in x['data']['point_age_sec'].values()) for x in maps))
  previous=None
  for m in maps:
   d=m['data'];state=(d['traffic_mode'],d['traffic_reason'])
   if state!=previous:
    motor=next((x for x in motors if m['t_ns']<=x['t_ns']<=m['t_ns']+2e9),None)
    od=next((x for x in odom if m['t_ns']<=x['t_ns']<=m['t_ns']+2e9),None)
    e=dict(bag=p.parent.name,time_jst=iso(m['t_ns']),source_time_jst=iso(round(d['ros_time_sec']*1e9)),map=state,light=d['traffic_light'],raw=d['traffic_light_raw'],stop=d.get('traffic_stop_required'),motor=({'time_jst':iso(motor['t_ns']),**motor['data']} if motor else None),odom=({'time_jst':iso(od['t_ns']),**od['data']} if od else None));events.append(e);previous=state
 if motors:
  r['motor_reasons']=count(motors,'reason');r['motor_nonzero_commands']=sum(any(abs(x['data']['command'][k])>1e-9 for k in (0,1)) for x in motors)
  r['automatic_red_hold_nonzero_CAN']=sum(x['data']['reason'].startswith('semantic_hold:red') and any(x['data']['can_data']) for x in motors)
 if session:
  ages=[(x['t_ns']-x['data']['stamp_ns'])/1e9 for x in session]
  r['session']=dict(identities=dict(collections.Counter(x['data']['session'] for x in session)),min_receipt_age_sec=min(ages),max_receipt_age_sec=max(ages),median_receipt_age_sec=statistics.median(ages),active=count(session,'active'))
 reports.append(r)
reports.sort(key=lambda r:r['start_jst'])
(ROOT/'bag_summary.json').write_text(json.dumps(dict(method='Offline selected-message analysis. Bag record cadence is not publisher rate. Sparse recorded poses approximate path length only; no causal latency or physical stop proof.',bags=reports),ensure_ascii=False,indent=2)+'\n')
(ROOT/'traffic_events.json').write_text(json.dumps(events,ensure_ascii=False,indent=2)+'\n')
for r in reports:
 sem=r.get('semantic',{});print(r['bag'],r['start_jst'],r['duration_sec'],'map_m',round(sem.get('max_distance_m',0),2),'samples',sem.get('max_samples'), 'closure',sem.get('reference_ever_available'),'motor',r.get('motor_reasons',{}))
print('TRAFFIC EVENTS')
for e in events:
 if e['map'][1]!='traffic_idle':print(json.dumps(e,ensure_ascii=False))
