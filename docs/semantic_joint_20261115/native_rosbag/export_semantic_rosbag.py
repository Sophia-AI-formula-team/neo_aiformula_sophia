"""Export existing Unity records to native MapAssist ROS 2 input topics."""
from pathlib import Path
import csv, hashlib, json, math, shutil
from collections import Counter
import cv2
import numpy as np
import yaml
from rosbags.rosbag2 import Writer, Reader
from rosbags.typesys import Stores, get_typestore

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'deliverables/semantic_native_topics_inner_outer_20261116_v2'
STORE=get_typestore(Stores.ROS2_HUMBLE)
T=STORE.types
cv2.setNumThreads(1)

def make(name,*args): return T[name](*args)
def ns(sec): return round(sec*1_000_000_000)
def tm(n): return make('builtin_interfaces/msg/Time',n//10**9,n%10**9)
def header(n,frame): return make('std_msgs/msg/Header',tm(n),frame)
def vector(x=0.,y=0.,z=0.): return make('geometry_msgs/msg/Vector3',float(x),float(y),float(z))
def twist(v,w): return make('geometry_msgs/msg/Twist',vector(v),vector(0,0,w))
def pose(x,y,yaw): return make('geometry_msgs/msg/Pose',make('geometry_msgs/msg/Point',float(x),float(y),0.),make('geometry_msgs/msg/Quaternion',0.,0.,math.sin(yaw/2),math.cos(yaw/2)))
def jdump(path,value): path.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
def sha(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
 return h.hexdigest()

def main():
 if OUT.exists():raise RuntimeError('Refuse existing delivery')
 OUT.mkdir(parents=True)
 bag=OUT/'bag'
 topics={
 '/aiformula_sensing/gyro_odometry_publisher/odom':'nav_msgs/msg/Odometry',
 **{f'/original_processed_point_{a}':'geometry_msgs/msg/Pose2D' for a in 'abc'},
 '/lane_stop_flag':'std_msgs/msg/Bool','/map_assist/branch_active':'std_msgs/msg/Bool',
 '/map_assist/reset':'std_msgs/msg/Bool','/map_assist/traffic_light_input':'std_msgs/msg/String',
 '/map_assist/controller_cmd_vel':'geometry_msgs/msg/Twist',
 '/camera/image_raw':'sensor_msgs/msg/Image','/camera/image_raw/compressed':'sensor_msgs/msg/CompressedImage',
 '/camera/camera_info':'sensor_msgs/msg/CameraInfo','/semantic_replay/session':'std_msgs/msg/String',
 '/semantic_replay/camera_calibration':'std_msgs/msg/String',
 '/recorded/map_assist/status':'std_msgs/msg/String','/recorded/map_assist/map_data':'std_msgs/msg/String',
 '/recorded/map_assist/path':'nav_msgs/msg/Path',
 '/recorded/map_assist/cmd_vel':'geometry_msgs/msg/Twist',
 **{f'/recorded/processed_point_{a}':'geometry_msgs/msg/Pose2D' for a in 'abc'}}
 counts=Counter(); image_index=[];vision_index=[];segments=[];sources=[]; offset=1.
 with Writer(bag,version=8) as writer:
  conn={topic:writer.add_connection(topic,typ,typestore=STORE) for topic,typ in topics.items()}
  def emit(topic,t,msg):
   writer.write(conn[topic],t,STORE.serialize_cdr(msg,topics[topic]));counts[topic]+=1
  for run_name in ['inner_joint_07','outer_joint_03']:
   run=ROOT/'runs'/run_name
   case_dir=OUT/'source_metadata'/run_name;case_dir.mkdir(parents=True)
   for name in ['result.json','configuration.json','camera_calibration.json','events.jsonl','integration_hashes.json','integration_hashes_after.json']:
    shutil.copyfile(run/name,case_dir/name)
   ready=json.loads((run/'worker/ready.json').read_text())
   result=json.loads((run/'result.json').read_text(encoding='utf-8-sig'))
   calib=json.loads((run/'camera_calibration.json').read_text())
   params=dict(ready['parameters']);params['output_cmd_vel_topic']='/semantic_replay/recomputed_cmd_vel'
   params['use_sim_time']=True
   params_topomap=params.get('topomap_path')
   params['topomap_path']=''  # Resolve installed original package map; portable CLI may select bundled exact copy.
   (OUT/'replay_map_assist.yaml').write_text(yaml.safe_dump({'map_assist':{'ros__parameters':params}},sort_keys=False),encoding='utf-8')
   if params_topomap and Path(params_topomap).is_file():shutil.copyfile(params_topomap,OUT/'aiformula_topomap.osm')
   traces={}
   with (run/'trace.jsonl').open(encoding='utf-8-sig') as f:
    for line in f:
     row=json.loads(line);seq=row['sequence']
     val={k:row.get(k) for k in ['candidate','detected_light','frame_sim_time','signal_panel','age']}
     if seq in traces and traces[seq]!=val:raise ValueError('Conflicting detector snapshots')
     traces[seq]=val
   recorded_frames=[json.loads(line) for line in (run/'frames.jsonl').read_text().splitlines()]
   unique_frames={}
   for frame in recorded_frames:
    # Harness may log the same received camera frame repeatedly. Publish each capture once.
    key=(frame['sequence'],frame['frame_sim_time'],frame['rgb'])
    unique_frames.setdefault(key,frame)
   frames=list(unique_frames.values())
   pending=[]
   for frame in frames:
    pending.append((ns(offset+frame['frame_sim_time']),'image',frame))
   # A session reset is an explicit export boundary between two originally separate workers.
   pending.append((ns(offset-.1),'reset',None))
   pending.sort(key=lambda x:x[0]);pi=0
   frame_counter=0;last_flags={};last_map=-math.inf;last_revision=None;accepted=0;votes=0;last_seq=-1;run_counts=Counter()
   emit('/semantic_replay/session',ns(offset-.1),make('std_msgs/msg/String',json.dumps({'case':run_name,'offset_sec':offset,'source_clock':'original sim_time + offset','minimum_laps':2,'reset_event':'synthetic bag boundary; original worker cold start'},ensure_ascii=False)))
   emit('/semantic_replay/camera_calibration',ns(offset-.1),make('std_msgs/msg/String',json.dumps(calib)))
   def pending_event(t,kind,frame):
    nonlocal frame_counter
    if kind=='reset':emit('/map_assist/reset',t,make('std_msgs/msg/Bool',True));return
    path=run/frame['rgb'];data=path.read_bytes();image=cv2.imdecode(np.frombuffer(data,dtype=np.uint8),cv2.IMREAD_COLOR)
    if image is None:raise ValueError(str(path))
    height,width=image.shape[:2];h=header(t,'camera_optical_frame')
    emit('/camera/image_raw',t,make('sensor_msgs/msg/Image',h,height,width,'bgr8',0,width*3,image.reshape(-1)))
    emit('/camera/image_raw/compressed',t,make('sensor_msgs/msg/CompressedImage',h,'bgr8; jpeg compressed bgr8',np.frombuffer(data,dtype=np.uint8)))
    k=np.asarray(calib['K'],dtype=np.float64);p=np.asarray([k[0],k[1],k[2],0.,k[3],k[4],k[5],0.,k[6],k[7],k[8],0.],dtype=np.float64)
    info=make('sensor_msgs/msg/CameraInfo',h,height,width,'',np.array([],dtype=np.float64),k,np.eye(3).reshape(-1),p,0,0,make('sensor_msgs/msg/RegionOfInterest',0,0,0,0,False))
    emit('/camera/camera_info',t,info)
    image_index.append({'case':run_name,'detection_sequence':frame['sequence'],'bag_timestamp_ns':t,'original_capture_sec':frame['frame_sim_time'],'original_receipt_sec':frame['t'],'source':str(path),'jpeg_sha256':hashlib.sha256(data).hexdigest(),'decoded_bgr_sha256':hashlib.sha256(image.tobytes()).hexdigest(),'width':width,'height':height})
    frame_counter+=1
   source=run/'worker/semantic_joint_ticks.jsonl';digest=hashlib.sha256()
   with source.open('rb') as f:
    for line_number,line in enumerate(f,1):
     digest.update(line);row=json.loads(line);t=ns(offset+row['sim_time']);s=row['status'];o=row['odom']
     while pi<len(pending) and pending[pi][0]<=t:
      pending_event(*pending[pi]);pi+=1
     for topic,field in [('/lane_stop_flag','avoidance_active'),('/map_assist/branch_active','branch_active')]:
      value=s[field]
      if not isinstance(value,bool):raise ValueError('Unknown control ownership')
      if field not in last_flags or last_flags[field]!=value:
       emit(topic,t,make('std_msgs/msg/Bool',value));last_flags[field]=value
     vr=row['vision_result'];seq=row['last_vision_sequence']
     if vr in ['accepted','light_only_unsupported_lane','light_only_invalid_corridor']:
      if seq==last_seq:raise ValueError('Duplicate vision callback')
      last_seq=seq;v=traces.get(seq)
      if not v:raise ValueError(f'Missing actual pre-semantic candidate {run_name} {seq}')
      color=v['detected_light']
      if color not in ['red','green','unknown']:raise ValueError(color)
      emit('/map_assist/traffic_light_input',t,make('std_msgs/msg/String',color));votes+=1
      if vr=='accepted':
       abc=v['candidate']['selected_points']
       if len(abc)!=3:raise ValueError('Invalid ABC')
       for a,point in zip('abc',abc):emit('/original_processed_point_'+a,t,make('geometry_msgs/msg/Pose2D',float(point[0]),float(point[1]),float(point[2]) if len(point)>2 else 0.))
       accepted+=1
      vision_index.append({'case':run_name,'source_line':line_number,'worker_sequence':row['sequence'],'detection_sequence':seq,'bag_timestamp_ns':t,'original_capture_sec':v['frame_sim_time'],'receive_result':vr,'light':color,'ABC_published':vr=='accepted'})
     odom=make('nav_msgs/msg/Odometry',header(t,'odom'),'base_link',make('geometry_msgs/msg/PoseWithCovariance',pose(o['x'],o['y'],o['yaw']),np.zeros(36)),make('geometry_msgs/msg/TwistWithCovariance',twist(o['v'],o['w']),np.zeros(36)))
     emit('/aiformula_sensing/gyro_odometry_publisher/odom',t,odom);run_counts['odom']+=1
     if row['command_gate']['raw_command_new']:
      emit('/map_assist/controller_cmd_vel',t,twist(row['controller_v'],row['controller_w']));run_counts['controller']+=1
     emit('/recorded/map_assist/status',t,make('std_msgs/msg/String',json.dumps(s,separators=(',',':'))))
     if row['command_new']:emit('/recorded/map_assist/cmd_vel',t,twist(row['semantic_v'],row['semantic_w']))
     if row['selected_points_new']:
      for a,point in zip('abc',row['selected_points']):emit('/recorded/processed_point_'+a,t,make('geometry_msgs/msg/Pose2D',float(point[0]),float(point[1]),0.))
     m=row['map_data'];revision=m['path_revision']
     if revision!=last_revision or row['sim_time']-last_map>=1.:
      emit('/recorded/map_assist/map_data',t,make('std_msgs/msg/String',json.dumps(m,separators=(',',':'))));last_map=row['sim_time']
     if revision!=last_revision:
      poses=[make('geometry_msgs/msg/PoseStamped',header(t,'odom'),pose(p[0],p[1],0.)) for p in m['reference_points']]
      emit('/recorded/map_assist/path',t,make('nav_msgs/msg/Path',header(t,'odom'),poses));last_revision=revision
     last_row=row
   while pi<len(pending):pending_event(*pending[pi]);pi+=1
   assert votes==last_row['vision_votes_total'],(votes,last_row['vision_votes_total'])
   assert frame_counter==len(frames)
   segments.append({'case':run_name,'offset_sec':offset,'source_duration_sec':result['seconds'],'physical_laps':result['completedLaps'],'accepted':result['accepted'],'source_image_receipts':len(recorded_frames),'duplicate_image_receipts_not_republished':len(recorded_frames)-frame_counter,'exported_images':frame_counter,'accepted_ABC_events':accepted,'signal_votes':votes,'counts':dict(run_counts),'original_worker_log_sha256':digest.hexdigest(),'original_worker_log':str(source),'startup_reset_bag_sec':offset-.1})
   for rel in ['trace.jsonl','frames.jsonl','camera_calibration.json','worker/ready.json']:
    sources.append({'path':str(run/rel),'sha256':sha(run/rel)})
   offset+=result['seconds']+5.
   print(json.dumps({'exported':run_name,'images':frame_counter,'odom':run_counts['odom'],'ABC':accepted,'votes':votes}),flush=True)
 jdump(OUT/'manifest.json',{'schema':1,'format':'rosbag2 sqlite3 CDR; metadata version8','source_version':'0.4.0','source_commit':'6b71267e24e81459220c7e6d30d84c3f7a00e591','integration_commit':'149d97c0ad13b36962d91330f80481194a884f50','segments':segments,'topics':topics,'message_counts':dict(counts),'sources':sources,'camera_raw_origin':'Decoded actual detector.Input JPEG, bgr8. Original lossless pixels were not retained. Compressed topic retains exact JPEG bytes.','output_provenance':'Native inputs reconstructed from saved pre-semantic candidates at recorded worker receive times. Not a native DDS recording. Recorded outputs are isolated under /recorded.','clock':'Not stored. ros2 bag play --clock supplies /clock; stamps are continuous rebased sim time; each session starts with an explicit reset.'})
 for name,rows in [('image_index.csv',image_index),('vision_input_index.csv',vision_index)]:
  with (OUT/name).open('w',encoding='utf-8-sig',newline='') as f:
   w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
 # Full readback: deserialize every CDR message and verify every raw/JPEG image.
 expected_images={row['bag_timestamp_ns']:row for row in image_index};seen=Counter();previous=-1
 with Reader(bag) as reader:
  for c,t,data in reader.messages():
   assert t>=previous;previous=t
   msg=STORE.deserialize_cdr(data,c.msgtype);seen[c.topic]+=1
   if c.topic in ['/camera/image_raw','/camera/image_raw/compressed']:
    spec=expected_images[t];key='decoded_bgr_sha256' if c.topic=='/camera/image_raw' else 'jpeg_sha256'
    assert hashlib.sha256(msg.data.tobytes()).hexdigest()==spec[key]
    assert msg.header.stamp.sec*10**9+msg.header.stamp.nanosec==t
   if c.topic=='/aiformula_sensing/gyro_odometry_publisher/odom':
    assert msg.header.stamp.sec*10**9+msg.header.stamp.nanosec==t
    assert abs(sum(getattr(msg.pose.pose.orientation,a)**2 for a in 'xyzw')-1)<1e-10
 assert dict(seen)==dict(counts)
 db=next(bag.glob('*.db3'))
 jdump(OUT/'verification.json',{'all_messages_deserialized':sum(seen.values()),'counts':dict(seen),'timestamps_globally_sorted':True,'every_raw_image_equals_source_JPEG_decoding':True,'every_compressed_image_equals_original_JPEG':True,'native_ROS_or_DDS_execution':False,'bag_db3_bytes':db.stat().st_size,'bag_db3_sha256':sha(db)})
 print(json.dumps({'bag':str(bag),'bytes':db.stat().st_size,'messages':sum(seen.values()),'verified':True}),flush=True)

if __name__=='__main__':main()
