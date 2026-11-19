from pathlib import Path
import sqlite3, json,struct,collections,datetime,statistics
import numpy as np
def jst(ns):
    return datetime.datetime.fromtimestamp(ns/1e9,datetime.timezone(datetime.timedelta(hours=9))).isoformat(timespec='milliseconds')
def stats(xs):
    if not len(xs): return None
    return dict(zip(('min','median','p95','max'),np.percentile(xs,[0,50,95,100]).round(4).tolist()))
def counts(items): return dict(collections.Counter(str(k) for k in items).most_common())
def audit(p):
    d=sqlite3.connect(p.as_uri()+'?mode=ro&immutable=1',uri=True)
    def fetch(n):
        return d.execute('select timestamp,data from messages join topics on topics.id=topic_id where name=? order by timestamp',(n,))
    def string(n):
        return [(t,json.loads(b[8:7+struct.unpack('<I',b[4:8])[0]])) for t,b in fetch(n)]
    dec=string('/concept/decision'); dr=[m for _,m in dec]
    gs=[m for _,m in string('/concept/gate/status')]
    vs=[m for _,m in string('/concept/vehicle_state')]
    start,end=d.execute('select min(timestamp),max(timestamp) from messages').fetchone()
    topics=dict(d.execute('select name,count(messages.id) from topics left join messages on topic_id=topics.id group by topics.id'))
    shadow=[m for m in dr if m['status']=='shadow']
    view=[m for m in shadow if m['selected_path']]
    empty=[m for m in shadow if not m['selected_path']]
    xy=lambda r: sum(x*x for x in r[:2])**.5
    stamp=lambda m: int(m['stamp']['sec'])*10**9+int(m['stamp']['nanosec'])
    selected_trace=[(m['selected_index'],bool(m['selected_path']), m['recovery']) for m in shadow]
    gated=lambda m: (m['mode'],m['reason'],m['output_enabled'],m['paused'])
    motor=[struct.unpack_from('<6d',b,4) for _,b in fetch('/concept/internal/motor_cmd')]
    wheel=[m for m in vs if m['valid']]
    pairs=collections.Counter()
    for m in shadow:
        pairs[(str(m['recovery'])[:160],m['feasible_count']>0,m['selected_index'] is not None,bool(m['selected_path']),len(m['candidates']))]+=1
    return dict(source=p.parent.name, times=[jst(start),jst(end)],topics=topics,decisions=dict(
        states=counts((m['status'],m['reason']) for m in dr),profiles=counts(m['geometry_profile'] for m in dr),
        exact_candidate_evaluations=counts(str(m.get('candidate_evaluation')) for m in shadow),
        candidates_count=counts(len(m['candidates']) for m in shadow),feasible_counts=counts(m['feasible_count'] for m in shadow),
        indices=counts(m['selected_index'] for m in shadow),whitebox_indices=counts(m['whitebox_index'] for m in shadow),
        select_path_points=counts(len(m['selected_path']) for m in shadow),path_endpoint_distances_m=stats([xy(m['selected_path'][-1]) for m in view]),
        nonzero_planned=sum(1 for m in dr if m.get('command') and any(m['command'].values())),
        decision_out_age_ms=stats([m['capture_age_s']*1000 for m in dr if m['capture_age_s'] is not None]),
        processing_ms=stats([m['processing_ms'] for m in dr]), late_output_500ms=sum(m['capture_age_s']>.5 for m in shadow),
        path_presence_changes=sum(a[1]!=b[1] for a,b in zip(selected_trace,selected_trace[1:])),
        recovery_feasible_selectedindex_path_candidatecounts=[[list(k),n] for k,n in pairs.most_common(30)],
        no_path_sample=[dict(reason=m.get('reason'),recovery=m.get('recovery'),rejected=m.get('rejected'),candidate_count=len(m['candidates']), road_rows=len(m['road_rows']),tstamp=stamp(m)) for m in empty[:1]],
        map_counts=counts((len(m['edges']),len(m['lines']),len(m['road_rows'])) for m in shadow),
        selected_spans_jst=([[jst(t),len(m['selected_path']),m['selected_index'],m.get('recovery')] for t,m in dec if m['selected_path']][:3])),
        gate=dict(status=counts(gated(m) for m in gs), nonzero_status=sum(1 for m in gs if m['linear_mps'] or m['yaw_rps']),nonzero_private_cmd=sum(any(c) for c in motor)),
        wheel=dict(status=counts((m['valid'],m['reason']) for m in vs),nonzero_valid=sum(1 for m in wheel if m['leftRpm'] or m['rightRpm'])))
files=sorted(Path.home().glob('rosbags/concept/concept_20261119_*/*.db3'))
print(json.dumps(dict(queried=datetime.datetime.now().astimezone().isoformat(timespec='seconds'),runs=[audit(p) for p in files]),indent=2))
