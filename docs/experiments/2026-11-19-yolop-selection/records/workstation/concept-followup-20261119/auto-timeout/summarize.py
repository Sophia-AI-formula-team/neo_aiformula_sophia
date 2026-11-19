import collections
import json
import sys
from pathlib import Path

root=Path(sys.argv[1])
p=json.loads((root/'probe.json').read_text())
c=json.loads((root/'camera.json').read_text()) if (root/'camera.json').exists() else {}
def dist(values):
    v=sorted(values)
    if not v: return None
    return [round(v[int((len(v)-1)*q)],2) for q in (.5,.95,1.)]
metrics=collections.defaultdict(list)
for k,d in p['dec'].items():
    if d['status']!='ok':continue
    age=d['capture_age_s']*1000
    metrics['capture_to_driver'].append(age-d['processing_ms'])
    metrics['driver_ms'].append(d['processing_ms'])
    metrics['decision_age_ms'].append(age)
    if k in c and 'pub' in c[k]:
        r=c[k]
        metrics['capture_to_vision'].append((r['start']-int(k))/1e6)
        metrics['publish_to_driver'].append(age-d['processing_ms']-(r['pub']-int(k))/1e6)
        for name in ('rectify','predict','encode','visibility'):metrics[name].append(r[name])
        if k in p['raw']:
            metrics['capture_to_raw_sub'].append((p['raw'][k]-int(k))/1e6)
            metrics['raw_sub_to_vision'].append((r['start']-p['raw'][k])/1e6)
for k,(t,speed) in p['cmd'].items():metrics['command_age_ms'].append((t-int(k))/1e6)
cmd=sorted((t,int(k)) for k,(t,v) in p['cmd'].items())
age_next=[(v-u)/1e6 for (_,u),(v,_) in zip(cmd,cmd[1:])]
result=dict(percentiles='median,p95,max',timing_ms={k:dist(v) for k,v in metrics.items()},
  decisions=dict(collections.Counter((d['status']+' '+d['reason']) for d in p['dec'].values())),cmd=len(cmd),
  positive=sum(v[1]>0 for v in p['cmd'].values()), valid_states=sum(s[1] for s in p['states']),
  right_source=sorted(set(s[2] for s in p['states'])),joy=len(p['joy']),auto_windows=[])
for i,t in enumerate(p['events']):
    end=p['events'][i+1] if i+1<len(p['events']) else p['gate'][-1][0]+.001
    if p.get('fault'):end=min(end,p['fault']['suspend_at'])
    rows=[r for r in p['gate'] if t+.12<=r[0]<end and r[2]!='operator_pause']
    after=[(arr,k) for arr,k in cmd if t<=arr/1e9<end]
    local_ages=[(v-u)/1e6 for (_,u),(v,_) in zip(after,after[1:])]
    result['auto_windows'].append(dict(secs=round(end-t,2), cmd=len(after),
                     stale_intervals=sum(age>500 for age in local_ages),
                     worst_interval_ms=round(max(local_ages,default=0.),2),
                     modes=dict(collections.Counter(r[1] for r in rows)),
                     reasons=dict(collections.Counter(r[2] for r in rows)),positive_status=sum(r[3]>0 for r in rows)))
if p.get('fault'):result['deliberate_stream_loss']=p['fault']
result['expiry_gap_count']=sum(x>500 for x in age_next)
result['max_last_capture_to_next_receipt_ms']=max(age_next,default=0)
print(json.dumps(result,indent=2))
