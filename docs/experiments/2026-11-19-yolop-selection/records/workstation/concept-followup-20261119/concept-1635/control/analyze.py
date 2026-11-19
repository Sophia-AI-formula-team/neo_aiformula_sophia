import gzip,json,math,statistics,collections
from pathlib import Path
d=Path(__file__).parent
rows=[json.loads(i) for i in gzip.open(d/'control.jsonl.gz')]
start=rows[0][0]
s={}
segments=[]
for t,k,*v in rows:
 s[k]=v
 if k!='m' or not all(i in s for i in 'wgc'):continue
 elapsed=(t-start)/1e9
 if len(segments)==0 or segments[-1]['k']!=s['g'][:1]+v:
  segments.append(dict(k=s['g'][:1]+v,t0=elapsed,t1=elapsed,w=[],gyro=[],can=[]))
 seg=segments[-1]
 seg['t1']=elapsed
 l,r,g,valid=s['w']
 if valid:seg['w'].append((r+l)*math.pi*.127/60);seg['gyro'].append(g)
 seg['can'].append(s['c'])
for seg in segments:
 mode,v,w=seg['k'];stat=lambda a: [round(i,3) for i in (a[0],statistics.median(a),a[-1],max(a,key=abs))] if a else []
 if mode=='auto' and (v or w) or (mode=='manual' and seg['t1']-seg['t0']>.28 and v):
  print(seg['k'],round(seg['t0'],2),round(seg['t1']-seg['t0'],2),stat(seg['w']),stat(seg['gyro']),seg['can'][len(seg['can'])//2])
print('DECISION',collections.Counter((e['status'],e['reason']) for t,k,*v in rows if k=='d' for e in v))
(d/'segments.json').write_text(json.dumps(segments,separators=(',',':')))
