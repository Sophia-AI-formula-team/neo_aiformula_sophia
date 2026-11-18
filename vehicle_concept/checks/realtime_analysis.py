"""Offline replay timing analysis using recorded monotonic timestamps; requires only NumPy."""
import argparse
from bisect import bisect_right
from collections import Counter
import json
from pathlib import Path

import numpy as np

BILLION=10**9


def pct(values):
    if not values:
        return None
    return dict(zip(('p50','p95','p99','max'),np.percentile(values,[50,95,99,100]).round(3).tolist()))


def summary(args, sent, started, out, record_dir=None):
    data={k:json.loads(((record_dir or out)/(k+'.json')).read_text()) for k in ('perception','decision','twist')}
    by_stamp={r['stamp_ns']:r for r in sent}
    def slot(record):
        return by_stamp[record['stamp_ns']]['seq']*args.decision_hz//args.camera_hz
    end_ns = started + args.seconds*BILLION
    valid = [r for r in data['twist'] if r['stamp_ns'] in by_stamp]
    earliest = {}
    for r in valid:
        key = slot(r)
        if key not in earliest or r['received_ns']<earliest[key]['received_ns']:
            earliest[key]=r
    expected = args.seconds*args.decision_hz
    spacings = np.diff([r['received_ns'] for r in valid]).tolist()
    stop_gaps = np.diff([started]+[r['received_ns'] for r in valid if started<=r['received_ns']<=end_ns]+[end_ns])
    deadline_count=sum(r['received_ns']<=started+(k+1)*BILLION//args.decision_hz for k,r in earliest.items())
    latency=[(r['received_ns']-by_stamp[r['stamp_ns']]['capture_mono_ns'])/1e6 for r in valid]
    within200=sum(r['received_ns']-by_stamp[r['stamp_ns']]['capture_mono_ns']<=200_000_000 for r in earliest.values())
    diagnostics=[r for r in data['decision'] if r['stamp_ns'] in by_stamp]
    stationary=[r for r in valid if r['cmd'][0]==0]
    nonshadow={r['stamp_ns'] for r in diagnostics if r['status']=='ok'}
    result=dict(camera_hz=args.camera_hz,decision_hz=args.decision_hz,seconds=args.seconds,
        planned_frames=args.seconds*args.camera_hz,sent_frames=len(sent),
        unique_archived_images=len({r['archive_frame'] for r in sent}),archive_hz=5,feedback=args.feedback,
        publish_lag_ms=pct([(r['capture_mono_ns']-r['planned_ns'])/1e6 for r in sent]),
        perception_frames=len({r['stamp_ns'] for r in data['perception'] if r['stamp_ns'] in by_stamp}),
        decision_frames=len(diagnostics),expected_slots=expected,distinct_output_slots=len(earliest),
        raw_matching_twists=len(valid),twists_with_matching_ok_decision=sum(r['stamp_ns'] in nonshadow for r in valid),
        zero_linear_outputs=len(stationary),
        nonzero_linear_outputs=sum(r['cmd'][0]!=0 for r in valid),
        zero_twists=sum(r['cmd']==[0.,0.] for r in valid),
        nonzero_twists=sum(r['cmd']!=[0.,0.] for r in valid),
        yaw_only_twists=sum(r['cmd'][0]==0 and r['cmd'][1]!=0 for r in valid),
        forward_twists=sum(r['cmd'][0]>0 for r in valid),
        reverse_twists=sum(r['cmd'][0]<0 for r in valid),
        on_slot_deadline=deadline_count,late_slots=len(earliest)-deadline_count,
        within_200ms_capture=within200,missing_slots=expected-len(earliest),
        latency_ms=pct(latency),output_interval_ms=pct([x/1e6 for x in spacings]),
        consecutive_intervals_over_200ms=sum(x>200_000_000 for x in spacings),
        longest_gap_including_run_edges_ms=round(max(stop_gaps)/1e6,3),
        pauses_over_200ms=int(sum(stop_gaps>200_000_000)),status_reason=dict(Counter(
            str(r['status'])+' / '+str(r['reason']) for r in diagnostics)),
        first_output_ms=round((valid[0]['received_ns']-started)/1e6,3) if valid else None,
        policy_processing_ms=pct([r['processing_ms'] for r in diagnostics if r['processing_ms'] is not None]))
    rows=[]
    seen={r['stamp_ns'] for r in data['perception']}
    for k in range(expected):
        inputs=[r for r in sent if r['seq']*args.decision_hz//args.camera_hz==k]
        captures={r['stamp_ns'] for r in inputs}
        reasoning=[r for r in diagnostics if r['stamp_ns'] in captures]
        cmd=earliest.get(k)
        due=started+(k+1)*BILLION//args.decision_hz
        rows.append(dict(slot=k,input_ids=[r['seq'] for r in inputs],perceived=len(captures&seen),
            decision_count=len(reasoning),reasons=[r['reason'] for r in reasoning],has_output=cmd is not None,
            on_deadline=bool(cmd and cmd['received_ns']<=due),
            latency_ms=None if cmd is None else (cmd['received_ns']-by_stamp[cmd['stamp_ns']]['capture_mono_ns'])/1e6))
    def portion(items):
        delays=[r['latency_ms'] for r in items if r['has_output']]
        return dict(expected=len(items),received=len(delays),on_deadline=sum(r['on_deadline'] for r in items),
                    within_200ms=sum(t<=200 for t in delays),latency_ms=pct(delays))
    first=min(expected,5*args.decision_hz)
    result['cold_first_5s']=portion(rows[:first])
    result['following_slots']=portion(rows[first:])
    # The production driver anchors its period to the first accepted camera stamp.
    # Keep the exogenous source grid and report its phase separately.
    camera_slots_per_decision=args.camera_hz//args.decision_hz
    result['output_source_phase_counts']=dict(Counter(
        by_stamp[r['stamp_ns']]['seq'] % camera_slots_per_decision for r in earliest.values()))
    result['distinct_receive_slots']=len({
        (r['received_ns']-started)*args.decision_hz//BILLION for r in valid
        if started<=r['received_ns']<end_ns})
    result['receive_slots_with_capture_age_at_most_200ms']=len({
        (r['received_ns']-started)*args.decision_hz//BILLION for r in valid
        if started<=r['received_ns']<end_ns
        and r['received_ns']-by_stamp[r['stamp_ns']]['capture_mono_ns']<=200_000_000})
    result['duplicate_output_stamps']=len(valid)-len({r['stamp_ns'] for r in valid})
    # Exact minimum over all one-second (left, right] windows fully inside the run.
    # The count can decrease only when a reception crosses the left edge.
    receipts=sorted(r['received_ns'] for r in valid if started<=r['received_ns']<=end_ns)
    edges=[started,end_ns-BILLION]+[t for t in receipts if t<=end_ns-BILLION]
    result['min_twists_in_sliding_1s']=min(
        bisect_right(receipts,t+BILLION)-bisect_right(receipts,t) for t in edges)
    result['missing_slot_evidence']=dict(Counter(
        ('capture_not_sent' if not r['input_ids'] else 'no_perception_observed' if not r['perceived']
         else 'no_decision_observed' if not r['decision_count'] else 'no_twist_despite_decision')
        for r in rows if not r['has_output']))
    result['capture_plan_to_receive_ms']=pct([(r['received_ns']-by_stamp[r['stamp_ns']]['planned_ns'])/1e6 for r in valid])
    (out/'slots.json').write_text(json.dumps(rows))
    (out/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2),flush=True)


def rescore(args,record_dir,out):
    recorded=json.loads((record_dir/'summary.json').read_text())
    args.seconds,args.camera_hz,args.feedback=(recorded[k] for k in ('seconds','camera_hz','feedback'))
    sent=json.loads((record_dir/'sent.json').read_text())
    if not sent:raise ValueError('Cannot reconstruct the acquisition grid without any sent images')
    started=sent[0]['planned_ns']-sent[0]['seq']*BILLION//args.camera_hz
    out.mkdir(parents=True,exist_ok=False)
    summary(args,sent,started,out,record_dir)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rescore-from",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--decision-hz",type=int,choices=(5,15),default=5)
    args=parser.parse_args()
    rescore(args,args.rescore_from,args.out)
