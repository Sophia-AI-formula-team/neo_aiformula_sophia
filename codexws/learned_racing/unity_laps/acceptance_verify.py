"""Check all recorded controls, image streams and encoded videos before delivery."""
from pathlib import Path
import zipfile
import av
import numpy as np
from PIL import Image,ImageDraw
from acceptance_run import HERE,ROOT,read,save,sha
from acceptance_media import OUT,txt,BG,MUTED


def main():
    plan=read(ROOT/'plan.json');checks=[];total_frames=0
    for cohort in (ROOT,ROOT/'race_rule'):
        for case in plan['cases']:
            for v in plan['versions']:
                name=case['id']+'_'+v;r=read(cohort/'runs'/(name+'.json'))
                assert r['request']['seed']==case['seed'] and r['request']['externalControl']
                assert r['request']['model'].startswith(v.replace('_','.')+'/A4/')
                n=0
                for decision in r['decisions']:
                    obs=decision['decision'];a=obs['audit']
                    assert set(obs['sensor'])=={'t','leftRpm','rightRpm','gyro','wheelRadius','source'}
                    assert abs(obs['sensor']['t']-decision['t'])<1e-6
                    if a['selected'] is None:continue
                    tab=np.array(a['table']);selected=a['selected']
                    assert tab.shape==(60,12) and np.array(a['paths']).shape==(60,12,2)
                    # JSON is intentionally rounded for display; command check
                    # allows only its explicit 1e-6 serialization precision.
                    assert abs(tab[selected,0]-decision['speedCmd'])<1e-6
                    assert abs(tab[selected,1]-decision['yawCmd'])<1e-6
                    assert abs(tab[selected,7]-tab[:,7].max())<1e-6
                    assert np.max(abs(tab[:,3]-a['uncertainty_scale']*tab[:,4]-tab[:,5]))<3e-6
                    n+=1
                for suffix in ('c','f'):
                    assert len(list((cohort/'.work/frames'/name).glob('*'+suffix+'.jpg')))==len(r['frames'])
                assert len(list((cohort/'.work'/name/'frames').glob('*.jpg')))==r['seenFrames']
                assert r['frames'][-1][0]>=r['seconds']-.201
                checks.append(dict(cohort='all_wheels'if cohort!=ROOT else 'strict_body',id=name,decisions=len(r['decisions']),candidate_frames=n))
                total_frames+=n
    videos=[]
    for p in sorted(OUT.glob('*.mp4')):
        with av.open(str(p))as file:
            st=file.streams.video[0];assert (st.width,st.height)==(1920,1080)
            assert st.codec_context.name=='h264';count=0
            for frame in file.decode(video=0):
                assert (frame.width,frame.height)==(1920,1080);count+=1
            assert count>50
            videos.append(dict(name=p.name,frames=count,duration_s=count/10,bytes=p.stat().st_size,sha256=sha(p),fully_decoded=True))
    assert len(videos)==18
    save(ROOT/'verification.json',dict(runs=checks,run_count=len(checks),candidate_frames=total_frames,videos=videos,
        sensor_whitelist_checked=True,all_image_counts_checked=True,all_video_frames_decoded=True,
        renderer_sources={f:sha(HERE/f)for f in ['acceptance_media.py','acceptance_diagnostics.py','acceptance_verify.py']}))
    # Review sheet uses unmodified middle/terminal frames from each actual film.
    sheet=Image.new('RGB',(1920,2400),BG);draw=ImageDraw.Draw(sheet)
    for i,case in enumerate(plan['cases']):
        x=(i%4)*480;y=(i//4)*600
        txt(draw,x+12,y+8,case['id']+' '+case['kind'],21,MUTED)
        for offset,suffix in ((40,'middle'),(315,'end')):
            sheet.paste(Image.open(OUT/(case['id']+'_'+suffix+'.jpg')).resize((480,270)),(x,y+offset))
    sheet.save(ROOT/'visual_review.jpg',quality=90)
    # Raw tables, both rules, source versions and frozen checkpoints. No login
    # state, browser screenshots, credentials or raw bag content is included.
    paths=[]
    for f in ['plan.json','freeze.json','results.json','instrumentation_parity.json','referee_prefix_parity.json','prediction_followup.json','verification.json']:
        paths.append((ROOT/f,f))
    for folder in ['frozen','runs','race_rule/runs']:
        paths.extend((p,p.relative_to(ROOT).as_posix())for p in sorted((ROOT/folder).rglob('*'))if p.is_file())
    paths.append((ROOT/'race_rule/protocol.json','race_rule/protocol.json'))
    for f in ['acceptance_audit.py','acceptance_run.py','acceptance_race.py','acceptance_media.py','acceptance_diagnostics.py','acceptance_verify.py',
              'v3_model.py','v3_vision.py','run.py','UnityRace.cs','Road.cs','v3_evidence/plant_fit.json']:
        paths.append((HERE/f,'sources/'+f))
    manifest=[dict(path=dest,bytes=p.stat().st_size,sha256=sha(p))for p,dest in paths]
    with zipfile.ZipFile(OUT/'evidence.zip','w',zipfile.ZIP_DEFLATED,compresslevel=6)as z:
        import json
        z.writestr('manifest.json',json.dumps(manifest,indent=2))
        for p,dest in paths:z.write(p,dest)
    with zipfile.ZipFile(OUT/'evidence.zip')as z:
        assert z.testzip()is None
        for item in manifest:
            import hashlib
            assert hashlib.sha256(z.read(item['path'])).hexdigest()==item['sha256']
    print({'runs':len(checks),'candidate_frames':total_frames,'videos':len(videos),'zip_files':len(paths),'zip_bytes':(OUT/'evidence.zip').stat().st_size})


if __name__=='__main__':main()
