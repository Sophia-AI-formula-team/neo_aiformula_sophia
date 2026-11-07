"""Auditable paired films and readable results from recorded Unity runs only."""
import argparse
from functools import lru_cache
import html
import json
import math
from pathlib import Path
import av
import numpy as np
from PIL import Image, ImageDraw
from render import font, track
from acceptance_run import ROOT, read, save, sha

OUT=ROOT/'delivery'
BG='#101720';PANEL='#1b2635';INK='#eff5fb';MUTED='#a9b8ca';GREEN='#50e3b4';RED='#ff7d88';BLUE='#6ab6ff';YELLOW='#ffd278'
STATUS={'lap':'完成整圈','segment':'完成本段','boundary':'全部轮位越界','obstacle':'碰桶','stalled':'停滞'}


def txt(d,x,y,s,size=19,color=INK,bold=False):
    d.text((x,y),str(s),font=font(size,bold),fill=color)


@lru_cache(maxsize=256)
def photo(path,size):
    return Image.open(path).convert('RGB').resize(size,Image.Resampling.LANCZOS)


def metrics(r,case):
    rows=np.asarray(r['rows']);ds=np.linalg.norm(np.diff(rows[:,1:3],axis=0),axis=1)
    dt=np.diff(rows[:,0]);success=r['status'] in ('lap','segment')
    decisions=r['decisions'];used=[d['decision'] for d in decisions]
    ahead=[min([o[0] for o in d['decision']['objects'] if o[0]>0],default=99) for d in decisions]
    # Straightness/obstacle-free selection belongs to the offline referee only.
    geom=track();center=(geom[0]+geom[1])/2
    di=np.roll(center,-5,axis=0)-np.roll(center,5,axis=0)
    heading=np.unwrap(np.arctan2(di[:,0],di[:,1]));station=np.r_[0,np.cumsum(np.linalg.norm(np.diff(center,axis=0),axis=1))]
    curvature=np.gradient(heading,station)
    idx=np.argmin(np.sum((rows[:,None,1:3]-center[None,:,:])**2,axis=2),axis=1)
    clean=(abs(curvature[idx])<.015)&(rows[:,10]>10)&(rows[:,0]>2)
    clean_ds=ds*clean[:-1];clean_time=dt*clean[:-1]
    windows=[(a,b)for a,b in zip(r['zoneStart'],r['zoneEnd'])if a>=0 and b>=0]
    return dict(id=r['id'],status=r['status'],success=success,full_lap=case['full_lap'],seconds=r['seconds'],
        progress_m=r['progress'],target_m=case['limitM'],travel_m=float(ds.sum()),
        mean_body_speed=float(ds.sum()/max(r['seconds'],.01)),peak_body_speed=float(rows[:,4].max()),
        clear_straight_distance_m=float(clean_ds.sum()),
        clear_straight_mean_speed=float(clean_ds.sum()/clean_time.sum()) if clean_time.sum() else None,
        clear_straight_at_least_4_distance_fraction=float((clean_ds*(rows[:-1,4]>=4)).sum()/clean_ds.sum()) if clean_ds.sum() else None,
        yaw_command_tv_per_s=float(np.abs(np.diff([d['yawCmd']for d in decisions])).sum()/max(r['seconds'],.01)),
        min_body_road_clearance=r['minRoad'],min_cone_clearance=r['minGap'] if r['props'] else None,
        obstacles_passed=sum(b>=0 for b in r['zoneEnd']),obstacles_total=len(r['props']),
        encounter_seconds=max(r['zoneEnd'])-min(r['zoneStart']) if r['props'] and all(b>=0 for b in r['zoneEnd']) else None,
        no_feasible_frames=sum(d['reason']=='no_feasible_trajectory'for d in used),decision_frames=len(used),
        first_body_edge=r.get('firstBodyEdge'),terminal_decision=used[-1]['reason'],
        terminal_predicted_margin=used[-1].get('margin'),
        terminal_wheel_gaps=r['wheelRows'][-1][1:] if r.get('wheelRows')else None,
        policy_sensor_fields=sorted(used[0]['sensor']),source_sha256=sha((ROOT/'race_rule/runs'/(r['id']+'.json'))))


class View:
    def __init__(self,case,version,cohort):
        self.case=case;self.version=version;self.cohort=cohort
        self.r=read(cohort/'runs'/(case['id']+'_'+version+'.json'))
        self.rows=np.array(self.r['rows']);self.ft=np.array(self.r['frames']);self.ds=self.r['decisions'];self.ts=np.array([d['t']for d in self.ds])
        self.wheels=np.array(self.r.get('wheelRows',[]))
        self.media=cohort/'.work/frames'/self.r['id'];self.perception=cohort/'.work'/self.r['id']/'frames'

    def draw(self,canvas,x,t,total):
        d=ImageDraw.Draw(canvas);r=self.r;end=t>=r['seconds'];clock=min(t,r['seconds'])
        i=max(0,int(np.searchsorted(self.rows[:,0],clock+1e-6,side='right')-1));row=self.rows[i]
        f=max(0,int(np.searchsorted(self.ft[:,0],clock+1e-6,side='right')-1));frame=int(self.ft[f,1])
        j=max(0,int(np.searchsorted(self.ts,clock+1e-6,side='right')-1));decision=self.ds[j];obs=decision['decision'];audit=obs['audit']
        version=self.version.replace('_','.');col=BLUE if version=='V3.0' else GREEN
        d.rounded_rectangle((x+8,61,x+952,1012),radius=12,fill=PANEL)
        txt(d,x+24,68,version+('  未训练' if version=='V3.0'else'  30,000 次试错后 / 第 118 轮'),25,col,True)
        txt(d,x+24,104,f'跟车实录  {self.ft[f,0]:.2f} s',18,MUTED)
        txt(d,x+494,104,f'A4 + RGB 路面观测  {decision["t"]:.2f} s',18,MUTED)
        canvas.paste(photo(str(self.media/f'{frame:04d}c.jpg'),(456,257)),(x+20,132))
        canvas.paste(photo(str(self.perception/f'{j:05d}.jpg'),(456,257)),(x+484,132))
        if end:
            status=STATUS.get(r['status'],r['status'])
            if r['status']=='boundary' and r.get('departureRule')!='all_wheel_centers':status='车身角触界（严格判据）'
            d.rectangle((x+20,340,x+476,389),fill='#173e38'if r['status']in('lap','segment')else'#6f2632')
            txt(d,x+30,346,status+' · 终点保留',24,INK,True)
        txt(d,x+22,399,f'车身实速 {row[4]:.2f} m/s',27,col,True)
        txt(d,x+332,402,f'指令 {decision["speedCmd"]:.2f} m/s',22)
        txt(d,x+602,402,f'轮速代理 {obs["speedEstimate"]:.2f}',22,MUTED)
        txt(d,x+22,438,'车辆看到的局部道路 / 候选轨迹',19,INK,True)
        txt(d,x+452,438,'为什么选这个动作？',21,INK,True)
        plane=Image.new('RGB',(412,382),'#111c28');g=ImageDraw.Draw(plane)
        def q(z,xx):return (206+xx*31,337-z*31)
        for zz in (0,2,4,6,8,10):
            g.line([q(zz,-6),q(zz,6)],fill='#273647');txt(g,4,337-zz*31,f'{zz}m',13,MUTED)
        road=obs['road']
        if len(road)>1:g.polygon([q(p[0],p[1])for p in road]+[q(p[0],p[2])for p in reversed(road)],fill='#3a4b59')
        table=np.asarray(audit['table']);paths=audit['paths'];selected=audit['selected']
        for k,path in enumerate(paths):
            g.line([q(*p)for p in path],fill='#446657'if table[k,6]else'#663c46',width=1)
        for z,xx,vz,vx,rr,sigma in obs['objects']:
            px,py=q(z,xx);radius=rr*31
            g.ellipse((px-radius,py-radius,px+radius,py+radius),fill=YELLOW,outline='#fff2c7',width=1)
            a=q(z+vz,xx+vx);g.line([(px,py),a],fill=YELLOW,width=3);g.ellipse((a[0]-3,a[1]-3,a[0]+3,a[1]+3),fill=YELLOW)
        if selected is not None:
            g.line([q(0,0)]+[q(*p)for p in paths[selected]],fill=GREEN,width=4)
            tx,ty=q(*obs['target']);g.polygon([(tx,ty-6),(tx+6,ty),(tx,ty+6),(tx-6,ty)],fill='#d4a2ff')
        g.rectangle((*q(.28,-.38),*q(-.88,.38)),fill=BLUE,outline='white',width=1)
        txt(g,12,354,'前 ↑    黄：桶与速度    菱形：参考点',15)
        canvas.paste(plane,(x+20,469))
        # Every quantitative value below comes from the action actually issued.
        if selected is None:
            txt(d,x+452,478,'道路不足 3 段：逐步减速',22,RED,True)
        else:
            rowpick=table[selected];prev=audit['previous_command']
            longitudinal='提速'if decision['speedCmd']>prev[0]+.1 else '收油减速'if decision['speedCmd']<prev[0]-.1 else '保持目标速度'
            turn='右转'if decision['yawCmd']>.1 else '左转'if decision['yawCmd']<-.1 else '接近直行'
            good=obs['reason']=='feasible'
            txt(d,x+452,475,longitudinal+' / '+turn,25,GREEN if good else RED,True)
            txt(d,x+452,511,f'可行 {obs["feasible"]}/{obs["candidates"]} 条；预测 {audit["horizon"]:.2f} 秒',20)
            txt(d,x+452,542,'净空减不确定度 ≥ 训练余量，才算可行',18,MUTED)
            margin=f'{rowpick[3]:.2f} − {audit["uncertainty_scale"]:.2f}×{rowpick[4]:.2f} = {rowpick[5]:.2f} m'
            txt(d,x+452,570,margin,20,GREEN if good else RED)
            txt(d,x+452,598,f'门槛 {audit["threshold"]:.3f} m；选中候选 #{selected+1}',19)
            txt(d,x+452,629,'选项      速度      净空余量     得分      可行',17,MUTED)
            options=[selected]
            for pick in (int(np.argmax(table[:,0])),int(np.argmax(table[:,5])),int(np.argmin(abs(table[:,2])))):
                if pick not in options:options.append(pick)
            for pick in np.argsort(table[:,7])[::-1]:
                if len(options)>=4:break
                if int(pick)not in options:options.append(int(pick))
            for n,k in enumerate(options):
                rr=table[k];score=f'{rr[7]:.2f}'if rr[7]>-999999 else '排除'
                txt(d,x+452,656+n*26,f'{"选"if k==selected else "  "}#{k+1:<2}    {rr[0]:4.2f}      {rr[5]:+5.2f}       {score:>5}       {"是"if rr[6]else"否"}',17,GREEN if k==selected else MUTED)
            note='在可行候选中选综合得分最高的一条。'if good else'无可行轨迹：仍在发出最低损害动作！'
            txt(d,x+452,770,note,18,GREEN if good else RED,True)
            if good:
                txt(d,x+452,799,'得分 = 前进 − 朝向误差 − 摆动 − 余量罚项',17,MUTED)
                txt(d,x+452,824,f'{rowpick[8]:.2f} − {rowpick[9]:.2f} − {rowpick[10]:.2f} − {rowpick[11]:.2f}',17,MUTED)
            else:
                base=rowpick[8]-rowpick[9]-rowpick[10]-rowpick[11]
                txt(d,x+452,799,'此时得分 = 100×余量 + 基础分 − 0.3×速度',17,RED)
                txt(d,x+452,824,f'100×{rowpick[5]:.2f} + {base:.2f} − 0.3×{rowpick[0]:.2f}',17,MUTED)
        txt(d,x+20,859,f'转向指令 {decision["yawCmd"]:+.2f} rad/s  |  实际 yaw {row[14]:+.2f}',17,MUTED)
        txt(d,x+500,859,f'实际进度 {row[7]:.1f} / {self.case["limitM"]:.1f} m',18,col)
        # Fixed time and speed scales, matching the opposite version.
        xx0,yy0,ww,hh=x+62,903,850,70
        for speed in (0,2,4):
            yy=yy0+hh-speed/5*hh;d.line((xx0,yy,xx0+ww,yy),fill='#334355');txt(d,x+22,yy-10,str(speed),14,MUTED)
        past=self.rows[self.rows[:,0]<=clock+1e-6]
        for k,color in ((4,col),(5,'#7b8797')):
            points=[(xx0+p[0]/total*ww,yy0+hh-min(5,max(0,p[k]))/5*hh)for p in past]
            if len(points)>1:d.line(points,fill=color,width=3 if k==4 else 1)
        d.line((xx0+clock/total*ww,yy0,xx0+clock/total*ww,yy0+hh),fill='white',width=1)
        txt(d,x+22,981,'实线：车身速度   灰线：目标速度   相同时间轴 / 1×播放',15,MUTED)
        if len(self.wheels):
            wheel=self.wheels[max(0,int(np.searchsorted(self.wheels[:,0],clock+1e-6,side='right')-1))]
            txt(d,x+20,1021,'裁判轮位距边：'+ ' / '.join(f'{g:+.2f}'for g in wheel[1:])+' m',17,RED if all(g<0 for g in wheel[1:])else MUTED)


def frame_pair(views,case,t,total):
    canvas=Image.new('RGB',(1920,1080),BG);d=ImageDraw.Draw(canvas)
    txt(d,22,12,case['id']+'  '+case['title'],30,INK,True)
    txt(d,1290,18,f'Unity 实际闭环 · 同起点 / 同布局 · {t:.1f} s',21,MUTED)
    for i,v in enumerate(views):v.draw(canvas,i*960,t,total)
    txt(d,20,1053,'绿：选中路径  暗绿：可行备选  红：不满足余量  |  左右均仅使用 RGB / A4 / 轮速代理 / gyro；裁判真值不输入驾驶',17,MUTED)
    return canvas


def encode(path,frames,rate=10):
    with av.open(str(path),'w',options={'movflags':'+faststart'})as output:
        stream=output.add_stream('libx264',rate=rate);stream.width=1920;stream.height=1080;stream.pix_fmt='yuv420p'
        stream.options={'crf':'22','preset':'fast','threads':'4'}
        for canvas in frames:
            for packet in stream.encode(av.VideoFrame.from_image(canvas)):output.mux(packet)
        for packet in stream.encode():output.mux(packet)


def film(case,cohort):
    views=[View(case,v,cohort)for v in ('V3_0','V3_2')];total=max(v.r['seconds']for v in views)
    path=OUT/(case['id']+'_'+case['kind']+'_V3.0_vs_V3.2.mp4')
    def frames():
        title=Image.new('RGB',(1920,1080),BG);d=ImageDraw.Draw(title)
        txt(d,100,160,case['title'],52,INK,True)
        txt(d,100,270,'左：V3.0 未训练    右：V3.2 / 30,000 次训练',40,GREEN)
        lines=['同起点、同障碍位置与运动相位；各自从静止起步。',
               '跟车画面 + A4 识别 + 全部候选路径 + 净空门槛 + 得分 + 实际速度。',
               '两车使用同一模拟时钟。提前结束的一侧定格，失败不剪掉。',
               '碰桶或全部轮位出界即结束；此仿真为前二后一的三轮底盘。',
               '局部段通过不等于完圈；几何余量不等于通过概率。']
        for i,line in enumerate(lines):txt(d,100,390+i*70,line,28,MUTED)
        if case['kind'].startswith('gap_'):
            gap=case['props'][1][1]-case['props'][0][1]-.68
            txt(d,100,800,f'裁判布置：桶间净宽 {gap:.2f} m；车宽 0.76 m。该数据只用于验收，不喂给模型。',28,YELLOW)
        for _ in range(40):yield title
        for t in np.arange(0,total+3.1,.1):yield frame_pair(views,case,float(t),total)
    encode(path,frames())
    for label,t in [('start',0),('middle',total*.55),('end',total+1)]:
        frame_pair(views,case,t,total).save(OUT/(case['id']+'_'+label+'.jpg'),quality=88)
    print(path.name,flush=True)


def report():
    plan=read(ROOT/'plan.json');cohort=ROOT/'race_rule';results=[];prefix=[]
    for case in plan['cases']:
        pair={}
        for version in plan['versions']:
            r=read(cohort/'runs'/(case['id']+'_'+version+'.json'));pair[version]=metrics(r,case)
            strict=read(ROOT/'runs'/(r['id']+'.json'))
            # Both runs must be identical until the earlier strict termination.
            a=np.array(strict['rows']);b=np.array(r['rows']);times=[q[0]for q in b]
            shared=[(q,b[int(np.argmin(abs(b[:,0]-q[0])))])for q in a if min(abs(b[:,0]-q[0]))<1e-6]
            err=max((float(np.max(abs(x-y)))for x,y in shared),default=0)
            prefix.append(dict(id=r['id'],shared_rows=len(shared),max_abs_difference=err,strict_status=strict['status']))
            assert sorted(pair[version]['policy_sensor_fields'])==sorted(['t','leftRpm','rightRpm','gyro','wheelRadius','source'])
            assert len(r['frames'])==len(list((cohort/'.work/frames'/r['id']).glob('*c.jpg')))
            assert r['seenFrames']==len(list((cohort/'.work'/r['id']/'frames').glob('*.jpg')))
            if r['status']=='boundary':assert all(g<0 for g in r['wheelRows'][-1][1:])
        success=all(p['success']for p in pair.values())
        results.append(dict(case=case,pair=pair,paired_time_delta_s=pair['V3_2']['seconds']-pair['V3_0']['seconds'] if success else None))
    save(ROOT/'referee_prefix_parity.json',prefix)
    totals={v:dict(completed=sum(r['pair'][v]['success']for r in results),cases=len(results),
                   full_laps=sum(r['pair'][v]['success']and r['case']['full_lap']for r in results),full_lap_attempts=3,
                   collisions=sum(r['pair'][v]['status']=='obstacle'for r in results),
                   departures=sum(r['pair'][v]['status']=='boundary'for r in results))for v in plan['versions']}
    save(ROOT/'results.json',dict(totals=totals,cases=results))
    lines=['# V3.0 / V3.2 Unity 验收 — 2026-11-08','',
           '冻结第 118 轮（30,000 次程序化训练），未在验收结果上调参。训练不是 30,000 趟 Unity。',
           '三轮仿真：前两驱动轮 + 后脚轮。主结果按全部三个轮中心出界终止；原严格车身角规则的 32 趟也完整保留。',
           '局部路段成功不等于整圈成功。碰撞/出界提前终止不能作为圈速更快的证据。','',
           '| 场景 | V3.0 | V3.2 | 两者完成时的用时差 |','|---|---|---|---|']
    table=[]
    for r in results:
        c=r['case'];p=r['pair'];cells=[]
        for v in plan['versions']:
            m=p[v];cells.append(f"{STATUS.get(m['status'],m['status'])} / {m['progress_m']:.1f}m / {m['seconds']:.2f}s")
        delta='不可比较'if r['paired_time_delta_s']is None else f"V3.2 {r['paired_time_delta_s']:+.2f}s"
        lines.append(f"| {c['id']} {c['title']} | {cells[0]} | {cells[1]} | {delta} |")
        video=c['id']+'_'+c['kind']+'_V3.0_vs_V3.2.mp4'
        table.append(f'<tr><td>{c["id"]} {html.escape(c["title"])}</td><td>{cells[0]}</td><td>{cells[1]}</td><td>{delta}</td><td><a href="{video}">播放对比</a></td></tr>')
    lines += ['', '判据：车身代理长 1.16 m、宽 0.76 m；桶半径 0.34 m。车轮规则使用轮中心，未覆盖完整接触斑宽度。',
              '感知：A4 检测/车线 + RGB 路面颜色；不是 A4 可行驶区语义分割。只输入图像、虚拟轮速代理与 raw gyro；第一圈没有预载地图。',
              '规划：白盒预测 60 个速度/转向候选，扣除不确定度后检查净空；在可行候选内按前进、朝向、平顺性综合评分。9 个系数由 CEM 训练。',
              '无可行轨迹时仍选择“最低损害”动作，这是当前失效风险；不是一个已验证的紧急停车策略。',
              '车身实速来自裁判，只用于显示/评分。轮速代理来自仿真电机目标，不能当作独立实测编码器。',
              '单步动力学拟合不证明长时预测正确；有限候选最高分不等于全局最短圈速；净空余量不是校准后的通过概率。',
              '同步：相机和推理 5 Hz、视频 10 fps 重复最近帧，1×模拟时间；不同侧提前终止后定格。',
              '判据仪表保留选中动作、备选/拒绝、全部路径和得分；原始 JSON 保存每帧全部 60 个候选。','']
    (OUT/'00_验收说明.md').write_text('\n'.join(lines),encoding='utf8')
    total='；'.join(f"{v.replace('_','.')}：完成 {m['completed']}/16 场景，整圈 {m['full_laps']}/3，碰桶 {m['collisions']} 次，全部轮位越界 {m['departures']} 次"for v,m in totals.items())
    page='<!doctype html><meta charset="utf-8"><title>V3 驾驶验收</title><style>body{background:#101720;color:#eff5fb;font:18px system-ui;max-width:1500px;margin:40px auto;padding:20px}h1{font-size:38px}p{line-height:1.7}a{color:#50e3b4}table{border-collapse:collapse;width:100%}th,td{padding:13px;border-bottom:1px solid #334355;text-align:left}video{width:100%}strong{color:#ffd278}.note{background:#1b2635;padding:24px}</style>'
    page+=f'<h1>V3.0 → V3.2：训练后的实际驾驶验收</h1><p><strong>{total}</strong></p><p class="note">64 趟 Unity 原始试车：32 趟原严格车身角判据，另 32 趟按全部轮位越界。以下视频采用后者。两版采用相同新起点与障碍布局，模型冻结。训练发生在程序化简化环境，验收使用真实 Unity/A4 相机闭环。</p>'
    page+='<video controls preload="metadata" src="00_验收总览.mp4"></video><h2>逐场对比（从起步到终点，无删失败）</h2><table><tr><th>场景</th><th>V3.0</th><th>V3.2</th><th>时间比较</th><th>完整视频</th></tr>'+''.join(table)+'</table>'
    page+='<h2>仪表怎么看</h2><p>绿线是选中轨迹，暗绿是可行备选，红线不满足余量。黄点与箭头是传感器估计的桶及其速度。右侧净空公式展示扣除不确定度后是否达到训练门槛；没有可行候选时显示红色告警。速度图使用相同坐标轴，实际车身速度和彩色线对应，灰线只是目标指令。</p><p>固定机位的场景总览仅用于裁判解释，不向驾驶模型输入世界坐标。虚拟车为三轮，所有轮中心出界是比赛规则的近似。此验收不是实车验证。</p><p><a href="00_验收说明.md">详细判据与局限</a> · <a href="evidence.zip">原始决策、指标、模型和协议</a></p>'
    (OUT/'index.html').write_text(page,encoding='utf8')
    return results,totals


def summary_video(results,totals):
    def card(title,lines,color=INK):
        im=Image.new('RGB',(1920,1080),BG);d=ImageDraw.Draw(im);txt(d,100,120,title,52,color,True)
        for i,s in enumerate(lines):txt(d,100,260+i*85,s,30,MUTED)
        return im
    def frames():
        intro=card('V3.2 验收：先看真实结果',[
            '30,000 次训练已完成；第 118 轮最终模型冻结，验收不调参。',
            f"V3.0：16 场景完成 {totals['V3_0']['completed']}；完整圈 {totals['V3_0']['full_laps']}/3。",
            f"V3.2：16 场景完成 {totals['V3_2']['completed']}；完整圈 {totals['V3_2']['full_laps']}/3。",
            '有失败就明确失败；未完圈时不比较圈速。',
            '跟车 / 识别 / 路径 / 判据 / 动作 / 实际速度，完整视频逐场提供。'],YELLOW)
        for _ in range(120):yield intro
        for result in results:
            c=result['case'];views=[View(c,v,ROOT/'race_rule')for v in ('V3_0','V3_2')];total=max(v.r['seconds']for v in views)
            # Overview excerpts are explicitly labeled; per-case films retain all frames.
            start=max(0,total-4)
            for t in np.arange(start,total+1,.1):
                im=frame_pair(views,c,float(t),total);d=ImageDraw.Draw(im)
                d.rectangle((1290,0,1919,58),fill='#623b18');txt(d,1300,16,'总览节选 · 完整起步到终点见各场视频',22,YELLOW)
                yield im
        end=card('当前结论与下一步',[
            '训练完成 ≠ 驾驶验收通过。完整圈失败仍需解决。',
            '首先修复局部道路误关联与长时动力学预测偏差。',
            '无可行轨迹时的控制需要可验证的停车/恢复方案。',
            '再用新版本、新留出场景比较成功率和有效通行时间。',
            '全部原始失败和候选判据都已留存；本轮未改驾驶参数。'],YELLOW)
        for _ in range(100):yield end
    encode(OUT/'00_验收总览.mp4',frames())


def main():
    p=argparse.ArgumentParser();p.add_argument('--case');p.add_argument('--report',action='store_true');args=p.parse_args()
    OUT.mkdir(parents=True,exist_ok=True);plan=read(ROOT/'plan.json')
    if args.case:
        film(next(c for c in plan['cases']if c['id']==args.case),ROOT/'race_rule');return
    results,totals=report()
    if args.report:return
    for case in plan['cases']:film(case,ROOT/'race_rule')
    summary_video(results,totals)


if __name__=='__main__':main()
