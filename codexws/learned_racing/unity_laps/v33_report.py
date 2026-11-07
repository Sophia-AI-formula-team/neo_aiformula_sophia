"""Build a local review bundle from retained real experiments, including failures."""
import hashlib,html,json
from pathlib import Path
import av
import numpy as np
from PIL import Image,ImageDraw
from acceptance_media import txt,frame_pair,BG,INK,MUTED,GREEN,RED,YELLOW,PANEL
from v33_media import ExperimentView
from v33_trials import ROOT,read,save

PAIRS=[
 ('V3_3_4_association','01','V3_0_same_speed','同速完整圈：V3.0 → V3.3.4'),
 ('V3_4_5_clear','01','V3_3_4_association','必须保留的退步：V3.4.5 空场停滞'),
 ('V3_4_5_tangent','05','V3_3_4_obstacles','A：中缝可通过'),
 ('V3_4_5_tangent','06','V3_3_4_obstacles','B：中缝不够，需绕行'),
 ('V3_4_5_tangent','07','V3_3_4_obstacles','C：中缝与绕行均可选择'),
 ('V3_4_5_tangent','12','V3_3_4_obstacles','连续障碍：新版出界，拒收'),
 ('V3_4_3_obstacles','14','V3_3_4_obstacles','移动障碍：仅 V3.4.3 开发结果'),
 ('V3_4_3_obstacles','16','V3_3_4_obstacles','弯中混合：仅 V3.4.3 开发结果')]

def main():
    out=ROOT/'delivery';entries=[]
    for exp,rid,old,title in PAIRS:
        files=list(out.glob(f'{exp}_{rid}_*.mp4'))
        if len(files)!=1:raise RuntimeError((exp,rid,files))
        version=read(ROOT/exp/'manifest.json')['arguments'].get('version','V3.3.4')
        oldversion=read(ROOT/old/'manifest.json')['arguments'].get('version','V3.3.4')
        if old=='V3_0_same_speed':oldversion='V3.0'
        views=[ExperimentView(old,rid,oldversion+' · 驾驶参数未重新训练'),ExperimentView(exp,rid,version+' · 驾驶参数未重新训练')]
        total=max(v.r['seconds'] for v in views);case=dict(id=rid,title=title,limitM=views[0].case['limitM'])
        for tag,t in [('start',0),('middle',total*.5),('end',total)]:
            frame_pair(views,case,t,total).save(out/f'{exp}_{rid}_{tag}.jpg',quality=90)
        entries.append(dict(file=files[0].name,title=title,poster=f'{exp}_{rid}_middle.jpg',old=old,new=exp,run=rid,status=views[1].r['status'],seconds=views[1].r['seconds']))
    entries.append(dict(file='V3_4_4_occlusion_05_near_field.mp4',title='同一 RGB / A4 mask 的近场回放',poster='V3_4_4_occlusion_05_perception_0040.jpg'))
    qa=[]
    for entry in entries:
        p=out/entry['file'];n=0
        with av.open(str(p)) as c:
            stream=c.streams.video[0];width,height=stream.width,stream.height
            for frame in c.decode(video=0):n+=1
            duration=float(stream.duration*stream.time_base)
        if n<2 or (width,height)!=(1920,1080):raise RuntimeError('Invalid video '+str(p))
        qa.append(dict(file=p.name,sha256=hashlib.sha256(p.read_bytes()).hexdigest(),bytes=p.stat().st_size,frames_decoded=n,width=width,height=height,duration_s=duration))
    save(out/'video_manifest.json',dict(files=qa,upload_status='not uploaded: Computer Use timed out twice and after session reset',visual_review='Representative actual first/middle/end frames inspected; complete video streams decoded'))
    im=Image.new('RGB',(1600,1060),BG);d=ImageDraw.Draw(im)
    txt(d,45,30,'V3.3 / V3.4 试验验收：还不能开始长训练',43,INK,True)
    txt(d,45,95,'49 次真实 Unity 闭环试验 · 新增长训练 0 次 · 不改物理与裁判',27,MUTED)
    txt(d,45,155,'版本 / 试验',27,MUTED);txt(d,650,155,'结果',27,MUTED);txt(d,1300,155,'是否准入训练',27,MUTED)
    rows=[('V3.0 同速空场','两个整圈都提前出界','否'),('V3.3.4 开发空场','2/2 完整圈；112.74 / 115.30 s','否'),('V3.3.4 新起点验收','第3圈 144.50 s，低于2 m/s门槛','失败即停止'),('V3.4.3 障碍开发','3/8 通过；有停滞和出界','否'),('V3.4.5 双桶 A / B / C','3/3 开发布局通过','仅局部改善'),('V3.4.5 连续障碍','出界；新版空场 0/2 完圈','整版拒收')]
    for i,(a,b,c) in enumerate(rows):
        y=207+i*72;d.rounded_rectangle((30,y-4,1570,y+58),10,fill=PANEL)
        txt(d,45,y,a,25);txt(d,650,y,b,24,GREEN if i in (1,4) else YELLOW);txt(d,1300,y,c,23,RED)
    txt(d,45,675,'新训练准入：所有条件一起过，不能只挑成功视频',31,INK,True)
    for i,s in enumerate(['近场边界稳定性 + 覆盖 / 误扩张：尚未通过','8 个新起点整圈 + 4 圈 lane-mask 扰动：未通过 / 未运行','16 段新障碍布局 + 执行预测验证：尚未运行 / 未通过']):txt(d,65,730+i*44,s,26,MUTED)
    txt(d,45,905,'方法：A4 识别 + RGB 路面 + 轮速/gyro 记忆 + 显式运动/碰撞预测',26,GREEN)
    txt(d,45,950,'没有地图、GNSS、全局位置输入策略；两车道类别先验已在记录中披露。',24,MUTED)
    txt(d,45,995,'当前主要阻塞：路界估计冲突、停车后的恢复、连续障碍与执行预测。',24,YELLOW)
    im.save(out/'验收总览.png')
    cards=[]
    for e in entries:
        cards.append(f'<article><h2>{html.escape(e["title"])}</h2><video controls preload="metadata" poster="{html.escape(e["poster"])}" src="{html.escape(e["file"])}"></video><p>{html.escape(e.get("new","同输入回放"))} · {html.escape(e.get("status","不是闭环验收"))}</p></article>')
    document='''<!doctype html><html lang="zh"><meta charset="utf-8"><title>V3.3 / V3.4 验收</title><style>body{background:#101720;color:#eff5fb;font:18px system-ui;margin:30px auto;max-width:1480px;line-height:1.6}h1{font-size:36px}.warn{color:#ffd278}video,img{max-width:100%;width:100%}article{padding:22px;background:#1b2635;margin:24px 0;border-radius:14px}h2{font-size:25px}a{color:#50e3b4}</style><h1>当前结论：未准入训练</h1><p>这轮完成49次Unity试验，所有失败保留。短段通过不代表完整圈可用；提前失败的时间不能拿来比较圈速。新增长训练0次。</p><p class="warn">视频含完整初始化、跟车、A4识别、当前局部道路、候选路线、净空与评分、速度及实际终止原因。近场回放使用同一保存的JPEG和A4 mask；原在线输入为PNG。两车道类别先验不是米制地图，但仍限制了当前方案的通用性。</p><img src="验收总览.png"><p>已训练的A4视觉模型与既有数据拟合车辆模型仍在使用；新驾驶参数没有进行长训练。当前路线与风险比较是可解释的几何模型。风险余量不是经过标定的通过概率。</p>'''+''.join(cards)+'''<p class="warn">本轮尚未上传：Computer Use 连接在重试和重置后仍超时。没有覆盖上一轮 Drive 文件。</p><p><a href="video_manifest.json">视频完整解码检查与SHA256</a> · <a href="training_admission.json">训练准入状态</a> · <a href="PROTOCOL.md">完整验收标准</a></p></html>'''
    (out/'index.html').write_text(document,encoding='utf-8')
    for name in ('training_admission.json','PROTOCOL.md','all_trials.json'):(out/name).write_bytes((ROOT/name).read_bytes())
    print('Videos checked:',len(qa),'total seconds:',sum(q['duration_s'] for q in qa),flush=True)
if __name__=='__main__':main()
