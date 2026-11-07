"""Offline referee views and training plots. These never enter the controller."""
import json
import numpy as np
from PIL import Image,ImageDraw
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from acceptance_media import ROOT,OUT,BG,INK,MUTED,GREEN,RED,BLUE,YELLOW,txt,encode,track,View,STATUS
from acceptance_run import read,save


def training_plot():
    hist=read(ROOT/'frozen/history.json');model=read(ROOT/'frozen/V3_2.json');base=read(ROOT/'frozen/V3_0.json')
    fp=FontProperties(fname='C:/Windows/Fonts/msyh.ttc')
    plt.rcParams.update({'font.family':fp.get_name(),'axes.unicode_minus':False,'font.size':12})
    fig,axes=plt.subplots(2,2,figsize=(16,10),facecolor='#101720',gridspec_kw={'height_ratios':[1,.9]})
    for ax in axes.flat:
        ax.set_facecolor('#1b2635');ax.tick_params(colors='#a9b8ca')
        for spine in ax.spines.values():spine.set_color('#526076')
        ax.title.set_color('white');ax.xaxis.label.set_color('#a9b8ca');ax.yaxis.label.set_color('#a9b8ca')
    x=[h['attempts']for h in hist]
    axes[0,0].plot(x,[h['success']*100 for h in hist],color=GREEN,label='训练群体完圈率')
    axes[0,0].plot(x,[h['collision']*100 for h in hist],color=RED,label='训练群体碰撞率')
    axes[0,0].set(title='每一轮训练群体的成绩（不是最终模型成功率）',xlabel='累计试错次数',ylabel='%');axes[0,0].legend()
    axes[0,1].plot(x,[h['mean_speed']for h in hist],color=BLUE)
    axes[0,1].set(title='训练群体平均速度（包含失败试次）',xlabel='累计试错次数',ylabel='m/s')
    axes[1,0].axis('off');stats=read(ROOT/'plan.json')['training']
    lines=['训练：30,000 次 / 22,736 个不同场景 / 118 轮',
           f"全部试次累计：完圈 {stats['completed']:,}；碰撞 {stats['contact']:,}；出界 {stats['departure']:,}",
           '训练环境：程序化简化动力学 + 局部符号传感器',
           '验收环境：Unity 原生物理 + 实际 A4/RGB 相机闭环',
           '独立验收：V3.0 和 V3.2 均为 2/16 场景完成；整圈 0/3',
           '训练曲线变好，不能替代独立驾驶验收。']
    for i,line in enumerate(lines):axes[1,0].text(.02,.95-i*.16,line,transform=axes[1,0].transAxes,color='#eff5fb',fontsize=12,va='top')
    axes[1,1].axis('off')
    table=axes[1,1].table(cellText=[[key,f'{base["parameters"][key]:.3f}',f'{val:.3f}']for key,val in model['parameters'].items()],
                          colLabels=['可解释参数','V3.0','V3.2'],loc='center',cellLoc='center')
    table.auto_set_font_size(False);table.set_fontsize(12);table.scale(1,1.7)
    fig.suptitle('训练过程与独立验收：两种证据分开看',color='white',fontsize=25)
    fig.tight_layout(rect=[0,0,1,.95]);fig.savefig(OUT/'00_训练过程.png',dpi=140);plt.close(fig)


def model_prediction_error():
    out=[]
    for p in sorted((ROOT/'race_rule/runs').glob('*.json')):
        r=read(p);rows=np.array(r['rows']);samples=[]
        for d in r['decisions']:
            obs=d['decision'];a=obs['audit'];h=a.get('horizon')
            if h is None or d['t']+h>r['seconds']:continue
            initial=np.array([np.interp(d['t'],rows[:,0],rows[:,k])for k in (1,2,3)])
            final=np.array([np.interp(d['t']+h,rows[:,0],rows[:,k])for k in (1,2)])
            forward=np.array([np.sin(initial[2]),np.cos(initial[2])]);right=np.array([forward[1],-forward[0]])
            diff=final-initial[:2];actual=np.array([diff@forward,diff@right]);predicted=np.array(obs['path'][-1])
            samples.append(dict(t=d['t'],horizon=h,error_m=float(np.linalg.norm(actual-predicted))))
        out.append(dict(id=r['id'],samples=len(samples),p50_m=float(np.median([s['error_m']for s in samples]))if samples else None,
                        p95_m=float(np.quantile([s['error_m']for s in samples],.95))if samples else None,
                        limitation='Open-loop candidate forecast versus subsequent re-planned closed-loop travel. Includes later control changes; not a pure plant-fit error.'))
    save(ROOT/'prediction_followup.json',out)


def referee_frame(views,case,t,total):
    im=Image.new('RGB',(1920,1080),BG);d=ImageDraw.Draw(im)
    txt(d,40,22,case['id']+' '+case['title']+'：场景与实际通行',34,INK,True)
    txt(d,40,78,'裁判俯视复核 — 世界坐标只用于这张图，绝不输入驾驶模型',25,YELLOW,True)
    geom=track()
    allpos=np.vstack([v.rows[:,1:3]for v in views]);lo=allpos.min(axis=0)-8;hi=allpos.max(axis=0)+8
    for ci,v in enumerate(views):
        x=ci*960;r=v.r;clock=min(t,r['seconds']);end=t>=r['seconds'];color=BLUE if ci==0 else GREEN
        txt(d,x+40,125,v.version.replace('_','.')+f'   {clock:.2f} s',28,color,True)
        tile=Image.new('RGB',(880,665),'#122033');g=ImageDraw.Draw(tile)
        scale=min(850/(hi[0]-lo[0]),630/(hi[1]-lo[1]));center=(lo+hi)/2
        def q(points):return (np.asarray(points)-center)*[scale,-scale]+[440,332]
        def points(a):return [tuple(map(float,p))for p in a]
        g.polygon(points(q(geom[0])),fill='#41515e');g.polygon(points(q(geom[2])),fill='#122033')
        for a in geom:g.line(points(q(a))+[tuple(q(a[0]))],fill='#8e9aa7',width=2)
        before=v.rows[v.rows[:,0]<=clock+1e-6];row=before[-1]
        if len(before)>1:g.line(points(q(before[:,1:3])),fill=color,width=4)
        forward=np.array([np.sin(row[3]),np.cos(row[3])]);right=np.array([forward[1],-forward[0]])
        body=np.array([row[1:3]+forward*z+right*xx for z,xx in ((.28,-.38),(.28,.38),(-.88,.38),(-.88,-.38))])
        g.polygon(points(q(body)),fill=color,outline='white')
        for j,p in enumerate(r['props']):
            world=np.array(p[:2])+np.array(p[2:4])*p[4]*np.sin(p[5]+clock*p[6]);pt=q(world);radius=max(4,.34*scale)
            g.ellipse((pt[0]-radius,pt[1]-radius,pt[0]+radius,pt[1]+radius),fill=YELLOW)
            # Instantaneous true lateral velocity, clearly a referee overlay.
            vel=np.array(p[2:4])*p[4]*p[6]*np.cos(p[5]+clock*p[6]);tip=q(world+vel)
            g.line([tuple(pt),tuple(tip)],fill=YELLOW,width=3)
            if 0<pt[0]<840 and 0<pt[1]<620:txt(g,pt[0]+8,pt[1]-18,str(j+1)+(' 动'if p[4]else' 静'),16,YELLOW)
        g.line((25,630,25+5*scale,630),fill='white',width=3);txt(g,25,600,'5 m',19)
        im.paste(tile,(x+40,175));d=ImageDraw.Draw(im)
        txt(d,x+40,857,f'车身速度 {row[4]:.2f} m/s   进度 {row[7]:.1f} / {case["limitM"]:.1f} m',25,color)
        txt(d,x+40,902,f'最近桶实际净空 {row[10]:.2f} m'if r['props']else'无障碍物',23,MUTED)
        if end:txt(d,x+40,948,'结果：'+STATUS.get(r['status'],r['status']),30,GREEN if r['status']in('lap','segment')else RED,True)
    txt(d,40,1030,'固定坐标观察移动桶：黄色箭头为实际瞬时速度；车线与桶真值仅供裁判复核。右侧定格代表已结束。',22,MUTED)
    return im


def referee_video():
    def frames():
        for c in read(ROOT/'plan.json')['cases']:
            views=[View(c,v,ROOT/'race_rule')for v in ('V3_0','V3_2')];total=max(v.r['seconds']for v in views)
            for t in np.arange(0,total+2,.1):yield referee_frame(views,c,float(t),total)
    encode(OUT/'17_裁判俯视_静止移动连续障碍.mp4',frames())


if __name__=='__main__':
    OUT.mkdir(parents=True,exist_ok=True)
    training_plot();model_prediction_error();referee_video()
