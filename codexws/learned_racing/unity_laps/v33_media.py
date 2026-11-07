"""Complete paired experimental films with actual source versions and criteria."""
import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
from acceptance_media import View,txt,encode,frame_pair,PANEL,BG,MUTED,INK,GREEN,BLUE,RED
from v33_trials import ROOT,read,save

class ExperimentView(View):
    def __init__(self,experiment,run_id,label):
        self.cohort=ROOT/experiment;self.r=read(self.cohort/'runs'/(run_id+'.json'))
        self.case=dict(limitM=self.r['length']*self.r['request'].get('limit',1.));self.version='V3_0' if label.startswith('V3.0') else 'V3_3'
        self.rows=np.asarray(self.r['rows']);self.ft=np.asarray(self.r['frames']);self.ds=self.r['decisions']
        self.ts=np.asarray([d['t']for d in self.ds]);self.wheels=np.asarray(self.r.get('wheelRows',[]))
        self.media=self.cohort/'.work/frames'/run_id;self.perception=self.cohort/'.work'/run_id/'frames'
        self.label=label
    def draw(self,canvas,x,t,total):
        super().draw(canvas,x,t,total)
        d=ImageDraw.Draw(canvas);d.rectangle((x+15,65,x+940,100),fill=PANEL)
        txt(d,x+24,68,self.label,24,BLUE if self.version=='V3_0' else GREEN,True)
        idx=max(0,int(np.searchsorted(self.ts,min(t,self.r['seconds'])+1e-6,side='right')-1))
        obs=self.ds[idx]['decision'];detail=obs.get('roadEvidence')
        d.rectangle((x+445,537,x+940,566),fill=PANEL)
        txt(d,x+452,542,'按各项净空与不确定度检查规划余量',18,MUTED)
        if 'sigma_definition' in obs.get('audit',{}):
            d.rectangle((x+445,766,x+940,791),fill=PANEL)
            txt(d,x+452,770,'无可行候选时只允许停车。' if obs['reason']!='feasible' else '比较可行速度与路线的前进得分。',18,RED if obs['reason']!='feasible' else GREEN,True)
        d.rectangle((x+445,791,x+940,851),fill=PANEL)
        if detail:
            txt(d,x+452,794,f'路缘观测 {detail["fresh_points"]} 点 / 历史 {detail["history_points"]} 点',17,MUTED)
            txt(d,x+452,821,f'历史最长 {detail["max_age_s"]:.1f}s；视场边缘不参与方向',17,GREEN)
        else:
            txt(d,x+452,794,'单帧色块；视场裁边参与道路方向',17,MUTED)
            txt(d,x+452,821,'此对照也使用 2.8 m/s 参数',17,MUTED)

def main():
    p=argparse.ArgumentParser();p.add_argument('--new',default='V3_3_1_temporal');p.add_argument('--run',default='01');p.add_argument('--label',default='V3.3.1 · 短时记忆 · 未重新训练')
    p.add_argument('--old',default='V3_0_same_speed');p.add_argument('--old-label',default='V3.0 · 原始路面判断 · 同速对照');a=p.parse_args()
    views=[ExperimentView(a.old,a.run,a.old_label),ExperimentView(a.new,a.run,a.label)]
    total=max(v.r['seconds'] for v in views);case=dict(id=a.run,title='同一随机起点 / 相同 2.8 m/s 巡航参数',limitM=views[0].r['length'])
    out=ROOT/'delivery';out.mkdir(exist_ok=True)
    def frames():
        cover=Image.new('RGB',(1920,1080),BG);d=ImageDraw.Draw(cover)
        txt(d,110,110,'基础试验：先证明 baseline，再训练',48,INK,True)
        txt(d,110,230,'左右同一起点、相同物理、相同速度参数；1×完整播放',34,GREEN)
        txt(d,110,310,'从静止开始，保留失败或完圈；先结束的一侧停留在终点',31)
        txt(d,110,415,'本片是开发场景改善，不代表新起点验收已通过',34,'#ffd278',True)
        txt(d,110,510,'新起点、识别抖动、连续/移动障碍门槛全部通过后才长训练',29)
        txt(d,110,620,'实际图像采样 5 Hz；10 fps 封装重复最新帧，不伪造中间画面',27,MUTED)
        for _ in range(35):yield cover
        for t in np.arange(0,total+3.1,.1):yield frame_pair(views,case,float(t),total)
    path=out/(a.new+'_'+a.run+'_同速完整对照.mp4');encode(path,frames())
    for label,t in [('start',0),('middle',total*.5),('end',total)]:
        frame_pair(views,case,t,total).save(out/(a.new+'_'+a.run+'_'+label+'.jpg'),quality=90)
    print(path,flush=True)
if __name__=='__main__':main()
