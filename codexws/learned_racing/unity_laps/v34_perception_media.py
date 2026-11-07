"""Same RGB/mask causal replay film; no synthetic driving frames."""
import argparse
import cv2
import numpy as np
from PIL import Image,ImageDraw
from acceptance_media import txt,encode,BG,PANEL,INK,MUTED,GREEN,BLUE,YELLOW
from vision import FOCAL,CX,CY
from v3_vision import VisibleRoad
from v34_road import EvidenceRoad
from v33_trials import ROOT,read

def main():
    p=argparse.ArgumentParser();p.add_argument('experiment');p.add_argument('--run',default='05');a=p.parse_args()
    root=ROOT/a.experiment;r=read(root/'runs'/(a.run+'.json'));ft=np.asarray(r['frames'])
    models=[VisibleRoad(),EvidenceRoad(False),EvidenceRoad(True)]
    labels=['V3.0 原始路面处理','V3.4.5 空间处理','V3.4.5 加入轮速 / gyro 短时记忆']
    out=ROOT/'delivery';out.mkdir(exist_ok=True)
    def frames():
        for i,d in enumerate(r['decisions']):
            q=d['decision'];fi=int(ft[np.argmin(abs(ft[:,0]-d['t'])),1]);dt=.2 if i==0 else d['t']-r['decisions'][i-1]['t']
            rgb=cv2.imread(str(root/'.work/frames'/a.run/f'{fi:04d}f.jpg'))
            mask=cv2.imread(str(root/'.work'/a.run/'frames'/f'{i:05d}_mask.png'),0)
            if rgb is None or mask is None:raise RuntimeError('Missing evidence frame')
            canvas=Image.new('RGB',(1920,1080),BG);dr=ImageDraw.Draw(canvas)
            txt(dr,25,15,'同一相机画面：为什么近处的可行驶区域会跳？',34,INK,True)
            txt(dr,25,65,f'{a.experiment} / {a.run} · {d["t"]:.1f} 秒 · 离线回放，三栏没有重新开车',22,MUTED)
            for k,model in enumerate(models):
                x=k*640
                args=(rgb,mask) if k==0 else (rgb,mask,q['speedEstimate'],q['sensor']['gyro'],dt,q.get('detectedBoxes',()))
                result=model.read(*args);road=np.asarray(result[0]);pic=rgb.copy()
                color=(100,220,110) if k else (250,160,80)
                for side in (1,2):
                    pts=[]
                    for z,left,right,sigma in road:
                        xx=left if side==1 else right;u=int(CX+FOCAL*xx/z);v=int(CY+.655*FOCAL/z)
                        pts.append([max(0,min(639,u)),max(0,min(359,v))])
                    if len(pts)>1:cv2.polylines(pic,[np.asarray(pts)],False,color,3)
                txt(dr,x+15,110,labels[k],23,GREEN if k else BLUE,True)
                canvas.paste(Image.fromarray(cv2.cvtColor(pic,cv2.COLOR_BGR2RGB)),(x,153))
                near=Image.fromarray(cv2.cvtColor(pic[250:360],cv2.COLOR_BGR2RGB)).resize((630,220))
                canvas.paste(near,(x+5,550));txt(dr,x+15,517,'近场放大：裂纹 / 白色涂装 / 桶的遮挡',19,MUTED)
                txt(dr,x+15,785,'距离       左边界      右边界      不确定度',19,MUTED)
                for j,z in enumerate((1.2,2.,3.)):
                    vals=[np.interp(z,road[:,0],road[:,c]) for c in (1,2,3)] if len(road) else [np.nan]*3
                    txt(dr,x+15,820+j*32,f'{z:.1f} m       {vals[0]:+5.2f} m      {vals[1]:+5.2f} m       {vals[2]:.2f} m',20)
                if k:
                    e=model.last;txt(dr,x+15,930,f'新观测 {e["fresh_points"]} 点；历史 {e["history_points"]} 点',19,GREEN)
                    txt(dr,x+15,961,f'历史最长 {e["max_age_s"]:.1f}s；遮挡补线 {e["occlusion_bridged_pixels"]} 像素',18,MUTED)
                else:txt(dr,x+15,930,'独立扫描线；把视场裁边也当作道路边缘',19,YELLOW)
            txt(dr,25,1010,'线条是道路估计，不是实测真值。遮挡处仅推断路面连续性，锥桶仍按障碍占用处理。',22,YELLOW)
            txt(dr,25,1046,'同一已保存 RGB JPEG / A4 mask；原在线输入为 PNG。未训练；不代表感知准入已通过。',19,MUTED)
            if i in (0,len(r['decisions'])//2,len(r['decisions'])-1):canvas.save(out/f'{a.experiment}_{a.run}_perception_{i:04d}.jpg',quality=91)
            yield canvas;yield canvas
    path=out/f'{a.experiment}_{a.run}_near_field.mp4';encode(path,frames());print(path,flush=True)
if __name__=='__main__':main()
