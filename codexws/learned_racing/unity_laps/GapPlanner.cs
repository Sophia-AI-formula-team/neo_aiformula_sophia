using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace LapLab {
public sealed class GapConfig {
    public bool openings;
    public float paceScale, padding, lateralRate, responseTime;
    public float[] speedFractions;
}
public sealed class GapDecision {
    public string state;
    public float speed,offset,travelTime;
    // Road-frame [range, legal left, legal right, selected x] for each visible row.
    public float[][] crossings;
}
public sealed class GapPlanner {
    const float HalfWidth=.38f, Front=.28f, Rear=.88f, Radius=.34f;
    readonly GapConfig cfg;
    float held,remaining;
    bool inPass;
    public GapDecision Decision;
    sealed class Row {
        public float first,last;
        public List<Sight> tracks=new List<Sight>();
    }
    sealed class PathState {
        public float x,time,tie;
        public List<float[]> path=new List<float[]>();
    }
    public GapPlanner(GapConfig config){cfg=config;}
    static float Slope(float[] line,float z){
        return (Policy.CenterAt(line,z+.25f)-Policy.CenterAt(line,z-.25f))/.5f;
    }
    List<Row> Rows(float[] line,List<Sight> objects,float speed){
        var rows=new List<Row>();Row row=null;
        foreach(var track in objects.Where(o=>o.z>=-Rear-Radius && o.z<=12f).OrderBy(o=>o.z)){
            // A4 may also track roadside objects. Keep those which can enter
            // the physical road during the car's approach and passage.
            float slope=Slope(line,track.z),norm=Mathf.Sqrt(1+slope*slope);
            float cross=(track.x-Policy.CenterAt(line,track.z))/norm;
            float velocity=(track.fitVx-slope*track.fitVz)/norm;
            float horizon=Mathf.Max(0,track.z+Rear+Radius)/Mathf.Max(.5f,speed);
            float future=cross+velocity*horizon;
            float room=Radius+cfg.padding+Mathf.Min(.5f,track.noiseX+Mathf.Abs(slope)*track.noiseZ);
            if(Mathf.Min(cross,future)-room>.5f*Track.Lane ||
               Mathf.Max(cross,future)+room< -1.5f*Track.Lane)continue;
            if(row==null || track.z-row.last>Front+Rear+2*Radius){
                row=new Row{first=track.z};rows.Add(row);
            }
            row.last=track.z;row.tracks.Add(track);
        }
        return rows;
    }
    List<Vector2> Openings(float[] line,Row row,float speed,float pace){
        float left=-1.5f*Track.Lane+HalfWidth+cfg.padding;
        float right=.5f*Track.Lane-HalfWidth-cfg.padding;
        var occupied=new List<Vector2>();
        foreach(var cone in row.tracks){
            float slope=Slope(line,cone.z),norm=Mathf.Sqrt(1+slope*slope);
            float x=(cone.x-Policy.CenterAt(line,cone.z))/norm;
            float lateral=(cone.fitVx-slope*cone.fitVz)/norm;
            float along=(cone.fitVz+slope*cone.fitVx)/norm;
            float relative=Mathf.Max(.5f,.5f*(pace+speed)-along);
            float enter=Mathf.Max(0,cone.z-Front-Radius)/relative;
            float leave=Mathf.Max(0,cone.z+Rear+Radius)/relative;
            float a=x+lateral*enter,b=x+lateral*leave;
            float margin=HalfWidth+Radius+cfg.padding+Mathf.Min(.5f,cone.noiseX+Mathf.Abs(slope)*cone.noiseZ);
            occupied.Add(new Vector2(Mathf.Min(a,b)-margin,Mathf.Max(a,b)+margin));
        }
        var free=new List<Vector2>();float end=left;
        foreach(var band in occupied.OrderBy(b=>b.x)){
            if(end>=right)break;
            if(band.x>end)free.Add(new Vector2(end,Mathf.Min(right,band.x)));
            end=Mathf.Max(end,band.y);
        }
        if(end<right)free.Add(new Vector2(end,right));
        return free;
    }
    static float Clamp(float x,Vector2 band){return Mathf.Clamp(x,band.x,band.y);}
    PathState Search(float[] line,List<Row> rows,float speed,float pace,float ego,float bias){
        var openings=rows.Select(row=>Openings(line,row,speed,pace)).ToList();
        var states=new List<PathState>{new PathState{x=ego}};
        float previousExit=0;
        for(int j=0;j<rows.Count;j++){
            var next=new List<PathState>();var row=rows[j];
            float travel=Mathf.Max(0,row.first-Front-Radius-previousExit);
            float dwell=Mathf.Max(0,row.last+Rear+Radius)-Mathf.Max(0,row.first-Front-Radius);
            float attainable=cfg.lateralRate*Mathf.Max(0,travel/Mathf.Max(.5f,j==0?(pace+speed)*.5f:pace)-cfg.responseTime);
            foreach(var last in states)foreach(var band in openings[j]){
                var targets=new List<float>{Clamp(last.x,band),Clamp(bias,band),(band.x+band.y)*.5f};
                if(inPass)targets.Add(Clamp(held,band));
                if(j+1<rows.Count)foreach(var after in openings[j+1])
                    targets.Add(Clamp(Clamp(last.x,after),band));
                foreach(float x in targets.Distinct()){
                    float dx=Mathf.Abs(x-last.x);
                    if(dx>attainable+.035f)continue;
                    float time=last.time+(Mathf.Sqrt(travel*travel+dx*dx)+dwell)/pace;
                    // Break nearly equal-time ties using the current aim and the free band's midpoint.
                    float tie=last.tie+.015f*Mathf.Abs(x-bias)+.012f*Mathf.Abs(x-(band.x+band.y)*.5f);
                    if(j==0 && inPass)tie+=.06f*Mathf.Abs(x-held);
                    var path=new List<float[]>(last.path){new[]{row.first,band.x,band.y,x}};
                    next.Add(new PathState{x=x,time=time,tie=tie,path=path});
                }
            }
            states=next.OrderBy(p=>p.time+p.tie).GroupBy(p=>Mathf.RoundToInt(100*p.x)).Select(g=>g.First()).Take(20).ToList();
            if(states.Count==0)return null;
            previousExit=Mathf.Max(0,row.last+Rear+Radius);
        }
        return states.OrderBy(p=>p.time+p.tie).First();
    }
    public bool Plan(float[] line,List<Sight> observed,float speed,float nominal,float bias,float offset,float dt){
        Decision=new GapDecision{state="base"};float fast=nominal*cfg.paceScale;
        remaining-=speed*dt;
        var rows=Rows(line,observed,speed);
        if(rows.Count==0){
            if(inPass && remaining>0){
                Decision.state="hold_until_rear";Decision.speed=fast;Decision.offset=held;return true;
            }
            inPass=false;
            if(cfg.paceScale==1)return false;
            Decision.state="clear_pace";Decision.speed=fast;Decision.offset=bias;return true;
        }
        if(!cfg.openings)return false;
        float slope=Slope(line,0),ego=-Policy.CenterAt(line,0)/Mathf.Sqrt(1+slope*slope);
        PathState best=null;float selected=0;
        foreach(float ratio in cfg.speedFractions){
            float pace=ratio*fast;
            var path=Search(line,rows,speed,pace,ego,bias);
            if(path!=null && (best==null || path.time+path.tie<best.time+best.tie)){best=path;selected=pace;}
        }
        if(best==null){Decision.state="no_reachable_opening";return false;}
        held=best.path[0][3];remaining=Mathf.Max(0,rows[0].last+Rear+Radius);inPass=true;
        Decision.state="visible_opening";Decision.offset=held;Decision.speed=selected;
        Decision.travelTime=best.time;Decision.crossings=best.path.ToArray();return true;
    }
}
}
