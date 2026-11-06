using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace LapLab {
public sealed class Track {
    public Vector2[] curb, lanes, centers, dir, right;
    public float[] stations;
    public float length;
    public const float Lane=3.43051f;
    public static Vector2 XZ(Vector3 p) {return new Vector2(p.x,p.z);}
    public static Vector3 World(Vector2 p,float h=0){return new Vector3(p.x,h,p.y);}
    public int Wrap(int i){return (i%curb.Length+curb.Length)%curb.Length;}
    public Track(float[][] vertices,Transform surface){
        curb=vertices.Select(p=>XZ(surface.TransformPoint(new Vector3(p[0],0,p[1])))).ToArray();
        int n=curb.Length; dir=new Vector2[n];right=new Vector2[n];lanes=new Vector2[n];centers=new Vector2[n];stations=new float[n];
        for(int i=0;i<n;i++){
            dir[i]=(curb[Wrap(i+1)]-curb[Wrap(i-1)]).normalized;right[i]=new Vector2(dir[i].y,-dir[i].x);
            lanes[i]=curb[i]-right[i]*(Lane*.5f);centers[i]=curb[i]-right[i]*Lane;
        }
        for(int i=0;i<n;i++){stations[i]=length; length+=Vector2.Distance(lanes[i],lanes[Wrap(i+1)]);}
    }
    public int AtFraction(float fraction){
        int at=Array.BinarySearch(stations,Mathf.Repeat(fraction,1)*length);
        return at>=0?at:Wrap(~at);
    }
    public int Nearest(Vector2 pos,int previous){
        float best=float.PositiveInfinity;int pick=previous;for(int j=-9;j<=9;j++){
            int i=Wrap(previous+j);float e=(pos-lanes[i]).sqrMagnitude;if(e<best){best=e;pick=i;}
        }return pick;
    }
    public float Station(Vector2 p,int i){return stations[i]+Vector2.Dot(p-lanes[i],dir[i]);}
    public float Offset(Vector2 p,int i){return Vector2.Dot(p-lanes[i],right[i]);}
    // Mock semantic sensor. Only curb segments actually inside the local cone
    // supply lane geometry. Global station, lap progress and spawn stay in the referee.
    public float[] LocalLane(Vector2 pos,Vector2 forward,int near){
        var side=new Vector2(forward.y,-forward.x);var result=new float[Policy.Ahead.Length];
        var weights=new float[result.Length];var nearest=new float[result.Length];
        for(int a=0;a<nearest.Length;a++)nearest[a]=999;
        for(int j=-42;j<=48;j++){
            int i=Wrap(near+j);var boundary=curb[i]-pos;
            float z=Vector2.Dot(boundary,forward),x=Vector2.Dot(boundary,side);
            if(z<.1f||boundary.sqrMagnitude>12*12||Mathf.Abs(x)>z*4.0f||Vector2.Dot(dir[i],forward)<.18f)continue;
            var local=lanes[i]-pos;z=Vector2.Dot(local,forward);x=Vector2.Dot(local,side);
            for(int a=0;a<result.Length;a++){
                float distance=Mathf.Abs(z-Policy.Ahead[a]);if(distance>=.55f)continue;
                float w=.56f-distance;result[a]+=w*x;weights[a]+=w;nearest[a]=Mathf.Min(nearest[a],distance);
            }
        }
        for(int a=0;a<result.Length;a++)result[a]=weights[a]>0?result[a]/weights[a]:float.NaN;
        return result;
    }
}

public class Sight {public float z,x,vx,vz;}
public class Tracklet {
    public Vector2 p,v;public float t;public bool matched;
}
public sealed class ObjectTracker {
    readonly List<Tracklet> items=new List<Tracklet>();
    public List<Sight> Update(List<Vector2> measured,Vector2 pos,Vector2 forward,float t){
        var right=new Vector2(forward.y,-forward.x);
        items.RemoveAll(k=>t-k.t>1.25f);foreach(var k in items)k.matched=false;
        foreach(var local in measured.OrderBy(o=>o.x)){
            var world=pos+forward*local.x+right*local.y;Tracklet chosen=null;float best=1.15f;
            foreach(var k in items){if(k.matched)continue;float e=(world-(k.p+k.v*(t-k.t))).magnitude;if(e<best){best=e;chosen=k;}}
            if(chosen==null){chosen=new Tracklet{p=world,t=t};items.Add(chosen);}
            else {float age=t-chosen.t;if(age>.04f)chosen.v=Vector2.Lerp(chosen.v,Vector2.ClampMagnitude((world-chosen.p)/age,2.2f),.42f);}
            chosen.p=world;chosen.t=t;chosen.matched=true;
        }
        var outp=new List<Sight>();
        foreach(var k in items){
            var delta=k.p+k.v*(t-k.t)-pos;float z=Vector2.Dot(delta,forward),x=Vector2.Dot(delta,right);
            if(z>-1.2f && z<12.5f && Mathf.Abs(x)<5.8f)
                outp.Add(new Sight{z=z,x=x,vx=Vector2.Dot(k.v,right),vz=Vector2.Dot(k.v,forward)});
        }return outp;
    }
}

public sealed class Policy {
    public static readonly float[] Ahead={2f,3.5f,5.5f,8f};
    // pace, curvature sensitivity, steering response, lookahead, obstacle
    // clearance, onset distance of lateral maneuver, motion forecast multiplier.
    readonly float[] p;
    public float speedCmd,yawCmd,offset,curvature,chosenSpeed=1f;
    public int blind,calls,withObjects;
    public float cruise, look, target, preferred, guardCap, conflictRange;
    public string reason="start";
    float recovery;
    public Policy(float[] weights){p=weights;}
    static float Interpolate(float[] line,float s){
        if(s<=Ahead[0])return line[0];
        for(int i=1;i<Ahead.Length;i++)if(s<=Ahead[i])return Mathf.Lerp(line[i-1],line[i],(s-Ahead[i-1])/(Ahead[i]-Ahead[i-1]));
        return line[line.Length-1];
    }
    static float CenterAt(float[] line,float s){
        if(s<Ahead[0])return line[0]+(s-Ahead[0])*(line[1]-line[0])/(Ahead[1]-Ahead[0]);
        if(s>Ahead[3])return line[3]+(s-Ahead[3])*(line[3]-line[2])/(Ahead[3]-Ahead[2]);
        return Interpolate(line,s);
    }
    public bool Act(float[] line,List<Sight> objects,float speed,float yaw,float period){
        calls++;int n=line.Count(q=>!float.IsNaN(q));
        if(n<2){blind++;reason="insufficient_lane";speedCmd=blind<=2?Mathf.Min(speedCmd,1.2f):0;return blind<9;}
        blind=0;
        for(int i=0;i<line.Length;i++)if(float.IsNaN(line[i])){
            int a=-1,b=-1;
            for(int j=0;j<line.Length;j++)if(!float.IsNaN(line[j])){
                if(a<0||Mathf.Abs(j-i)<Mathf.Abs(a-i)){b=a;a=j;}else if(b<0||Mathf.Abs(j-i)<Mathf.Abs(b-i))b=j;
            }
            float slope=Mathf.Clamp((line[a]-line[b])/(Ahead[a]-Ahead[b]),-.9f,.9f);
            line[i]=line[a]+slope*(Ahead[i]-Ahead[a]);
        }
        float near=(line[1]-line[0])/1.5f,mid=(line[2]-line[1])/2f,far=(line[3]-line[2])/2.5f;
        curvature=(Mathf.Atan(far)-Mathf.Atan(near))/4f;
        cruise=Mathf.Clamp(p[0]/(1f+p[1]*Mathf.Abs(curvature)),1.05f,p[0]);
        // The two fitted gains choose a lateral racing line using only visible changes of direction.
        float lineBias=Mathf.Clamp(p[7]*(Mathf.Atan(mid)-Mathf.Atan(near))+
                                    p[8]*(Mathf.Atan(far)-Mathf.Atan(mid)),-1.15f,.82f);
        preferred=lineBias;float pace=1;float best=float.PositiveInfinity;
        bool guarded=p.Length>=12;
        conflictRange=99;
        foreach(var o in objects){
            if(o.z<-.9f)continue;
            float dist=Mathf.Abs(o.x-CenterAt(line,o.z)-lineBias);
            if(dist<1.65f+Mathf.Min(.85f,Mathf.Abs(o.vx)*1.1f))conflictRange=Mathf.Min(conflictRange,o.z);
        }
        bool yielding=conflictRange<12f;
        if(yielding)recovery=1.4f;else recovery=Mathf.Max(0,recovery-period);
        // Limit approach speed from the distance available before lateral displacement is needed.
        // Restore cruise only after the visible road and the body heading agree again.
        guardCap=cruise;reason="road";
        if(guarded){
            if(yielding){
                guardCap=Mathf.Sqrt(p[9]*p[9]+2f*1.25f*Mathf.Max(0,conflictRange-6.5f));
                reason="yield";
            }else if(recovery>0||Mathf.Abs(offset-lineBias)>.48f||Mathf.Abs(near)>.32f){
                guardCap=p[10];reason="recover";
            }
        }
        float planningSpeed=Mathf.Min(cruise,guardCap);
        if(objects.Count>0){
            withObjects++;
            foreach(float lateral in new[]{-3.65f,-3.08f,-2.6f,-2.05f,-1.55f,-1.15f,-.78f,0f,.70f,.92f}){
                foreach(float ratio in new[]{1f,.68f,.40f}){
                    float cost=.14f*Mathf.Abs(lateral-lineBias)+.22f*Mathf.Abs(lateral-offset)+(guarded?.48f:1.55f)*(1-ratio);
                    // Reduce the appeal of a path that leaves no space to correct toward the outside curb.
                    float edgeRoom=Mathf.Min(Track.Lane*.5f-lateral,Track.Lane*1.5f+lateral)-.38f;
                    cost+=.40f/Mathf.Max(.05f,edgeRoom);
                    foreach(var o in objects){
                        float tau=Mathf.Clamp((o.z-.28f)/Mathf.Max(.55f,.45f*speed+.55f*planningSpeed*ratio-o.vz),0,4.5f);
                        float ahead=Mathf.Clamp(o.z+o.vz*tau,-1.2f,13f);
                        float endX=CenterAt(line,ahead)+lateral;
                        float nowX=o.x, predictedX=nowX+o.vx*tau*p[6];
                        float low=Mathf.Min(nowX,predictedX), high=Mathf.Max(nowX,predictedX);
                        // The same first-order steering response applies to every candidate.
                        // A slower arrival gives the chassis more time to move sideways.
                        float u=tau*Mathf.Max(1.15f,11f/p[5]);
                        float attainable=endX*(1f-(1f+u)*Mathf.Exp(-u));
                        if(guarded){
                            // Camera practice around 3 m/s measured |v*yawRate| near 0.9–1.4 m/s².
                            // Body-frame lateral reach over this horizon limits late side changes.
                            float reach=.5f*1.20f*tau*tau;
                            attainable=Mathf.Clamp(attainable,-reach,reach);
                        }
                        float detectionWidth=.06f+Mathf.Min(.55f,.28f*tau*Mathf.Min(Mathf.Abs(o.vx),1.2f));
                        float required=.76f+p[4]+detectionWidth;
                        float sep=Mathf.Max(low-attainable,attainable-high,0);
                        float finalSep=Mathf.Max(low-endX,endX-high,0);
                        float shortfall=Mathf.Max(0,required-sep), terminal=Mathf.Max(0,required-finalSep);
                        cost+=30f*shortfall*shortfall+20f*terminal*terminal;
                        if(o.z<4.5f)cost+=Mathf.Clamp01((4.5f-o.z)/2f)*1.25f*Mathf.Abs(lateral-offset);
                    }
                    if(cost<best){best=cost;preferred=lateral;pace=ratio;}
                }
            }
        }
        float shiftStep=guarded?period*(yielding?2.5f:p[11]):.52f;
        offset=Mathf.MoveTowards(offset,preferred,shiftStep);chosenSpeed=pace;
        look=Mathf.Clamp(p[3]+.19f*speed,2.3f,6f);
        target=Interpolate(line,look)+offset;
        yawCmd=Mathf.Clamp(p[2]*Mathf.Atan2(target,look)-.38f*yaw+.9f*speed*curvature,-5f,5f);
        float requested=planningSpeed*pace;
        if(guarded){
            // The archived asymmetric motor map saturates beyond these yaw inputs.
            yawCmd=Mathf.Clamp(yawCmd,-1.60119f,1.69546f);
            speedCmd=Mathf.Min(requested,speedCmd+1.8f*period);
        }else speedCmd=requested;
        return true;
    }
}
}
