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

public class Sight {public float z,x,vx,vz,history,age,fitVx,fitVz,noiseX,noiseZ,coherence;public int id;}
public class Tracklet {
    public Vector2 p,v;public float t;public bool matched;
    public int id;
    public List<Vector3> history=new List<Vector3>();
}
public sealed class ObjectTracker {
    readonly List<Tracklet> items=new List<Tracklet>();
    int nextId;
    public List<Sight> Update(List<Vector2> measured,Vector2 pos,Vector2 forward,float t){
        var right=new Vector2(forward.y,-forward.x);
        items.RemoveAll(k=>t-k.t>1.25f);foreach(var k in items)k.matched=false;
        foreach(var local in measured.OrderBy(o=>o.x)){
            var world=pos+forward*local.x+right*local.y;Tracklet chosen=null;float best=1.15f;
            foreach(var k in items){if(k.matched)continue;float e=(world-(k.p+k.v*(t-k.t))).magnitude;if(e<best){best=e;chosen=k;}}
            if(chosen==null){chosen=new Tracklet{p=world,t=t,id=++nextId};items.Add(chosen);}
            else {float age=t-chosen.t;if(age>.04f)chosen.v=Vector2.Lerp(chosen.v,Vector2.ClampMagnitude((world-chosen.p)/age,2.2f),.42f);}
            chosen.p=world;chosen.t=t;chosen.matched=true;
            chosen.history.RemoveAll(s=>t-s.z>1.21f);
            chosen.history.Add(new Vector3(world.x,world.y,t));
        }
        var outp=new List<Sight>();
        foreach(var k in items){
            var delta=k.p+k.v*(t-k.t)-pos;float z=Vector2.Dot(delta,forward),x=Vector2.Dot(delta,right);
            if(z<=-1.2f || z>=12.5f || Mathf.Abs(x)>=5.8f)continue;
            var s=new Sight{z=z,x=x,vx=Vector2.Dot(k.v,right),vz=Vector2.Dot(k.v,forward),id=k.id,age=t-k.t};
            var h=k.history;
            if(h.Count>=4){
                var mean=Vector3.zero;foreach(var q in h)mean+=q;mean/=h.Count;
                float tt=0;var st=Vector2.zero;
                foreach(var q in h){float dt=q.z-mean.z;tt+=dt*dt;st+=new Vector2(q.x-mean.x,q.y-mean.y)*dt;}
                var fit=st/Mathf.Max(.001f,tt);
                s.fitVx=Vector2.Dot(fit,right);s.fitVz=Vector2.Dot(fit,forward);s.history=h[h.Count-1].z-h[0].z;
                foreach(var q in h){
                    var residual=new Vector2(q.x-mean.x,q.y-mean.y)-fit*(q.z-mean.z);
                    s.noiseX=Mathf.Max(s.noiseX,Mathf.Abs(Vector2.Dot(residual,right)));
                    s.noiseZ=Mathf.Max(s.noiseZ,Mathf.Abs(Vector2.Dot(residual,forward)));
                }
                // Both independent half-window displacements must support the fitted direction.
                var middle=h[h.Count/2];var a=(Vector2)(middle-h[0]);var b=(Vector2)(h[h.Count-1]-middle);
                s.coherence=Mathf.Sign(s.fitVx)*Vector2.Dot(a,right);
                s.coherence=Mathf.Min(s.coherence,Mathf.Sign(s.fitVx)*Vector2.Dot(b,right));
            }
            outp.Add(s);
        }return outp;
    }
}

// Local, opt-in temporal alternative to the frozen lateral planner. Distances are
// in metres, times in seconds, velocities in the observed road's moving frame.
public sealed class TimingEvidence {
    public string state="no_track";
    public bool offered;
    public int id;
    public float range,cross,motion,history,fitNoise,rangeNoise,along,frontFast,rearFast,frontPace,rearPace,rawFast,rawPace,marginFast,marginPace,remaining;
}
public sealed class LocalTiming {
    readonly bool enabled;
    int passing=-1;
    float remaining;
    public TimingEvidence evidence;
    public LocalTiming(bool enabled){this.enabled=enabled;}

    // Conservative acceleration after a command update. Integrate the same
    // acceleration cap and target speed for both front and rear arrival.
    static float Arrive(float distance,float speed,float target,float objectSpeed){
        float travelled=0;
        float v=speed;
        for(int i=1;i<=160;i++){
            if(i>3)v=Mathf.MoveTowards(v,target,1.45f*.025f);
            travelled+=Mathf.Max(.05f,v-objectSpeed)*.025f;
            if(travelled>=distance)return i*.025f;
        }
        return 4.025f;
    }

    static float Clearance(float cross,float velocity,float first,float last,float padding){
        float a=cross+velocity*first,b=cross+velocity*last;
        float closest=a*b<=0?0:Mathf.Min(Mathf.Abs(a),Mathf.Abs(b));
        return closest-padding;
    }

    public bool Act(float[] line,List<Sight> objects,float speed,float command,float boost,float pace,float bias,float offset,float yaw,float dt){
        evidence=new TimingEvidence();
        if(passing>=0){
            remaining-=speed*dt;
            var track=objects.Find(o=>o.id==passing);
            bool intruder=objects.Exists(o=>o.id!=passing && o.z>-.5f && o.z<remaining+1f);
            bool drift=track!=null && track.z>1.8f && track.age<.23f &&
                Mathf.Abs(track.x-Policy.CenterAt(line,track.z)-bias)<.86f;
            if(remaining<=0 || intruder || drift || (remaining>4f && (track==null || track.age>.41f))){
                passing=-1;evidence.state=remaining<=0?"finished":"abort";return false;
            }
            evidence.id=passing;evidence.remaining=remaining;evidence.state="committed";return true;
        }
        if(objects.Count!=1){evidence.state=objects.Count==0?"no_track":"multiple_tracks";return false;}
        var o1=objects[0];var e=evidence;e.id=o1.id;e.range=o1.z;e.history=o1.history;
        float slope=(Policy.CenterAt(line,o1.z+.35f)-Policy.CenterAt(line,o1.z-.35f))/.70f;
        float scale=Mathf.Sqrt(1+slope*slope);
        e.cross=(o1.x-Policy.CenterAt(line,o1.z)-bias)/scale;
        e.motion=(o1.fitVx-slope*o1.fitVz)/scale;
        e.along=(o1.fitVz+slope*o1.fitVx)/scale;
        e.fitNoise=(o1.noiseX+Mathf.Abs(slope)*o1.noiseZ)/scale;e.rangeNoise=(o1.noiseZ+Mathf.Abs(slope)*o1.noiseX)/scale;
        if(o1.history<.59f || o1.age>.21f || o1.coherence<.055f || e.fitNoise>.24f || e.rangeNoise>.65f){e.state="motion_unconfirmed";return false;}
        if(e.cross*e.motion>=0 || Mathf.Abs(e.motion)<.32f || Mathf.Abs(e.motion)>1.65f || Mathf.Abs(e.along)>.8f){e.state="no_approaching_motion";return false;}
        if(o1.z<1.4f || o1.z>10.5f || Mathf.Abs(slope)>.25f || Mathf.Abs(yaw)>.28f || Mathf.Abs(offset-bias)>.75f ||
           Mathf.Abs(Policy.CenterAt(line,0)+bias)>.65f || boost<speed+.48f){e.state="outside_envelope";return false;}
        // Include cone radius, front (+.28) and rear (-.88) of the rover. Include
        // speed-command lag and short-range uphill drift of an observed target.
        float dist=o1.z*scale,subject=Mathf.Max(.1f,Mathf.Min(speed,command+.35f));
        e.frontFast=Arrive(dist-.62f-e.rangeNoise,subject,boost,e.along-.12f);
        e.rearFast=Arrive(dist+1.22f+e.rangeNoise,subject,boost,e.along+.12f);
        e.frontPace=Arrive(dist-.62f-e.rangeNoise,subject,pace,e.along-.12f);
        e.rearPace=Arrive(dist+1.22f+e.rangeNoise,subject,pace,e.along+.12f);
        e.rawFast=Clearance(e.cross,e.motion,e.frontFast,e.rearFast,.38f+.34f);
        e.rawPace=Clearance(e.cross,e.motion,e.frontPace,e.rearPace,.38f+.34f);
        e.marginFast=e.rawFast-(.12f+e.fitNoise+.10f*e.rearFast+.055f*e.rearFast*e.rearFast);
        e.marginPace=e.rawPace-(.12f+e.fitNoise+.10f*e.rearPace+.055f*e.rearPace*e.rearPace);
        if(e.rearFast>2.8f || e.marginFast<.12f || e.marginPace>.0f || e.rearPace-e.rearFast<.32f){e.state="timing_rejected";return false;}
        e.offered=true;e.state=enabled?"accepted":"disabled";
        if(!enabled)return false;
        passing=o1.id;remaining=dist+1.22f+Mathf.Max(0,e.along+.12f)*e.rearFast+.22f;
        e.remaining=remaining;return true;
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
    float passGoal;
    bool inPass;
    // Search the currently visible cross section for a corridor wide enough for the body.
    // Clearance is measured along the observed road normal; short-term memory keeps a passing side during occlusion.
    void ChooseCorridor(float[] line,List<Sight> objects,float speed,float lineBias,float period){
        var blocked=new List<Vector2>();
        float localSlope=(line[1]-line[0])/(Ahead[1]-Ahead[0]);
        float egoOffset=-CenterAt(line,0)/Mathf.Sqrt(1+localSlope*localSlope);
        float arrivalCap=cruise;
        float hazard=99;
        foreach(var o in objects){
            if(o.z<-.85f || o.z>12f)continue;
            float slope=(CenterAt(line,o.z+.25f)-CenterAt(line,o.z-.25f))*2f;
            float scale=Mathf.Sqrt(1+slope*slope);
            float cross=(o.x-CenterAt(line,o.z))/scale;
            float motion=Mathf.Clamp((o.vx-slope*o.vz)/scale,-1.6f,1.6f);
            float horizon=Mathf.Clamp(o.z/Mathf.Max(1f,speed),0,2.5f);
            float future=cross+motion*horizon*p[6];
            float padding=.76f+p[4]+Mathf.Min(.4f,.22f*horizon*Mathf.Abs(motion));
            var interval=new Vector2(Mathf.Min(cross,future)-padding,Mathf.Max(cross,future)+padding);
            if(interval.x>1.35f || interval.y< -4.65f)continue;
            blocked.Add(interval);
            if(interval.x<lineBias+.36f && interval.y>lineBias-.36f || inPass)hazard=Mathf.Min(hazard,o.z);
        }
        if(hazard<12f){
            float best=float.NegativeInfinity,bestTarget=lineBias;
            for(int i=0;i<=33;i++){
                float x=-4.30f+.16f*i;
                float margin=Mathf.Min(Track.Lane*.5f-.40f-x,Track.Lane*1.5f-.40f+x);
                foreach(var b in blocked)margin=Mathf.Min(margin,Mathf.Max(b.x-x,x-b.y));
                float score=2f*Mathf.Min(margin,1.25f)-.10f*Mathf.Abs(x-lineBias);
                if(inPass)score-=p[12]*Mathf.Abs(x-passGoal);
                if(score>best){best=score;bestTarget=x;}
            }
            passGoal=bestTarget;inPass=true;recovery=1.2f;
            guardCap=Mathf.Sqrt(p[9]*p[9]+2f*1.25f*Mathf.Max(0,hazard-7f));
            if(hazard>0 && Mathf.Abs(passGoal-egoOffset)>.45f){
                bool exposed=blocked.Any(b=>egoOffset>=b.x && egoOffset<=b.y);
                if(exposed){
                    float needed=Mathf.Sqrt(2f*Mathf.Abs(passGoal-egoOffset)/1.20f)+.35f;
                    arrivalCap=Mathf.Max(.38f,(hazard-.65f)/needed);
                }
            }
            guardCap=Mathf.Min(guardCap,arrivalCap);preferred=passGoal;reason="yield_corridor";
        }else if(inPass && recovery>0){
            preferred=passGoal;guardCap=p[10];reason="hold_pass";
        }else{
            inPass=false;preferred=lineBias;
            bool returning=Mathf.Abs(egoOffset-lineBias)>.60f || Mathf.Abs(offset-lineBias)>.35f || Mathf.Abs(localSlope)>.35f;
            guardCap=returning?p[10]:cruise;reason=returning?"recover":"road";
        }
        conflictRange=hazard;
    }
    readonly LocalTiming timing;
    readonly float approach;
    public TimingEvidence Timing {get{return timing==null?null:timing.evidence;}}
    public Policy(float[] weights,string timingMode="",float approachCap=0){
        p=weights;approach=approachCap;if(!string.IsNullOrEmpty(timingMode))timing=new LocalTiming(timingMode=="both");
    }
    static float Interpolate(float[] line,float s){
        if(s<=Ahead[0])return line[0];
        for(int i=1;i<Ahead.Length;i++)if(s<=Ahead[i])return Mathf.Lerp(line[i-1],line[i],(s-Ahead[i-1])/(Ahead[i]-Ahead[i-1]));
        return line[line.Length-1];
    }
    internal static float CenterAt(float[] line,float s){
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
        float nominalCruise=cruise;
        if(approach>0)cruise=Mathf.Min(cruise,approach);
        float planningSpeed=Mathf.Min(cruise,guardCap);
        if(p.Length>=13){
            ChooseCorridor(line,objects,speed,lineBias,period);
            planningSpeed=Mathf.Min(cruise,guardCap);
        }else if(objects.Count>0){
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
        if(timing!=null && timing.Act(line,objects,speed,speedCmd,nominalCruise,planningSpeed*pace,lineBias,offset,yaw,period)){
            reason="go_first";preferred=lineBias;guardCap=nominalCruise;planningSpeed=nominalCruise;pace=1;recovery=0;
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
