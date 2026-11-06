using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
namespace LapLab {

public sealed class SmoothConfig {
    public string mode,modelId;
    public int forced;
    public float[][] actions;
    // Each row is either [feature, split, left, right] or [-1, action scores...].
    public float[][][] forest;
    public GapConfig gap;
}
public sealed class SmoothEvidence {
    public string state,modelId;
    public int chosen=-1;
    public float targetOffset,targetSpeed,remaining,observedGoal;
    public float[] features,actionScores;
}
public sealed class SmoothEncounter {
    readonly SmoothConfig cfg;
    readonly HashSet<int> consumed=new HashSet<int>();
    int choice;
    bool active;
    float toRear,goal,measuredGoal;
    public SmoothEvidence evidence;
    const float BodyCone=.38f+.34f;
    const float RearAndCone=.88f+.34f;
    public SmoothEncounter(SmoothConfig cfg){this.cfg=cfg;}
    static float Slope(float[] l,float z){return (Policy.CenterAt(l,z+.3f)-Policy.CenterAt(l,z-.3f))/.6f;}
    static float Cross(float[] line,Sight s){
        float g=Slope(line,s.z);return (s.x-Policy.CenterAt(line,s.z))/Mathf.Sqrt(1+g*g);
    }
    static float Motion(float[] line,Sight s){
        float g=Slope(line,s.z);return (s.fitVx-g*s.fitVz)/Mathf.Sqrt(1+g*g);
    }
    static float[] Features(float[] l,List<Sight> obs,float speed,float yaw,float curvature,float offset){
        var n=obs[0];float x=Cross(l,n),v=Motion(l,n);var xs=obs.Select(o=>Cross(l,o)).ToArray();
        var last=obs[obs.Count-1];
        return new[]{x,v,n.z,x+v*n.z/Mathf.Max(.5f,speed),n.history,n.noiseX,
            (float)obs.Count,xs.Min(),xs.Max(),last.z-n.z,Cross(l,last),Motion(l,last),curvature,
            Slope(l,Policy.Ahead[0]),Slope(l,Policy.Ahead[3]),speed,yaw,offset};
    }
    int Choose(float[] features,out float[] scores){
        scores=new float[cfg.actions.Length];
        if(cfg.mode=="probe")return cfg.forced;
        foreach(var tree in cfg.forest){
            int idx=0;var node=tree[idx];
            while(node[0]>=0){
                idx=(int)(features[(int)node[0]]<=node[1]?node[2]:node[3]);node=tree[idx];
            }
            for(int j=0;j<scores.Length;j++)scores[j]+=node[j+1];
        }
        int best=0;for(int j=0;j<scores.Length;j++){scores[j]/=cfg.forest.Length;if(scores[j]>scores[best])best=j;}
        return best;
    }
    float Lateral(float[] l,List<Sight> near,float speed,float cruise,float bias){
        float[] action=cfg.actions[choice];float sign=action[0],padding=BodyCone+action[1];
        float target=bias;
        if(sign==0)return target;
        foreach(var s in near){
            float x=Cross(l,s),v=Motion(l,s);
            // Include longitudinal occupancy of the whole car. Re-evaluate from
            // fresh A4 observations rather than assuming a known periodic phase.
            float arrival=Mathf.Clamp((s.z-.62f)/Mathf.Max(1f,.5f*(speed+cruise)),0,2.5f);
            float departure=Mathf.Clamp((s.z+RearAndCone)/Mathf.Max(1f,.5f*(speed+cruise)),arrival,3.2f);
            float a=x+v*arrival,b=x+v*departure;
            float low=Mathf.Min(a,b),high=Mathf.Max(a,b);
            if(low>bias+padding || high<bias-padding)continue;
            target=sign<0?Mathf.Min(target,low-padding):Mathf.Max(target,high+padding);
        }
        return Mathf.Clamp(target,-1.5f*Track.Lane+.38f,.5f*Track.Lane-.38f);
    }
    public bool Act(float[] line,List<Sight> tracks,float speed,float yaw,float curvature,float offset,float bias,float dt){
        evidence=new SmoothEvidence{state="clear",modelId=cfg.modelId};
        var front=tracks.Where(o=>o.z>-.9f && o.z<=12.5f).OrderBy(o=>o.z).ToList();
        if(!active){
            var obs=front.Where(o=>o.z>0f).ToList();
            if(obs.Count==0 || obs[0].z>Policy.Ahead[3] || consumed.Contains(obs[0].id))return false;
            if(obs[0].history<.59f && obs[0].z>Policy.Ahead[0])return false;
            evidence.features=Features(line,obs,speed,yaw,curvature,offset);
            choice=Choose(evidence.features,out evidence.actionScores);goal=offset;
            toRear=obs.Max(o=>o.z)+RearAndCone;
            foreach(var o in obs)consumed.Add(o.id);
            active=true;evidence.state="selected";
        }else{
            toRear-=speed*dt;evidence.state="holding";
            // Assimilate a close group without restarting the passing side when
            // YOLO reacquires the same cone or sees its neighbour.
            foreach(var o in front)if(!consumed.Contains(o.id) && o.z>0f && o.z<=Mathf.Max(2f,toRear+2.5f)){
                consumed.Add(o.id);toRear=Mathf.Max(toRear,o.z+RearAndCone);
            }
            if(toRear < -Track.Lane){active=false;evidence.state="released";return false;}
        }
        evidence.chosen=choice;evidence.remaining=toRear;
        if(choice==0)return false;
        var action=cfg.actions[choice];var near=front.Where(o=>consumed.Contains(o.id)).ToList();
        if(near.Count>0 && toRear>0)measuredGoal=Lateral(line,near,speed,action[2],bias);
        if(toRear<=0){
            evidence.state="returning";
            float progress=Mathf.Clamp01(-toRear/Track.Lane);
            measuredGoal=Mathf.Lerp(goal,bias,progress*progress*(3f-2f*progress));
        }
        // Bound only the lateral obstacle correction. The road-following yaw
        // still responds immediately to road curvature and current body motion.
        goal=Mathf.Lerp(goal,measuredGoal,1f-Mathf.Exp(-dt/.30f));
        evidence.observedGoal=measuredGoal;evidence.targetOffset=goal;evidence.targetSpeed=action[2];
        return true;
    }
}
}
