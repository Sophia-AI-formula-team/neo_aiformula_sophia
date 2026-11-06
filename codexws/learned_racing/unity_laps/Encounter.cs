using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace LapLab {
// One row per paired training context. Performance entries come from actual
// rollouts; rows/scales/width are frozen by leave-one-context-out evaluation.
public sealed class EncounterConfig {
    public string mode, modelId;
    public int forced;
    public float[][] actions, contexts, returns;
    public float[] scales;
    public float width;
}
public sealed class EncounterEvidence {
    public string state,modelId;
    public int chosen;
    public float chosenSpeed,chosenOffset,remaining;
    public float[] features,actionScores;
}
public sealed class EncounterPolicy {
    readonly EncounterConfig cfg;
    readonly HashSet<int> used=new HashSet<int>();
    public EncounterEvidence evidence;
    float remaining;
    int choice;
    public EncounterPolicy(EncounterConfig cfg){this.cfg=cfg;}
    float[] Features(float[] line,List<Sight> objs,Sight nearest,float speed,float yaw,float curvature,float offset){
        float slope=(Policy.CenterAt(line,nearest.z+.3f)-Policy.CenterAt(line,nearest.z-.3f))/.6f;
        float scale=Mathf.Sqrt(1+slope*slope);
        float cross=(nearest.x-Policy.CenterAt(line,nearest.z))/scale;
        float motion=(nearest.fitVx-slope*nearest.fitVz)/scale;
        var extents=objs.Select(o=>(o.x-Policy.CenterAt(line,o.z))/scale);
        return new[]{cross,motion,cross+motion*nearest.z/Mathf.Max(.5f,speed),nearest.z,
            (float)objs.Count,extents.Min(),extents.Max(),curvature,speed,yaw,offset};
    }
    int Score(float[] features,out float[] scores){
        scores=new float[cfg.actions.Length];
        if(cfg.mode=="probe")return cfg.forced;
        float[] d2=new float[cfg.contexts.Length];
        for(int i=0;i<d2.Length;i++){
            for(int j=0;j<features.Length;j++){
                float dx=(features[j]-cfg.contexts[i][j])/cfg.scales[j];d2[i]+=dx*dx;
            }
        }
        // Subtract the nearest distance so far-out observations cannot underflow
        // every kernel weight. Action zero reuses the complete frozen controller.
        float nearest=d2.Min(),sum=0;
        for(int i=0;i<d2.Length;i++){
            float w=Mathf.Exp(-(d2[i]-nearest)/(2f*cfg.width*cfg.width));sum+=w;
            for(int a=0;a<scores.Length;a++)scores[a]+=w*cfg.returns[i][a];
        }
        int best=0;
        for(int a=0;a<scores.Length;a++){scores[a]/=sum;if(scores[a]>scores[best])best=a;}
        return best;
    }
    public bool Act(float[] line,List<Sight> tracks,float speed,float yaw,float curvature,float offset,float period){
        evidence=new EncounterEvidence{state="observing",chosen=-1,modelId=cfg.modelId};
        if(remaining>0){
            remaining-=Mathf.Max(0,speed)*period;
            if(remaining<=0){evidence.state="clear";return false;}
            // Yield control to the usual multi-object avoidance if another track
            // enters from outside the observed group after committing.
            if(tracks.Any(o=>!used.Contains(o.id) && o.z>0 && o.z<remaining)){
                remaining=0;evidence.state="new_object";return false;
            }
            evidence.state="holding";
        }else{
            var objects=tracks.Where(o=>o.z>0).OrderBy(o=>o.z).ToList();
            if(objects.Count==0 || used.Contains(objects[0].id))return false;
            var n=objects[0];
            // The decision window is the farthest measured lane point. Wait for
            // four camera samples for lateral velocity, with a close-range escape.
            if(n.z>Policy.Ahead[3] || (n.history<.59f && n.z>Policy.Ahead[0]))return false;
            evidence.features=Features(line,objects,n,speed,yaw,curvature,offset);
            choice=Score(evidence.features,out evidence.actionScores);
            foreach(var o in objects)used.Add(o.id);
            remaining=objects.Max(o=>o.z)+.88f+.34f;
            evidence.state="selected";
        }
        evidence.chosen=choice;evidence.remaining=remaining;
        if(choice==0)return false;
        evidence.chosenOffset=cfg.actions[choice][0];evidence.chosenSpeed=cfg.actions[choice][1];
        return true;
    }
}
}
