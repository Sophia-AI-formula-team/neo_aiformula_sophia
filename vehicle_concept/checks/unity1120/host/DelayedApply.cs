using System;
using System.IO;
using AIFormula;
using Newtonsoft.Json;
using UnityEngine;

namespace LapLab {
public sealed partial class FoxyExchange {
    Action commitPending;
    int dueTick, appliedSeq=-1, lastPhysicsTick=-1;
    StreamWriter ticks;

    void Arm(int seq, float imageTime, Action apply, int delayTicks) {
        if(commitPending!=null)throw new InvalidOperationException("Previous request not yet applied");
        dueTick=Mathf.RoundToInt(imageTime*100f)+delayTicks;
        commitPending=()=>{apply();appliedSeq=seq;};
        File.AppendAllText(Path.Combine(directory,"schedule.jsonl"),JsonConvert.SerializeObject(new {
            kind="accepted",seq,image_simulation_s=imageTime,apply_due_tick=dueTick
        })+"\n");
    }

    public void DelayedStep(AIFormulaDifferentialDrive drive, int tick) {
        if(commitPending!=null && tick==dueTick){commitPending();commitPending=null;}
        Apply(drive);
        if(ticks==null)ticks=new StreamWriter(Path.Combine(directory,"actuation.jsonl"));
        ticks.WriteLine(JsonConvert.SerializeObject(new {tick,sim_s=tick*.01,applied_seq=appliedSeq,
            right_can_rpm=drive.MotorCommand.rightCan,left_can_rpm=drive.MotorCommand.leftCan,
            left_rpm=drive.LeftWheelRpm,right_rpm=drive.RightWheelRpm,
            measured_forward_mps=Vector3.Dot(drive.Body.velocity,drive.Body.rotation*Vector3.forward),
            measured_gyro_ros_radps=-drive.Body.angularVelocity.y}));
        lastPhysicsTick=tick;
    }

    void EndDelayedRun() {
        if(ticks!=null)ticks.Dispose();
        File.AppendAllText(Path.Combine(directory,"schedule.jsonl"),JsonConvert.SerializeObject(new {
            kind="immediate_cleanup",tick=lastPhysicsTick+1,cancelled_request=commitPending!=null
        })+"\n");
        commitPending=null;
    }
}
}
