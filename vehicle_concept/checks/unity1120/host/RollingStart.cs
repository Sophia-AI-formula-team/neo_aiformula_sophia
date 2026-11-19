using System;
using System.Collections.Generic;
using System.IO;
using Newtonsoft.Json;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace LapLab {
public sealed partial class UnityRace {
    // Test fixture only. Open-loop straight-wheel command from the recorded
    // Honda conversion can_command(1.6, 0, physical=True); no steering policy.
    void RollingStart() {
        if(props.Count!=0)throw new InvalidOperationException("Rolling start fixture expects a clear course");
        var log=new List<object>(); float distance=0;
        var previousPosition=d.Body.position;
        var directory=Path.GetDirectoryName(cfg.output);
        for(int tick=0;tick<=100;tick++){
            bool handoff=tick==100;
            d.SetRecordedCanCommand(handoff?0:188,handoff?0:186,false);
            var pos=d.Body.position;
            distance+=Vector3.Distance(pos,previousPosition);previousPosition=pos;
            var axle=Track.XZ(d.Body.rotation*Vector3.right).normalized;
            bool leftOut=trackLimits.Out(Track.XZ(d.frontLeft.transform.position),axle,frontTreadWidths[0]);
            bool rightOut=trackLimits.Out(Track.XZ(d.frontRight.transform.position),axle,frontTreadWidths[1]);
            var v=Vector3.Dot(d.Body.velocity,d.Body.rotation*Vector3.forward);
            log.Add(new{tick,pre_roll_s=tick*.01,distance_m=distance,body_forward_mps=v,
                leftRpm=d.LeftWheelRpm,rightRpm=d.RightWheelRpm,gyro_ros_radps=-d.Body.angularVelocity.y,
                right_can_rpm=d.MotorCommand.rightCan,left_can_rpm=d.MotorCommand.leftCan,
                leftOut,rightOut,x=pos.x,z=pos.z});
            if(leftOut||rightOut){
                File.WriteAllText(Path.Combine(directory,"pre_roll.json"),JsonConvert.SerializeObject(log));
                throw new InvalidOperationException("Rolling start crossed the painted course before handoff");
            }
            if(handoff)break;
            d.StepDrive(dt);scene.GetPhysicsScene().Simulate(dt);
        }
        File.WriteAllText(Path.Combine(directory,"pre_roll.json"),JsonConvert.SerializeObject(log));
        near=track.Nearest(Track.XZ(d.Body.position),near);
        previous=startStation=track.Station(Track.XZ(d.Body.position),near);
        // The genuine latch is zero for the first observation. No physics tick
        // elapses between clearing it and camera capture. Wheel speed is retained.
    }
}
}
