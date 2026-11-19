using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using AIFormula;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace LapLab {
// Only this class crosses from the preview physics scene into the driving stack.
public sealed class FoxyExchange {
    readonly string directory;
    int sequence, rightCan, leftCan;
    float lastV, lastW;
    static long UtcUs() { return (DateTime.UtcNow.Ticks - 621355968000000000L) / 10; }
    public FoxyExchange(string resultPath) { directory = Path.GetDirectoryName(resultPath); }

    public void Mount(AIFormulaDifferentialDrive drive, AIFormulaRgbCameraTopic feed) {
        // The simulation's chassis origin sits below road level. Measure the
        // ground under the rig once at spawn, independent of road labels.
        var origin = drive.transform.TransformPoint(new Vector3(-.0600025f, 0, .055f));
        var hits = new RaycastHit[48];
        int count = drive.gameObject.scene.GetPhysicsScene().Raycast(origin + Vector3.up * 2f,
            Vector3.down, hits, 4f, ~0, QueryTriggerInteraction.Ignore);
        var ground = hits.Take(count).Where(x => !x.collider.transform.IsChildOf(drive.transform)
            && x.normal.y > .8f).OrderBy(x => x.distance).ToArray();
        if (count == hits.Length || ground.Length == 0) throw new IOException("No verified ground for camera mount");
        float groundY = ground[0].point.y;
        var optical = new Vector3(origin.x, groundY + .55f, origin.z);
        feed.vehicleLocalPosition = drive.transform.InverseTransformPoint(optical);
        feed.vehicleLocalRotation = Quaternion.Euler(-1f,0,0);
        feed.repositoryIntrinsics = new Vector4(246.0857544f,246.0857544f,328.6564941f,189.0118256f);
        feed.width = 640; feed.height = 360; feed.useRepositoryIntrinsics = true;
        File.WriteAllText(Path.Combine(directory,"mount.json"), JsonConvert.SerializeObject(new {
            pixel_space="Honda rectified left; Honda to canonical warp belongs to Foxy", groundY,
            ground=ground[0].collider.name, chassis_origin_y=drive.transform.position.y,
            optical_ground_m=feed.transform.TransformPoint(feed.vehicleLocalPosition).y-groundY,
            focal_px=feed.repositoryIntrinsics.x, cx=feed.repositoryIntrinsics.z, cy=feed.repositoryIntrinsics.w,
            left_m=.0600025f, forward_m=.055f, pitch_up_deg=1f,
            motor=drive.motorPlant.response.ToString(), velocity_conversion="external Foxy raw CAN",
            unused_scene_velocity_converter=drive.motorController.profile.ToString(),
            wheel_radius_m=drive.wheelRadius, axle_track_m=drive.frontTrack,
            rpm_reference_ramp=drive.rpmReferenceRamp, shaft_to_wheel_ratio=drive.motorToWheelRatio
        },Formatting.Indented));
    }

    public Perception Frame(TcpClient client, AIFormulaRgbCameraTopic feed, AIFormulaDifferentialDrive drive, float ts) {
        var begin = UtcUs();
        byte[] image = feed.Capture().EncodeToJPG(95);
        var sent = UtcUs();
        var header = new {
            schema="concept-unity/1", kind="frame", seq=sequence, simulation_time_s=ts,
            width=feed.width, height=feed.height, encoding="jpeg", image_bytes=image.Length,
            left_rpm=drive.LeftWheelRpm, right_rpm=drive.RightWheelRpm, gyro_ros_radps=-drive.Body.angularVelocity.y,
            wheel_radius_m=drive.wheelRadius, applied_linear_mps=lastV, applied_angular_z_radps=lastW,
            applied_seq=sequence-1, applied_right_can_rpm=drive.MotorCommand.rightCan,
            applied_left_can_rpm=drive.MotorCommand.leftCan
        };
        var stream = client.GetStream();
        byte[] line = Encoding.UTF8.GetBytes(JsonConvert.SerializeObject(header)+"\n");
        stream.Write(line,0,line.Length); stream.Write(image,0,image.Length); stream.Flush();
        var replyBytes = new List<byte>();
        while(replyBytes.Count < 65536) {
            int b=stream.ReadByte(); if(b<0) throw new EndOfStreamException("Foxy closed before a command");
            if(b==10)break; replyBytes.Add((byte)b);
        }
        if(replyBytes.Count==65536)throw new IOException("Foxy reply exceeds 64 KiB");
        var reply=JObject.Parse(Encoding.UTF8.GetString(replyBytes.ToArray()));
        var answer=UtcUs();
        if((string)reply["kind"]!="command" || (int?)reply["seq"]!=sequence)
            throw new IOException("Unexpected or stale Foxy command: "+reply);
        var canR=reply["right_can_rpm"]; var canL=reply["left_can_rpm"];
        if(canR==null || canL==null || canR.Type!=JTokenType.Integer || canL.Type!=JTokenType.Integer)
            throw new IOException("Both original CAN integers are required");
        float v=(float)reply["linear_mps"], w=(float)reply["angular_z_radps"];
        if(float.IsNaN(v)||float.IsInfinity(v)||float.IsNaN(w)||float.IsInfinity(w))
            throw new IOException("Non-finite Foxy command");
        var imageFile=String.Format("camera_{0:D4}.jpg",sequence);
        File.WriteAllBytes(Path.Combine(directory,imageFile),image);
        var previousRight=rightCan; var previousLeft=leftCan;
        rightCan=(int)canR; leftCan=(int)canL;
        Apply(drive);
        var applied=UtcUs();
        var row=new {frame=header, command=reply, camera_file=imageFile, timestamp_unit="Unix microseconds UTC",
            capture_start_utc_us=begin, capture_encoded_utc_us=sent, reply_received_utc_us=answer,
            can_set_utc_us=applied, previous_right_can=previousRight, previous_left_can=previousLeft,
            set_right_can=drive.MotorCommand.rightCan, set_left_can=drive.MotorCommand.leftCan,
            body_forward_mps=Vector3.Dot(drive.Body.velocity,drive.Body.rotation*Vector3.forward)};
        File.AppendAllText(Path.Combine(directory,"exchange.jsonl"),JsonConvert.SerializeObject(row)+"\n");
        sequence++;lastV=v;lastW=w;
        return new Perception{control=new[]{v,-w},decision=new{seq=sequence-1,mode=(string)reply["mode"],rightCan,leftCan}};
    }

    public void Apply(AIFormulaDifferentialDrive drive) {
        // An ordinary zero gate is a zero-RPM request. The scene's artificial
        // wheel-lock brake is only used when settling the spawn, never here.
        drive.SetRecordedCanCommand(rightCan,leftCan,false);
    }

    public void Release(AIFormulaDifferentialDrive drive) {
        rightCan=0; leftCan=0; Apply(drive);
        File.WriteAllText(Path.Combine(directory,"release.json"),JsonConvert.SerializeObject(new {
            seq=sequence, utc_us=UtcUs(), right_can_rpm=drive.MotorCommand.rightCan,
            left_can_rpm=drive.MotorCommand.leftCan, scene=drive.gameObject.scene.name
        }));
    }
}
}
