using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using System.Reflection;
using System.Text;
using AIFormula;
using Newtonsoft.Json;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace LapLab {
public class Setup {
    public string output="", media="";
    public float[][] edge, obstacles;
    public float[] weights;
    public float start,offset,yaw, limit=1f, captureInterval=.5f;
    public int visionPort;
    public bool externalControl;
    public string departureRule="body_edge";
    public string timingMode="";
    public EncounterConfig encounter;
    public SmoothConfig smooth;
    public float[] laneGateThresholdsM;
    public float approachCap;
}
public class Perception {public float?[] lane;public float[][] objects;public float[] control;public object decision;public string error;}
public class Prop {public GameObject go;public Vector2 home,right;public float amplitude,phase,hz,progress;}
public sealed class UnityRace {
    const float dt=.01f;
    const string ScenePath="Assets/Scenes/MotorControllerValidation.unity";
    readonly Setup cfg;Scene scene;AIFormulaDifferentialDrive d; Track track;Policy policy;
    AIFormulaRgbCameraTopic feed;Camera chase;AIFormulaPhotoVisualRig photo;
    TcpClient net;BinaryReader netIn;BinaryWriter netOut;
    readonly ObjectTracker tracker=new ObjectTracker();
    readonly OdometryLaneGate laneGate;
    readonly List<Prop> props=new List<Prop>();readonly List<float[]> rows=new List<float[]>(), frames=new List<float[]>();
    readonly List<object> decisions=new List<object>();
    readonly List<float[]> wheelRows=new List<float[]>();
    float firstBodyEdge=-1;
    int k,near,gate,seenFrames,empty,coneseens,closestProp=-1;float startStation,previous,progress,peakProgress,lastAdvance;
    float minRoad=99,minGap=99,localGap=99;float[] zoneStart,zoneEnd,propMin;
    bool done; string status="running";float speed,yaw;int captures;float lastCapture=-99,lastLaneOffset;
    public static string Schedule(string config){
        if(EditorApplication.isPlayingOrWillChangePlaymode || EditorApplication.isCompiling || SceneManager.GetActiveScene().isDirty)
            throw new InvalidOperationException("A clean, idle existing Unity editor is required.");
        var job=new UnityRace(JsonConvert.DeserializeObject<Setup>(File.ReadAllText(config)));
        try{job.Open();EditorApplication.update+=job.Batch;AssemblyReloadEvents.beforeAssemblyReload+=job.Interrupt;}
        catch{job.Dispose();throw;}return "scheduled";
    }
    UnityRace(Setup config){
        cfg=config;
        if(config.laneGateThresholdsM!=null)laneGate=new OdometryLaneGate(Policy.Ahead,config.laneGateThresholdsM);
    }
    void Open(){
        if(cfg.departureRule!="body_edge" && cfg.departureRule!="all_wheel_centers")throw new InvalidOperationException("Unknown departure rule");
        var open=typeof(EditorSceneManager).GetMethod("OpenPreviewScene",BindingFlags.NonPublic|BindingFlags.Static,null,new[]{typeof(string)},null);
        scene=(Scene)open.Invoke(null,new object[]{ScenePath});var ph=scene.GetPhysicsScene();
        if(!ph.IsValid()||ph.Equals(Physics.defaultPhysicsScene))throw new InvalidOperationException("Unisolated physics scene");
        var roots=scene.GetRootGameObjects();
        d=roots.SelectMany(o=>o.GetComponentsInChildren<AIFormulaDifferentialDrive>(true)).Single();
        if(!d.UsesRepositoryMotor||!d.TireContact.enabled||!d.TireContact.coupledRearContact||
           d.motorPlant.response.ToString()!="JanuaryLapIdentified"||d.EditorExperimentalStep!=null)throw new InvalidOperationException("Different installed plant");
        track=new Track(cfg.edge,roots.Single(o=>o.name=="Renderer").transform.Find("Codex/LitRenderer/RebuiltTrackSurface"));
        near=track.AtFraction(cfg.start);var f=track.dir[near];Vector2 p=track.lanes[near]+track.right[near]*cfg.offset;
        var root=d.transform;var orig=root.position;var origRot=root.rotation;
        var rot=Quaternion.AngleAxis(cfg.yaw,Vector3.up)*Quaternion.LookRotation(Track.World(f),Vector3.up);
        var spawn=Track.World(p,orig.y);var delta=rot*Quaternion.Inverse(origRot);
        var bodies=d.GetComponentsInChildren<Rigidbody>();var positions=bodies.Select(b=>spawn+delta*(b.position-orig)).ToArray();
        var rotations=bodies.Select(b=>delta*b.rotation).ToArray();
        root.SetPositionAndRotation(spawn,rot);for(int i=0;i<bodies.Length;i++){bodies[i].position=positions[i];bodies[i].rotation=rotations[i];}
        Physics.SyncTransforms();d.Initialize();d.SetInput(0,0,true);
        for(int i=0;i<200;i++){d.StepDrive(dt);ph.Simulate(dt);}
        if(!d.UsesImplicitContact||d.TireContact.Contacts<2)throw new InvalidOperationException("No ground contact at spawn");
        policy=new Policy(cfg.weights,cfg.timingMode,cfg.approachCap,cfg.encounter,cfg.smooth);previous=startStation=track.Station(Track.XZ(d.Body.position),near);
        foreach(var spec in cfg.obstacles){
            int idx=track.AtFraction(cfg.start+spec[0]);
            var cone=(GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>("Assets/TrainingEnvironment/Prefabs/TrafficCone_75cm.prefab"),scene);
            Vector2 at=track.lanes[idx]+track.right[idx]*spec[1];
            var prop=new Prop{go=cone,home=at,right=track.right[idx],amplitude=spec[2],hz=spec[3],phase=spec[4],
                progress=Mathf.Repeat(track.stations[idx]-startStation,track.length)};
            cone.transform.SetPositionAndRotation(Track.World(at),Quaternion.LookRotation(Track.World(track.dir[idx]),Vector3.up));
            props.Add(prop);
        }
        zoneStart=props.Select(o=>-1f).ToArray();zoneEnd=(float[])zoneStart.Clone();propMin=props.Select(o=>99f).ToArray();
        Physics.SyncTransforms();
        feed=d.GetComponent<AIFormulaRgbCameraTopic>();photo=d.GetComponentInChildren<AIFormulaPhotoVisualRig>(true);
        if(cfg.visionPort>0 || cfg.media.Length>0){feed.rgbCamera.scene=scene;}
        if(cfg.visionPort>0){
            net=new TcpClient();net.ReceiveTimeout=20000;net.SendTimeout=20000;net.NoDelay=true;net.Connect("127.0.0.1",cfg.visionPort);
            netIn=new BinaryReader(net.GetStream());netOut=new BinaryWriter(net.GetStream());
        }
        if(cfg.media.Length>0){
            Directory.CreateDirectory(cfg.media);var obj=EditorUtility.CreateGameObjectWithHideFlags("Lap replay camera",HideFlags.HideAndDontSave,typeof(Camera));
            SceneManager.MoveGameObjectToScene(obj,scene);chase=obj.GetComponent<Camera>();chase.scene=scene;chase.enabled=false;
            chase.cullingMask=~((1<<LayerMask.NameToLayer("UnlitMask"))|(1<<LayerMask.NameToLayer("UI")));chase.fieldOfView=58;chase.nearClipPlane=.09f;chase.farClipPlane=160;
            chase.clearFlags=feed.rgbCamera.clearFlags;chase.backgroundColor=feed.rgbCamera.backgroundColor;
        }
    }
    void Interrupt(){if(!done)Finish("editor_reload");}
    void Batch(){
        try{for(int j=0;j<(net==null?360:24)&&!done;j++)Tick();}
        catch(Exception ex){Finish("error: "+ex.Message+" "+ex.StackTrace);}
    }
    Vector2 PropPosition(Prop o,float t){return o.home+o.right*(o.amplitude*Mathf.Sin(o.phase+t*o.hz));}
    static float[] Floats(Perception v){return v.lane.Select(n=>n.HasValue?n.Value:float.NaN).ToArray();}
    Perception Detect(){
        if(cfg.externalControl){
            // Only virtual wheel/gyro sensor channels cross this boundary.
            // Implicit-contact RPM is a motor-plant proxy, not an independent encoder.
            var sensor=Encoding.UTF8.GetBytes(JsonConvert.SerializeObject(new {
                t=k*dt,leftRpm=d.LeftWheelRpm,rightRpm=d.RightWheelRpm,gyro=d.Body.angularVelocity.y,
                wheelRadius=.127f,source="Unity motor-plant RPM proxy and ideal raw gyro"}));
            netOut.Write(sensor.Length);netOut.Write(sensor);
        }
        var bytes=feed.Capture().EncodeToPNG();netOut.Write(bytes.Length);netOut.Write(bytes);netOut.Flush();
        int count=netIn.ReadInt32();if(count<2||count>1000000)throw new IOException("Invalid detector packet");
        var obs=JsonConvert.DeserializeObject<Perception>(Encoding.UTF8.GetString(netIn.ReadBytes(count)));
        if(!string.IsNullOrEmpty(obs.error))throw new IOException(obs.error);return obs;
    }
    void Tick(){
        float t=k*dt;var b=d.Body;Vector2 pos=Track.XZ(b.position),forward=Track.XZ(b.rotation*Vector3.forward).normalized;
        var right=new Vector2(forward.y,-forward.x);speed=Vector3.Dot(b.velocity,b.rotation*Vector3.forward);yaw=b.angularVelocity.y;
        near=track.Nearest(pos,near);float station=track.Station(pos,near);
        float delta=station-previous;if(delta>track.length/2)delta-=track.length;if(delta< -track.length/2)delta+=track.length;
        if(Mathf.Abs(delta)>1.6f && k>0){Finish("invalid_progress");return;}
        progress+=delta;previous=station;
        if(progress>peakProgress+.15f){peakProgress=progress;lastAdvance=t;}
        if(progress>=(gate+1)*track.length/32f && gate<32)gate++;
        localGap=99;closestProp=-1;
        for(int i=0;i<props.Count;i++){
            var o=props[i];var op=PropPosition(o,t);if(o.amplitude!=0 || k==0)o.go.transform.position=Track.World(op);
            var v=op-pos;float zs=Vector2.Dot(v,forward),xx=Vector2.Dot(v,right);
            float longitudinal=Mathf.Max(-.88f-zs,zs-.28f,0),lat=Mathf.Max(Mathf.Abs(xx)-.38f,0);
            float gap=Mathf.Sqrt(longitudinal*longitudinal+lat*lat)-.34f;
            if(gap<localGap){localGap=gap;closestProp=i;}propMin[i]=Mathf.Min(propMin[i],gap);
            if(progress>=o.progress-6&&zoneStart[i]<0)zoneStart[i]=t;
            if(progress>=o.progress+6&&zoneEnd[i]<0)zoneEnd[i]=t;
        }
        if(props.Count>0)Physics.SyncTransforms();
        minGap=Mathf.Min(minGap,localGap);
        float off=track.Offset(pos,near);lastLaneOffset=off;
        float road=99;
        for(int j=0;j<4;j++){
            var p=pos+forward*(j<2?-.88f:.28f)+right*((j%2)==0?-.38f:.38f);
            int nn=track.Nearest(p,near);float x=track.Offset(p,nn);
            road=Mathf.Min(road,Mathf.Min(Track.Lane*.5f-x,Track.Lane*1.5f+x));
        }
        minRoad=Mathf.Min(minRoad,road);
        if(road<0 && firstBodyEdge<0)firstBodyEdge=t;
        // Referee only. This installed Honda model has TWO front driven wheels
        // and ONE rear caster. Do not claim a four-wheel model or feed positions
        // to the controller. Tire contact-patch width is not represented here.
        var wheelPositions=new[]{d.frontLeft.transform.position,d.frontRight.transform.position,d.rearWheelBody.position};
        var wheelGaps=wheelPositions.Select(wp=>{var p=Track.XZ(wp);int nn=track.Nearest(p,near);float x=track.Offset(p,nn);return Mathf.Min(Track.Lane*.5f-x,Track.Lane*1.5f+x);}).ToArray();
        bool departure=cfg.departureRule=="all_wheel_centers" ? wheelGaps.All(g=>g<0) : road<0;
        if(k%10==0 || localGap<=0 || departure || progress>=track.length*cfg.limit){
            rows.Add(new[]{t,pos.x,pos.y,Mathf.Atan2(forward.x,forward.y),speed,policy.speedCmd,policy.yawCmd,progress,off,road,localGap,policy.offset,(float)d.MotorCommand.rightCan,(float)d.MotorCommand.leftCan,yaw});
            wheelRows.Add(new[]{t,wheelGaps[0],wheelGaps[1],wheelGaps[2]});
        }
        if(chase && (t-lastCapture>=cfg.captureInterval-.0005f || k==0 || localGap<=0 || departure)){Capture(t);lastCapture=t;}
        if(localGap<=0){Finish("obstacle");return;}if(departure){Finish("boundary");return;}
        if(progress>=track.length*cfg.limit && (cfg.limit<.999f||gate==32)){Finish(cfg.limit<.999f?"segment":"lap");return;}
        if(t>260||t-lastAdvance>9){Finish("stalled");return;}
        float?[] observed=null,filtered=null;List<Sight> tracks=null;int laneGateMask=0;
        if(k%(net==null?10:20)==0){
            if(cfg.externalControl){
                if(net==null)throw new InvalidOperationException("V3 requires a camera sensor server");
                var obs=Detect();
                seenFrames++;
                if(obs.control==null || obs.control.Length!=2 || obs.control.Any(v=>float.IsNaN(v)||float.IsInfinity(v)))
                    throw new InvalidOperationException("Invalid V3 command");
                policy.speedCmd=Mathf.Clamp(obs.control[0],0,6);policy.yawCmd=Mathf.Clamp(obs.control[1],-1.60119f,1.69546f);
                decisions.Add(new {t,version="V3",sensorSource="wheel proxy + raw gyro + A4/RGB",obs.decision,policy.speedCmd,policy.yawCmd});
            }else{
            List<Vector2> visible=new List<Vector2>();float[] lane;
            if(net!=null){
                var obs=Detect();lane=Floats(obs);foreach(var o in obs.objects)visible.Add(new Vector2(o[0],o[1]));
            } else{
                lane=track.LocalLane(pos,forward,near);
                foreach(var o in props){
                    var diff=PropPosition(o,t)-pos;float z=Vector2.Dot(diff,forward),x=Vector2.Dot(diff,right);
                    if(z>.1f&&diff.sqrMagnitude<=144&&Mathf.Abs(x)<z*1.2f){
                        Vector3 eye=b.position+Vector3.up*.655f,aim=o.go.transform.position+Vector3.up*.58f;RaycastHit hit;
                        if(scene.GetPhysicsScene().Raycast(eye,(aim-eye).normalized,out hit,(aim-eye).magnitude+.18f,~0,QueryTriggerInteraction.Ignore)&&hit.collider.gameObject==o.go)
                            visible.Add(new Vector2(z,x));
                    }
                }
            }
            seenFrames++;if(lane.Count(v=>!float.IsNaN(v))<2)empty++;if(visible.Count>0)coneseens++;
            observed=lane.Select(v=>float.IsNaN(v)?(float?)null:v).ToArray();
            if(laneGate!=null){
                laneGateMask=laneGate.Apply(lane,t,speed,yaw);
                filtered=lane.Select(v=>float.IsNaN(v)?(float?)null:v).ToArray();
            }
            tracks=tracker.Update(visible,pos,forward,t);
            if(!policy.Act(lane,tracks,Mathf.Max(0,speed),yaw,net==null?.10f:.20f)){Finish("lost_lane");return;}
            }
        }
        d.SetAutonomousVelocityCommand(policy.speedCmd,policy.yawCmd);d.StepDrive(dt);scene.GetPhysicsScene().Simulate(dt);k++;
        if(observed!=null){
            var decision=new{t,speed,yaw,observed,tracks=tracks.Select(o=>new[]{o.z,o.x,o.vx,o.vz}).ToArray(),
            policy.reason,policy.curvature,policy.cruise,policy.look,policy.target,policy.preferred,policy.offset,ratio=policy.chosenSpeed,
            policy.guardCap,policy.conflictRange,policy.speedCmd,policy.yawCmd,d.MotorCommand.rightCan,d.MotorCommand.leftCan,timing=policy.Timing,encounter=policy.Encounter,smooth=policy.Smooth,gap=policy.Gap};
            if(laneGate==null)decisions.Add(decision);
            else{
                var record=Newtonsoft.Json.Linq.JObject.FromObject(decision);
                record["laneGate"]=Newtonsoft.Json.Linq.JObject.FromObject(new {mask=laneGateMask,filtered});
                decisions.Add(record);
            }
        }
        if(d.MotorFault.Length>0||d.TireContact.Fault.Length>0)throw new InvalidOperationException(d.MotorFault+d.TireContact.Fault);
    }
    void Capture(float t){
        d.UpdateWheelVisuals();if(photo)photo.ApplyVisualPose();var ax=d.Body.rotation*Vector3.forward;
        var pos=d.Body.position;var cp=pos-ax*3.15f+Vector3.up*1.80f;
        chase.transform.SetPositionAndRotation(cp,Quaternion.LookRotation(pos+ax*1.45f+Vector3.up*.10f-cp,Vector3.up));
        string prefix=Path.Combine(cfg.media,captures.ToString("D4"));
        var prev=RenderTexture.active;var rt=RenderTexture.GetTemporary(640,360,24);var pix=new Texture2D(640,360,TextureFormat.RGB24,false);
        try{chase.targetTexture=rt;chase.Render();RenderTexture.active=rt;pix.ReadPixels(new Rect(0,0,640,360),0,0);pix.Apply();
            File.WriteAllBytes(prefix+"c.jpg",pix.EncodeToJPG(82));File.WriteAllBytes(prefix+"f.jpg",feed.Capture().EncodeToJPG(80));}
        finally{chase.targetTexture=null;RenderTexture.active=prev;RenderTexture.ReleaseTemporary(rt);UnityEngine.Object.DestroyImmediate(pix);}
        frames.Add(new[]{t,(float)captures++});
    }
    void Finish(string outcome){
        if(done)return;done=true;status=outcome;
        try{
            var result=new{status,seconds=k*dt,progress,length=track.length,gate,minRoad,minGap,seenFrames,empty,coneseens,
                propMin,zoneStart,zoneEnd,rows,frames,decisions,wheelRows,firstBodyEdge,departureRule=cfg.departureRule,profile=d.motorPlant.response.ToString(),coupled=d.TireContact.coupledRearContact,
                unity=Application.unityVersion,physicsStep=dt,props=props.Select(o=>new[]{o.home.x,o.home.y,o.right.x,o.right.y,o.amplitude,o.phase,o.hz,o.progress}).ToArray()};
            File.WriteAllText(cfg.output+".tmp",JsonConvert.SerializeObject(result));
            File.Move(cfg.output+".tmp",cfg.output);
        }finally{Dispose();}
    }
    void Dispose(){
        EditorApplication.update-=Batch;AssemblyReloadEvents.beforeAssemblyReload-=Interrupt;
        if(net!=null)net.Close();if(feed!=null)feed.Release();if(scene.IsValid())EditorSceneManager.ClosePreviewScene(scene);
    }
}
}
