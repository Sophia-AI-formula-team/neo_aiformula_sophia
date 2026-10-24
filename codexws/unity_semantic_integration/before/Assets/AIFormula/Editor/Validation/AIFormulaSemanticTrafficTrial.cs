using System;
using System.Collections.Generic;
using System.IO;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

namespace AIFormula.Editor
{
    // 位置和灯色真值仅用于试验初始化、评测与录像；不进入语义规划或 LYA。
    [InitializeOnLoad]
    public static class AIFormulaSemanticTrafficTrial
    {
        const string Key = "AIFormula.SemanticTrafficTrial";
        const double CapturePeriod = 1.0 / 15;
        const double MotionPeriod = 1.0 / 20;
        const float SignalPlaneZ = 15;
        static AIFormulaDifferentialDrive drive;
        static AIFormulaVisionLaneFollower follower;
        static AIFormulaRoadDetector detector;
        static AIFormulaSemanticPlanning semantic;
        static TrainingFlagPanel signal;
        static Camera source, capture;
        static RenderTexture target;
        static Texture2D pixels;
        static StreamWriter trace, motion, frames, events;
        static JObject request, lastPerception;
        static string output, overlayPath, rgbPath, finishReason;
        static double startWall, startSim, nextImage, nextMotion, nextStatus;
        static double movementStart, waitStart, greenAt, finishAt, tailWallDeadline;
        static long sequence;
        static int imageCount, detectionCount, motionCount, supportedCount;
        static float travelled, minimumStopZ;
        static Vector3 lastPosition;
        static bool active, finishing, accepted, stoppedForRed, crossedSignal, redViolation, semanticReadySeen;
        static int originalDetectorOutput;
        static float originalDetectorFrequency;
        static bool originalPanelRgb;

        static AIFormulaSemanticTrafficTrial()
        {
            EditorApplication.update += Tick;
            EditorApplication.playModeStateChanged += state => {
                if (state == PlayModeStateChange.ExitingPlayMode && active) {
                    BeginFinish("Play interrupted", false);
                    Complete();
                }
            };
            AssemblyReloadEvents.beforeAssemblyReload += () => {
                if (!active) return;
                BeginFinish("Assembly reload interrupted capture", false);
                Complete();
            };
        }

        public static string Start(string directory, string caseName, float laneX, float startZ,
                                   bool initialRed, bool releaseGreen, float perceptionHz = 5, bool panelRgb = false)
        {
            var scene = SceneManager.GetActiveScene();
            if (Application.isBatchMode || EditorApplication.isPlayingOrWillChangePlaymode || scene.isDirty || active)
                throw new InvalidOperationException("Requires clean GUI Edit Mode without another traffic trial.");
            if (!scene.path.EndsWith("/MotorControllerValidation.unity", StringComparison.Ordinal))
                throw new InvalidOperationException("Open the saved MotorControllerValidation scene first.");
            if (!Finite(laneX) || !Finite(startZ) || string.IsNullOrWhiteSpace(caseName))
                throw new ArgumentException("Finite start coordinates and a case name are required.");
            var car = Object.FindObjectOfType<AIFormulaDifferentialDrive>();
            // DebugPanel.Build 在进入 Play 后安装检测器和 follower；Edit 只核对其基础组件。
            if (!car || !car.GetComponent<AIFormulaDebugPanel>() || !car.GetComponent<AIFormulaRgbCameraTopic>()
                || !car.GetComponent<AIFormulaDriverInput>())
                throw new InvalidOperationException("Motor, debug panel, calibrated RGB topic and driver must already exist.");
            var bridge = car.GetComponent<AIFormulaLaneRuntimeBridge>();
            var demo = car.GetComponent<AIFormulaE2EDemo>();
            if ((bridge && bridge.isActiveAndEnabled) || (demo && demo.isActiveAndEnabled))
                throw new InvalidOperationException("Another autonomous mode owns this scene.");
            if (car.GetComponentsInChildren<Rigidbody>(true).Length != 3)
                throw new InvalidOperationException("Expected the three articulated vehicle bodies.");
            directory = Path.GetFullPath(directory);
            if (Directory.Exists(directory)) throw new InvalidOperationException("Trial directory already exists: " + directory);
            Directory.CreateDirectory(directory);
            var config = new JObject {
                ["directory"] = directory, ["case"] = caseName, ["lane_x"] = laneX, ["start_z"] = startZ,
                ["initial_red"] = initialRed, ["release_green"] = releaseGreen,
                ["perception_hz"] = perceptionHz,
                ["signal_panel_rgb"] = panelRgb,
                ["scene"] = scene.path, ["requested_utc"] = DateTime.UtcNow.ToString("o"),
                ["requested_editor_wall"] = EditorApplication.timeSinceStartup,
                ["signal_plane_z"] = SignalPlaneZ,
                ["finish_region"] = "x <= 38 m and z >= 35 m after signal pass; full northeast bend and north straight inside the requested eastern region",
                ["reference_role"] = "Start pose and signal truth initialize/evaluate the trial. No evaluator route or signal truth enters control. Unity Rigidbody pose supplies ideal noiseless simulated odometry to MapAssist.",
                ["baseline"] = Configuration(car)
            };
            File.WriteAllText(Path.Combine(directory, "configuration.json"), config.ToString(Formatting.Indented));
            SessionState.SetString(Key, config.ToString(Formatting.None));
            EditorApplication.isPlaying = true;
            return directory;
        }

        public static string Status() => active
            ? JsonConvert.SerializeObject(new { phase = finishing ? "finishing" : "running", output, imageCount, detectionCount, travelled })
            : "Idle; inspect the trial result.json for completion.";

        public static void Stop(string reason = "Operator stopped")
        {
            if (active) BeginFinish(reason, false);
            else SessionState.EraseString(Key);
        }

        static JObject Configuration(AIFormulaDifferentialDrive car)
        {
            var bodies = new JArray();
            foreach (var body in car.GetComponentsInChildren<Rigidbody>(true)) bodies.Add(new JObject {
                ["name"] = body.name, ["position"] = Vector(body.position), ["rotation"] = Rotation(body.rotation),
                ["mass"] = body.mass, ["isKinematic"] = body.isKinematic,
                ["constraints"] = body.constraints.ToString(), ["solverIterations"] = body.solverIterations,
                ["solverVelocityIterations"] = body.solverVelocityIterations
            });
            var feed = car.GetComponent<AIFormulaRgbCameraTopic>();
            return new JObject {
                ["bodies"] = bodies, ["drive"] = JsonUtility.ToJson(car),
                ["contact"] = car.GetComponent<AIFormulaTireContact>() ? JsonUtility.ToJson(car.GetComponent<AIFormulaTireContact>()) : null,
                ["rgb_topic"] = feed ? JsonUtility.ToJson(feed) : null,
                ["rgb_camera"] = feed && feed.rgbCamera ? EditorJsonUtility.ToJson(feed.rgbCamera) : null,
                ["preset"] = car.GetComponent<AIFormulaRuntimeTuning>() ? car.GetComponent<AIFormulaRuntimeTuning>().SerializePreset() : null,
                ["fixedDeltaTime"] = Time.fixedDeltaTime, ["timeScale"] = Time.timeScale
            };
        }

        static void Tick()
        {
            if (!EditorApplication.isPlaying || EditorApplication.isCompiling || EditorApplication.isPaused) return;
            try {
                if (!active) {
                    string pending = SessionState.GetString(Key, "");
                    if (string.IsNullOrEmpty(pending)) return;
                    request = JObject.Parse(pending);
                    output = (string)request["directory"];
                    var panel = Object.FindObjectOfType<AIFormulaDebugPanel>();
                    if (!panel || !panel.Canvas) {
                        if (EditorApplication.timeSinceStartup - (double)request["requested_editor_wall"] > 30)
                            throw new TimeoutException("Debug panel did not initialize within 30 wall seconds after Start.");
                        return;
                    }
                    SessionState.EraseString(Key);
                    Initialize(panel);
                }
                if (!drive || !detector || !semantic || !follower || !signal) {
                    BeginFinish("Required component lost", false); Complete(); return;
                }
                double now = Time.timeAsDouble;
                var position = drive.Body.position;
                travelled += new Vector2(position.x - lastPosition.x, position.z - lastPosition.z).magnitude;
                lastPosition = position;
                if (detector.FrameSequence > 0 && detector.FrameSequence != sequence) RecordDetection();
                if (now >= nextMotion) {
                    nextMotion = now + MotionPeriod;
                    motion.WriteLine(Sample("motion").ToString(Formatting.None)); motionCount++;
                }
                if (now >= nextImage) { nextImage = now + CapturePeriod; Capture(); }
                if (EditorApplication.timeSinceStartup >= nextStatus) { nextStatus = EditorApplication.timeSinceStartup + 1; WriteStatus(); }
                if (finishing) {
                    if (drive.Body.velocity.sqrMagnitude > .0025f || drive.Body.angularVelocity.sqrMagnitude > .0025f)
                        finishAt = now + 1;
                    if (now >= finishAt || EditorApplication.timeSinceStartup >= tailWallDeadline) Complete();
                    return;
                }
                Evaluate(now);
            } catch (Exception error) {
                Debug.LogException(error);
                if (active) { BeginFinish(error.ToString(), false); Complete(); }
                else {
                    SessionState.EraseString(Key);
                    if (!string.IsNullOrEmpty(output)) File.WriteAllText(Path.Combine(output, "result.json"),
                        JsonConvert.SerializeObject(new { accepted = false, reason = error.ToString(), stage = "initialization" }, Formatting.Indented));
                }
            }
        }

        static void Initialize(AIFormulaDebugPanel panel)
        {
            drive = panel.GetComponent<AIFormulaDifferentialDrive>();
            detector = panel.GetComponent<AIFormulaRoadDetector>();
            follower = panel.GetComponent<AIFormulaVisionLaneFollower>();
            if (!drive || !detector || !follower)
                throw new InvalidOperationException("Play debug panel did not install the required motor/perception/follower components.");
            semantic = panel.GetComponent<AIFormulaSemanticPlanning>();
            if (!semantic) semantic = panel.gameObject.AddComponent<AIFormulaSemanticPlanning>();
            var signals = Object.FindObjectsOfType<TrainingFlagPanel>();
            if (signals.Length != 1) throw new InvalidOperationException("Expected exactly one existing training traffic panel.");
            signal = signals[0];
            var follow = Object.FindObjectOfType<AIFormulaFollowCamera>();
            if (!follow || follow.target != drive.transform) throw new InvalidOperationException("Bound observer camera missing.");
            source = follow.GetComponent<Camera>();
            if (!source) throw new InvalidOperationException("Observer camera component missing.");
            trace = Writer("trace.jsonl"); motion = Writer("motion.jsonl");
            frames = Writer("frames.jsonl"); events = Writer("events.jsonl");
            active = true; finishing = accepted = stoppedForRed = crossedSignal = redViolation = semanticReadySeen = false;
            imageCount = detectionCount = motionCount = supportedCount = 0; sequence = 0;
            travelled = 0; minimumStopZ = float.PositiveInfinity; lastPerception = null;
            movementStart = waitStart = greenAt = -1; overlayPath = rgbPath = null; finishReason = null;
            startWall = EditorApplication.timeSinceStartup; startSim = Time.timeAsDouble;
            nextImage = nextMotion = nextStatus = 0;
            File.WriteAllText(Path.Combine(output, "play_before.json"), Configuration(drive).ToString(Formatting.Indented));
            MoveArticulatedVehicle((float)request["lane_x"], (float)request["start_z"]);
            follow.ApplyPose();
            lastPosition = drive.Body.position;
            signal.SetState((bool)request["initial_red"] ? TrainingFlagPanel.SignalState.Red : TrainingFlagPanel.SignalState.Green);
            File.WriteAllText(Path.Combine(output, "play_start.json"), Configuration(drive).ToString(Formatting.Indented));
            Event("initialized", new JObject { ["signal_position"] = Vector(signal.transform.position), ["signal_rotation"] = Rotation(signal.transform.rotation) });
            var holder = new GameObject("Semantic traffic read-only observer") { hideFlags = HideFlags.HideAndDontSave };
            capture = holder.AddComponent<Camera>(); capture.enabled = false;
            target = new RenderTexture(1280, 720, 24, RenderTextureFormat.ARGB32) { hideFlags = HideFlags.HideAndDontSave };
            target.Create();
            pixels = new Texture2D(1280, 720, TextureFormat.RGB24, false) { hideFlags = HideFlags.HideAndDontSave };
            panel.SetVisible(false);
            detector.Initialize(); originalDetectorOutput = detector.Configuration.output;
            originalDetectorFrequency = detector.Configuration.frequency;
            originalPanelRgb = detector.Configuration.enableSignalPanelRgb;
            detector.Configuration.frequency = (float?)request["perception_hz"] ?? 5;
            detector.Configuration.enableSignalPanelRgb = (bool?)request["signal_panel_rgb"] ?? false;
            AIFormulaRoadDetector.Validate(detector.Configuration);
            detector.Configuration.output = 0; // 仅本次 Play 选择既有识别 overlay，不改持久设置。
            File.WriteAllText(Path.Combine(output, "perception_settings.json"), JsonConvert.SerializeObject(detector.Configuration, Formatting.Indented));
            var calibration = typeof(AIFormulaRoadDetector).GetMethod("CameraCalibration", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);
            File.WriteAllText(Path.Combine(output, "camera_calibration.json"), JsonConvert.SerializeObject(calibration.Invoke(detector, null), Formatting.Indented));
            detector.SetRunning(true);
            semantic.Begin();
            follower.Begin();
            Debug.Log("SEMANTIC_TRAFFIC_STARTED " + output);
        }

        static void MoveArticulatedVehicle(float x, float z)
        {
            drive.StopMotion();
            var bodies = drive.GetComponentsInChildren<Rigidbody>(true);
            if (bodies.Length != 3) throw new InvalidOperationException("Articulated body count changed.");
            var origin = drive.Body.position;
            var delta = Quaternion.Euler(0, -drive.Body.rotation.eulerAngles.y, 0);
            var destination = new Vector3(x, origin.y, z);
            var positions = new Vector3[bodies.Length];
            var rotations = new Quaternion[bodies.Length];
            for (int i = 0; i < bodies.Length; i++) {
                positions[i] = destination + delta * (bodies[i].position - origin);
                rotations[i] = delta * bodies[i].rotation;
            }
            // 先保存所有世界位姿，再共同移动，避免父子 Transform 移动被重复计算。
            for (int i = 0; i < bodies.Length; i++) {
                bodies[i].position = positions[i]; bodies[i].rotation = rotations[i];
                bodies[i].velocity = Vector3.zero; bodies[i].angularVelocity = Vector3.zero;
            }
            Physics.SyncTransforms();
            drive.StopMotion();
        }

        static void RecordDetection()
        {
            sequence = detector.FrameSequence; detectionCount++;
            lastPerception = string.IsNullOrEmpty(detector.LastPayload) ? null : JObject.Parse(detector.LastPayload);
            if (lastPerception != null) foreach (string name in new[] { "overlay", "lane", "obstacles" }) lastPerception.Remove(name);
            if ((bool?)detector.LaneControl?["supported"] == true) supportedCount++;
            if (detector.Output) {
                overlayPath = "overlay_" + detectionCount.ToString("D5") + ".png";
                File.WriteAllBytes(Path.Combine(output, overlayPath), detector.Output.EncodeToPNG());
            }
            if (detector.Input) {
                rgbPath = "rgb_" + detectionCount.ToString("D5") + ".jpg";
                File.WriteAllBytes(Path.Combine(output, rgbPath), detector.Input.EncodeToJPG(90));
            }
            if (detector.LaneMask) File.WriteAllBytes(Path.Combine(output, "lane_" + detectionCount.ToString("D5") + ".png"), detector.LaneMask.EncodeToPNG());
            trace.WriteLine(Sample("detection").ToString(Formatting.None));
        }

        static JObject Sample(string kind)
        {
            var position = drive.Body.position;
            var perception = lastPerception;
            return new JObject {
                ["kind"] = kind, ["wall"] = EditorApplication.timeSinceStartup - startWall,
                ["t"] = Time.timeAsDouble - startSim, ["sequence"] = detector.FrameSequence,
                ["frame_sim_time"] = detector.FrameSimulationTime, ["epoch"] = detector.LaneEpoch,
                ["x"] = position.x, ["z"] = position.z, ["yaw"] = drive.Body.rotation.eulerAngles.y,
                ["actual_v"] = Vector3.Dot(drive.Body.velocity, drive.Body.rotation * Vector3.forward),
                ["actual_w"] = -drive.Body.angularVelocity.y,
                ["applied_v"] = drive.RequestedVelocity.x, ["applied_w"] = -drive.RequestedVelocity.y,
                ["signal_truth"] = signal.State.ToString().ToLowerInvariant(), ["detected_light"] = detector.TrafficLight,
                ["detected_light_original"] = perception?["traffic_light_original"],
                ["detected_light_source"] = perception?["traffic_light_source"],
                ["signal_panel"] = perception?["signal_panel"]?.DeepClone(),
                ["detections"] = perception?["detections"]?.DeepClone(),
                ["semantic"] = semantic.Result?.DeepClone(), ["candidate"] = detector.LaneControl?.DeepClone(),
                ["semantic_running"] = semantic.Running, ["semantic_runtime_status"] = semantic.Status,
                ["follower_stop_evidence"] = follower.StopEvidence?.DeepClone(),
                ["semantic_result_age"] = semantic.Result == null ? (double?)null : semantic.ResultAge,
                ["overlay"] = overlayPath, ["rgb"] = rgbPath,
                ["inference_ms"] = detector.InferenceMilliseconds, ["age"] = detector.FrameAge,
                ["distance"] = travelled, ["follower_status"] = follower.Status,
                ["phase"] = finishing ? "stopped_tail" : "running"
            };
        }

        static void Capture()
        {
            if (!source || !capture || !target || !pixels) return;
            capture.CopyFrom(source); capture.enabled = false; capture.targetTexture = target;
            capture.rect = new Rect(0, 0, 1, 1); capture.aspect = 16f / 9; capture.ResetProjectionMatrix();
            capture.transform.SetPositionAndRotation(source.transform.position, source.transform.rotation);
            var previous = RenderTexture.active;
            try {
                capture.Render(); RenderTexture.active = target;
                pixels.ReadPixels(new Rect(0, 0, 1280, 720), 0, 0, false); pixels.Apply(false, false);
            } finally { RenderTexture.active = previous; }
            string path = "chase_" + imageCount.ToString("D5") + ".jpg";
            File.WriteAllBytes(Path.Combine(output, path), pixels.EncodeToJPG(90));
            frames.WriteLine(JsonConvert.SerializeObject(new {
                index = imageCount, wall = EditorApplication.timeSinceStartup - startWall,
                t = Time.timeAsDouble - startSim, path, overlay = overlayPath, rgb = rgbPath,
                sequence = detector.FrameSequence, phase = finishing ? "stopped_tail" : "running"
            }));
            imageCount++;
        }

        static void Evaluate(double now)
        {
            // 全局时限先于加载状态分支，握手等待不能跳过 watchdog。
            double wallElapsed = EditorApplication.timeSinceStartup - startWall;
            if (wallElapsed > 240) { BeginFinish("Wall-time watchdog after 240 seconds", false); return; }
            if (detector.Faulted) { BeginFinish("Recognition: " + detector.Status, false); return; }
            if (semantic.Faulted) { BeginFinish("Semantic: " + semantic.Status, false); return; }
            if (!follower.Running) { BeginFinish("Follower: " + follower.Status, false); return; }
            if (!semantic.Running) {
                if (!semantic.Requested || semanticReadySeen) {
                    BeginFinish("Semantic stopped before trial completion: " + semantic.Status, false); return;
                }
                // Begin 返回时还在加载。Follower 的 TryGetCommand 路径保持制动，等待真实握手。
                if (wallElapsed > 35) BeginFinish("Semantic startup handshake exceeded 35 wall seconds", false);
                return;
            }
            if (!semanticReadySeen) { semanticReadySeen = true; Event("semantic_ready"); }
            var p = drive.Body.position;
            float speed = Vector3.Dot(drive.Body.velocity, drive.Body.rotation * Vector3.forward);
            if (movementStart < 0 && (Mathf.Abs(speed) > .05f || Mathf.Abs(drive.RequestedVelocity.x) > .05f)) {
                movementStart = now; Event("movement_started");
            }
            if (signal.State == TrainingFlagPanel.SignalState.Red && p.z > SignalPlaneZ && !redViolation) {
                redViolation = true; Event("critical_red_signal_plane_crossed");
            }
            if (signal.State == TrainingFlagPanel.SignalState.Red && p.z > SignalPlaneZ + 4) {
                BeginFinish("CRITICAL: red signal crossed without stopping (z > 19 m)", false); return;
            }
            bool waiting = (string)semantic.Result?["status"]?["traffic_mode"] == "wait";
            if (signal.State == TrainingFlagPanel.SignalState.Red && waiting && Mathf.Abs(speed) < .05f && p.z <= SignalPlaneZ) {
                if (waitStart < 0) { waitStart = now; Event("red_wait_started"); }
                minimumStopZ = Mathf.Min(minimumStopZ, p.z);
                if (now - waitStart >= 3 && !stoppedForRed) {
                    stoppedForRed = true; Event("red_stop_held_3_seconds");
                    if ((bool)request["release_green"]) {
                        signal.SetState(TrainingFlagPanel.SignalState.Green); greenAt = now; Event("signal_changed_green");
                    } else { BeginFinish("Red stop held for 3 seconds", !redViolation); return; }
                }
            } else waitStart = -1;
            if (signal.State == TrainingFlagPanel.SignalState.Green && p.z > SignalPlaneZ && !crossedSignal) {
                crossedSignal = true; Event("green_signal_passed");
            }
            if (p.x <= 38 && p.z >= 35) {
                bool pass = crossedSignal && !redViolation && (!(bool)request["initial_red"] || stoppedForRed);
                BeginFinish(pass ? "East straight, complete northeast bend and north straight completed" : "Segment ended without required stop/pass evidence", pass);
                return;
            }
            if (travelled >= 100) { BeginFinish("Local segment exceeded 100 m without reaching its exit", false); return; }
            // 仅用于局部试验的明显出界 watchdog；不向控制器提供道路参考。
            if (!Finite(p.x) || !Finite(p.y) || !Finite(p.z) || p.x < 34 || p.x > 80 || p.z < Math.Min(-14, (float)request["start_z"] - 5) || p.z > 47
                || p.y < -1 || p.y > 3 || Vector3.Dot(drive.Body.rotation * Vector3.up, Vector3.up) < .7f) {
                BeginFinish("CRITICAL: vehicle left the local trial region or overturned", false); return;
            }
            if (movementStart >= 0 && now - movementStart > 60) { BeginFinish("Motion timeout after 60 simulation seconds", false); return; }
            if (movementStart < 0 && wallElapsed > 100) BeginFinish("No motion within 100 wall seconds", false);
        }

        static void Event(string name, JObject detail = null)
        {
            var value = new JObject { ["event"] = name, ["wall"] = EditorApplication.timeSinceStartup - startWall,
                ["t"] = Time.timeAsDouble - startSim, ["detail"] = detail };
            if (drive) { value["x"] = drive.Body.position.x; value["z"] = drive.Body.position.z; }
            events?.WriteLine(value.ToString(Formatting.None));
        }

        static void WriteStatus()
        {
            File.WriteAllText(Path.Combine(output, "status.json"), JsonConvert.SerializeObject(new {
                phase = finishing ? "stopped_tail" : "running", reason = finishReason, imageCount, detectionCount, motionCount,
                travelled, stoppedForRed, crossedSignal, redViolation, detector = detector.Status, semantic = semantic.Status,
                follower = follower.Status, detectorWorker = detector.WorkerId, semanticWorker = semantic.WorkerId,
                wall = EditorApplication.timeSinceStartup - startWall, t = Time.timeAsDouble - startSim
            }, Formatting.Indented));
        }

        static void BeginFinish(string reason, bool pass)
        {
            if (!active || finishing) return;
            finishing = true; accepted = pass; finishReason = reason; finishAt = Time.timeAsDouble + 1;
            tailWallDeadline = EditorApplication.timeSinceStartup + 10;
            Event("trial_stop", new JObject { ["accepted"] = pass, ["reason"] = reason });
            // 保存终止时的候选与语义状态；Stop 会清理控制响应，不能反过来丢失证据。
            if (drive && detector && semantic && follower && signal)
                File.WriteAllText(Path.Combine(output, "terminal_sample.json"), Sample("terminal").ToString(Formatting.Indented));
            if (follower) follower.Stop(reason);
            if (drive) drive.SetInput(0, 0, true);
            var driver = drive ? drive.GetComponent<AIFormulaDriverInput>() : null;
            if (driver) driver.SetEmergencyStop(true);
            WriteStatus();
        }

        static void Complete()
        {
            if (!active) return;
            var cleanupErrors = new List<string>();
            try {
                try { if (follower) follower.Stop(finishReason); } catch (Exception error) { cleanupErrors.Add("follower: " + error.Message); }
                try { if (semantic) semantic.Stop(); } catch (Exception error) { cleanupErrors.Add("semantic: " + error.Message); }
                try {
                    if (detector) { detector.SetRunning(false); detector.Configuration.output = originalDetectorOutput; detector.Configuration.frequency = originalDetectorFrequency; detector.Configuration.enableSignalPanelRgb = originalPanelRgb; }
                } catch (Exception error) { cleanupErrors.Add("detector: " + error.Message); }
                try { if (drive) drive.SetInput(0, 0, true); } catch (Exception error) { cleanupErrors.Add("motor: " + error.Message); }
                if (cleanupErrors.Count > 0) accepted = false;
                File.WriteAllText(Path.Combine(output, "result.json"), JsonConvert.SerializeObject(new {
                    accepted, reason = finishReason, @case = (string)request["case"], imageCount, detectionCount, motionCount, supportedCount,
                    travelled, stoppedForRed, crossedSignal, redViolation,
                    stop_z = float.IsPositiveInfinity(minimumStopZ) ? (float?)null : minimumStopZ,
                    green_t = greenAt < 0 ? (double?)null : greenAt - startSim,
                    seconds = Time.timeAsDouble - startSim, wallSeconds = EditorApplication.timeSinceStartup - startWall,
                    captureHz = 15, motionHz = 20, framesUseActualTimestamps = true, signalPlaneZ = SignalPlaneZ,
                    initialPoseChangedOnce = true, articulatedBodiesMovedTogether = true,
                    signalTruthFedToControl = false, evaluatorRouteFedToControl = false,
                    odometrySource = "Unity Rigidbody pose and velocity; ideal noiseless simulated odometry",
                    physicsParametersChanged = false, cameraIntrinsicsChanged = false,
                    detectorWorker = detector ? detector.WorkerId : 0, semanticWorker = semantic ? semantic.WorkerId : 0,
                    cleanupErrors, terminalBodySpeed = drive ? drive.Body.velocity.magnitude : (float?)null,
                    evidence = "Live Unity Play, genuine RGB model output, native LYA and semantic result. Main video uses an observer-camera clone. Remains in Play with E-STOP."
                }, Formatting.Indented));
                File.WriteAllText(Path.Combine(output, "play_after.json"), Configuration(drive).ToString(Formatting.Indented));
            } finally {
                trace?.Dispose(); motion?.Dispose(); frames?.Dispose(); events?.Dispose();
                trace = motion = frames = events = null;
                if (target) { target.Release(); Object.DestroyImmediate(target); }
                if (pixels) Object.DestroyImmediate(pixels);
                if (capture) Object.DestroyImmediate(capture.gameObject);
                target = null; pixels = null; capture = null; active = false;
                SessionState.EraseString(Key);
            }
            Debug.Log("SEMANTIC_TRAFFIC_FINISHED " + finishReason + " " + output);
        }

        static StreamWriter Writer(string name) => new StreamWriter(Path.Combine(output, name)) { AutoFlush = true };
        static JArray Vector(Vector3 value) => new JArray(value.x, value.y, value.z);
        static JArray Rotation(Quaternion value) => new JArray(value.x, value.y, value.z, value.w);
        static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
