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
    public static class AIFormulaNeoTopoTrial
    {
        const string Key = "AIFormula.NeoTopoTrial";
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
        static float travelled;
        static float stoppedFrontGap;
        static Vector3 lastPosition;
        static bool active, finishing, accepted, stoppedForRed, redViolation, semanticReadySeen;
        static string stage, oldBoot, lastSemanticIdentity;
        static int oldGeneration, stableDetectorWorker, stableLaneEpoch, initialSemanticGeneration;
        static double stageAt, restartSim, oldReferenceLength, noProgressSince;
        static float restoredDistance, greenDistance, noProgressDistance;
        static double resumedAt;
        static Vector3 restartPosition;
        static Quaternion restartRotation;
        static bool firstLapClosed, partialRestored, referenceRestored, greenResumed;
        static bool redConfirmed, greenConfirmed;
        static JObject firstRedConfirmation, firstGreenConfirmation;
        static JObject learnedStatus, restartBefore;
        static JArray restartReceipts;
        static StreamWriter semanticTicks;
        static bool settingsCaptured, originalHold, originalApproach, originalNeoMode;
        static string originalSourceRoot, originalMotorGate, originalCheckpointRoot, originalCheckpointPath, originalWorkerRoot, originalDiagnosticRouteOutput;
        static int originalDetectorOutput;
        static float originalDetectorFrequency;
        static bool originalPanelRgb;
        static bool originalProjectedComponentRecovery, originalCurrentCorridor;
        static bool originalFirstFrameHeadroom;
        static bool originalApplicationFocus, applicationFocusOverridden;

        static AIFormulaNeoTopoTrial()
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

        public static string Start(string directory, string caseName, string caseKind, float laneX, float startZ,
                                   string sourceRoot, string motorGateSource, string workerRoot, bool panelRgb = true, float partialRestartDistance = 120, string laneRecoveryMode = "baseline")
        {
            var scene = SceneManager.GetActiveScene();
            if (Application.isBatchMode || EditorApplication.isPlayingOrWillChangePlaymode || scene.isDirty || active)
                throw new InvalidOperationException("Requires exclusive clean GUI Edit Mode.");
            if (!scene.path.EndsWith("/MotorControllerValidation.unity", StringComparison.Ordinal))
                throw new InvalidOperationException("Open saved MotorControllerValidation scene first.");
            if (laneRecoveryMode != "baseline" && laneRecoveryMode != "component-only")
                throw new ArgumentException("laneRecoveryMode must be baseline or component-only.");
            if (caseKind != "traffic" && caseKind != "closure" && caseKind != "recovery")
                throw new ArgumentException("caseKind must be traffic, closure or recovery.");
            if (!Finite(laneX) || !Finite(startZ) || !Finite(partialRestartDistance) || partialRestartDistance < 20 || partialRestartDistance > 200)
                throw new ArgumentException("Finite start pose and a 20-200 m restart schedule required.");
            if (string.IsNullOrWhiteSpace(caseName) || caseName.IndexOfAny(Path.GetInvalidFileNameChars()) >= 0 || caseName == "." || caseName == "..")
                throw new ArgumentException("caseName must be one safe directory name.");
            sourceRoot = Path.GetFullPath(sourceRoot); motorGateSource = Path.GetFullPath(motorGateSource); workerRoot = Path.GetFullPath(workerRoot);
            if (!Directory.Exists(sourceRoot) || !File.Exists(motorGateSource) || !File.Exists(Path.Combine(workerRoot, "semantic_worker.py"))) throw new ArgumentException("Pinned source paths missing.");
            var car = Object.FindObjectOfType<AIFormulaDifferentialDrive>();
            if (!car || !car.GetComponent<AIFormulaDebugPanel>() || !car.GetComponent<AIFormulaRgbCameraTopic>() || !car.GetComponent<AIFormulaDriverInput>())
                throw new InvalidOperationException("Existing vehicle, panel, RGB topic and driver required.");
            var bridge = car.GetComponent<AIFormulaLaneRuntimeBridge>(); var demo = car.GetComponent<AIFormulaE2EDemo>();
            if ((bridge && bridge.isActiveAndEnabled) || (demo && demo.isActiveAndEnabled)) throw new InvalidOperationException("Another autonomous mode owns scene.");
            directory = Path.GetFullPath(directory);
            if (Directory.Exists(directory)) throw new InvalidOperationException("Preserve evidence: output exists.");
            Directory.CreateDirectory(directory);
            var config = new JObject {
                ["directory"] = directory, ["case"] = caseName, ["case_kind"] = caseKind, ["lane_x"] = laneX, ["start_z"] = startZ,
                ["source_root"] = sourceRoot, ["motor_gate_source"] = motorGateSource, ["worker_root"] = workerRoot,
                ["checkpoint_root"] = Path.Combine(directory, "checkpoints"), ["checkpoint_path"] = Path.Combine(directory, "checkpoints", caseName, "topo.json"),
                ["traffic_policy"] = "stop_wait", ["distance_enabled"] = false, ["bbox_enabled"] = false, ["automatic_detour_enabled"] = false, ["signal_approach"] = false,
                ["lane_recovery_mode"] = laneRecoveryMode,
                ["runtime_first_frame_headroom"] = true,
                ["runtime_option_reason"] = "Cold first inference must satisfy existing 0.5 s first-accept gate; native 0.75 s active-command TTL unchanged",
                ["initial_red"] = caseKind == "traffic", ["signal_panel_rgb"] = panelRgb, ["partial_restart_distance_m"] = partialRestartDistance,
                ["scene"] = scene.path, ["requested_utc"] = DateTime.UtcNow.ToString("o"), ["requested_editor_wall"] = EditorApplication.timeSinceStartup,
                ["wall_timeout_sec"] = caseKind == "traffic" ? 180 : 420, ["motion_timeout_sec"] = 260, ["distance_limit_m"] = 600,
                ["reference_role"] = "One initial pose and physical lamp schedule only. No evaluator route/range/truth enters control. Rigidbody supplies ideal simulated odometry.",
                ["preview_rgb_role"] = "Decoded/re-encoded preview; not exact model input bytes", ["baseline"] = Configuration(car)
            };
            File.WriteAllText(Path.Combine(directory, "configuration.json"), config.ToString(Formatting.Indented));
            SessionState.SetString(Key, config.ToString(Formatting.None)); EditorApplication.isPlaying = true; return directory;
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
                RecordSemantic();
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
            if (detector.Requested || follower.Running) throw new InvalidOperationException("A live perception/follower session already owns the vehicle.");
            semantic = panel.GetComponent<AIFormulaSemanticPlanning>();
            if (!semantic) semantic = panel.gameObject.AddComponent<AIFormulaSemanticPlanning>();
            if (semantic.Requested) throw new InvalidOperationException("A live semantic session already owns the vehicle.");
            var signals = Object.FindObjectsOfType<TrainingFlagPanel>();
            if (signals.Length != 1) throw new InvalidOperationException("Expected exactly one existing training traffic panel.");
            signal = signals[0];
            var follow = Object.FindObjectOfType<AIFormulaFollowCamera>();
            if (!follow || follow.target != drive.transform) throw new InvalidOperationException("Bound observer camera missing.");
            source = follow.GetComponent<Camera>();
            if (!source) throw new InvalidOperationException("Observer camera component missing.");
            trace = Writer("trace.jsonl"); motion = Writer("motion.jsonl");
            frames = Writer("frames.jsonl"); events = Writer("events.jsonl");
            semanticTicks = Writer("semantic_ticks.jsonl");
            stage = (string)request["case_kind"] == "traffic" ? "red_wait" : "learning";
            firstLapClosed = partialRestored = referenceRestored = greenResumed = false;
            redConfirmed = greenConfirmed = false; firstRedConfirmation = firstGreenConfirmation = null;
            oldBoot = lastSemanticIdentity = null; oldGeneration = 0; stableDetectorWorker = stableLaneEpoch = -1;
            learnedStatus = restartBefore = null; restartReceipts = new JArray();
            stageAt = EditorApplication.timeSinceStartup; noProgressSince = resumedAt = -1; settingsCaptured = false;
            active = true; finishing = accepted = stoppedForRed = redViolation = semanticReadySeen = false;
            stoppedFrontGap = float.NaN;
            imageCount = detectionCount = motionCount = supportedCount = 0; sequence = 0;
            travelled = 0; lastPerception = null;
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
            originalDetectorFrequency = detector.Configuration.frequency; originalPanelRgb = detector.Configuration.enableSignalPanelRgb;
            originalProjectedComponentRecovery = detector.RecoverProjectedComponents;
            originalCurrentCorridor = detector.UseCurrentCorridor;
            originalFirstFrameHeadroom = follower.RequireFirstFrameHeadroom;
            originalApplicationFocus = follower.RequireApplicationFocus; applicationFocusOverridden = true;
            originalHold = semantic.DiagnosticHold; originalApproach = semantic.EnableSignalApproach;
            originalSourceRoot = semantic.SourceRoot; originalMotorGate = semantic.MotorGateSource;
            originalCheckpointRoot = semantic.CheckpointRoot; originalCheckpointPath = semantic.CheckpointPath;
            originalNeoMode = semantic.NeoStopWaitMode; originalWorkerRoot = semantic.WorkerRoot;
            originalDiagnosticRouteOutput = semantic.DiagnosticRouteOutput;
            settingsCaptured = true;
            detector.Configuration.enableSignalPanelRgb = (bool)request["signal_panel_rgb"]; detector.Configuration.output = 0;
            if ((string)request["lane_recovery_mode"] == "component-only") {
                detector.RecoverProjectedComponents = true; detector.UseCurrentCorridor = false;
            }
            AIFormulaRoadDetector.Validate(detector.Configuration);
            // Native LLP/LYA/frequency stay fixed. Only named existing runtime options differ from baseline.
            follower.RequireApplicationFocus = false;
            follower.RequireFirstFrameHeadroom = true;
            semantic.NeoStopWaitMode = true; semantic.WorkerRoot = (string)request["worker_root"];
            semantic.DiagnosticRouteOutput = null;
            semantic.EnableSignalApproach = false; semantic.DiagnosticHold = false;
            semantic.SourceRoot = (string)request["source_root"]; semantic.MotorGateSource = (string)request["motor_gate_source"];
            semantic.CheckpointRoot = (string)request["checkpoint_root"]; semantic.CheckpointPath = (string)request["checkpoint_path"];
            File.WriteAllText(Path.Combine(output, "perception_settings.json"), JsonConvert.SerializeObject(detector.Configuration, Formatting.Indented));
            var calibration = typeof(AIFormulaRoadDetector).GetMethod("CameraCalibration", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);
            if (calibration == null) throw new MissingMethodException("CameraCalibration missing.");
            File.WriteAllText(Path.Combine(output, "camera_calibration.json"), JsonConvert.SerializeObject(calibration.Invoke(detector, null), Formatting.Indented));
            initialSemanticGeneration = semantic.WorkerGeneration;
            detector.SetRunning(true); semantic.Begin(); follower.Begin();
            Event("preserved_native_settings", new JObject { ["frequency"] = detector.Configuration.frequency,
                ["lane_recovery_mode"] = request["lane_recovery_mode"], ["recover_projected_components_original"] = originalProjectedComponentRecovery,
                ["use_current_corridor_original"] = originalCurrentCorridor,
                ["recover_projected_components"] = detector.RecoverProjectedComponents, ["use_current_corridor"] = detector.UseCurrentCorridor,
                ["first_frame_headroom_original"] = originalFirstFrameHeadroom, ["first_frame_headroom_active"] = follower.RequireFirstFrameHeadroom });
            Debug.Log("NEO_TOPO_STARTED " + output);
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
            if (lastPerception?["lane"]?.Type == JTokenType.String) {
                byte[] packet = Convert.FromBase64String((string)lastPerception["lane"]);
                string packetName = "lane_packet_" + detectionCount.ToString("D5") + ".png";
                File.WriteAllBytes(Path.Combine(output, packetName), packet); lastPerception["lane_packet_file"] = packetName;
                using (var hash = System.Security.Cryptography.SHA256.Create())
                    lastPerception["lane_packet_sha256"] = BitConverter.ToString(hash.ComputeHash(packet)).Replace("-", "").ToLowerInvariant();
            }
            if (lastPerception != null) foreach (string name in new[] { "overlay", "lane", "obstacles" }) lastPerception.Remove(name);
            if ((bool?)detector.LaneControl?["supported"] == true) supportedCount++;
            if (detector.Output) {
                overlayPath = "overlay_" + detectionCount.ToString("D5") + ".png";
                File.WriteAllBytes(Path.Combine(output, overlayPath), detector.Output.EncodeToPNG());
            }
            if (detector.Input) {
                rgbPath = "rgb_" + detectionCount.ToString("D5") + ".png";
                File.WriteAllBytes(Path.Combine(output, rgbPath), detector.Input.EncodeToPNG());
            }
            if (detector.LaneMask) File.WriteAllBytes(Path.Combine(output, "lane_" + detectionCount.ToString("D5") + ".png"), detector.LaneMask.EncodeToPNG());
            trace.WriteLine(Sample("detection").ToString(Formatting.None));
        }

        static void RecordSemantic()
        {
            var result = semantic.Result; if (result == null) return;
            long current = (long?)result["sequence"] ?? 0;
            string identity = semantic.WorkerGeneration + ":" + current;
            if (current <= 0 || identity == lastSemanticIdentity) return;
            lastSemanticIdentity = identity;
            semanticTicks.WriteLine(Sample("semantic_response").ToString(Formatting.None));
        }
        static JObject Sample(string kind)
        {
            var position = drive.Body.position;
            var perception = lastPerception;
            return new JObject {
                ["case_kind"] = request["case_kind"], ["stage"] = stage,
                ["worker_generation"] = semantic.WorkerGeneration, ["worker_boot_id"] = semantic.WorkerBootId,
                ["diagnostic_hold"] = semantic.DiagnosticHold, ["absolute_sim_time"] = Time.timeAsDouble, ["start_sim_time"] = startSim,
                ["traffic_stop_required"] = semantic.Result?["status"]?["traffic_stop_required"],
                ["topo_closure"] = semantic.Result?["status"]?["closure"]?.DeepClone(), ["persistence"] = semantic.Result?["status"]?["persistence"]?.DeepClone(),
                ["command_gate"] = semantic.Result?["command_gate"]?.DeepClone(), ["restore_event"] = semantic.Result?["restore_event"]?.DeepClone(),
                ["semantic_worker"] = semantic.WorkerId, ["detector_worker"] = detector.WorkerId,
                ["kind"] = kind, ["wall"] = EditorApplication.timeSinceStartup - startWall,
                ["t"] = Time.timeAsDouble - startSim, ["sequence"] = detector.FrameSequence,
                ["frame_sim_time"] = detector.FrameSimulationTime, ["epoch"] = detector.LaneEpoch,
                ["x"] = position.x, ["z"] = position.z, ["yaw"] = drive.Body.rotation.eulerAngles.y,
                ["actual_v"] = Vector3.Dot(drive.Body.velocity, drive.Body.rotation * Vector3.forward),
                ["actual_w"] = -drive.Body.angularVelocity.y,
                ["body_velocity_world"] = Vector(drive.Body.velocity), ["body_angular_velocity_world"] = Vector(drive.Body.angularVelocity),
                ["body_speed"] = drive.Body.velocity.magnitude, ["body_angular_speed"] = drive.Body.angularVelocity.magnitude,
                ["applied_v"] = drive.RequestedVelocity.x, ["applied_w"] = -drive.RequestedVelocity.y,
                ["signal_truth"] = signal.State.ToString().ToLowerInvariant(), ["detected_light"] = detector.TrafficLight,
                ["detected_light_original"] = perception?["traffic_light_original"],
                ["detected_light_source"] = perception?["traffic_light_source"],
                ["signal_panel"] = perception?["signal_panel"]?.DeepClone(),
                ["actual_front_gap_m_evaluation_only"] = FrontGap(),
                ["detections"] = perception?["detections"]?.DeepClone(),
                ["semantic"] = semantic.Result?.DeepClone(), ["candidate"] = detector.LaneControl?.DeepClone(),
                ["semantic_running"] = semantic.Running, ["semantic_runtime_status"] = semantic.Status,
                ["follower_stop_evidence"] = follower.StopEvidence?.DeepClone(),
                ["semantic_result_age"] = semantic.Result == null ? (double?)null : semantic.ResultAge,
                ["overlay"] = overlayPath, ["rgb"] = rgbPath,
                ["lane_packet_file"] = perception?["lane_packet_file"], ["lane_packet_sha256"] = perception?["lane_packet_sha256"],
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
                sequence = detector.FrameSequence, stage, phase = finishing ? "stopped_tail" : "running"
            }));
            imageCount++;
        }

        static void Evaluate(double now)
        {
            double wall = EditorApplication.timeSinceStartup - startWall;
            if (wall > (double)request["wall_timeout_sec"]) { BeginFinish("Wall-time watchdog", false); return; }
            if (detector.Faulted || semantic.Faulted || !follower.Running) {
                BeginFinish("Chain stopped: detector=" + detector.Status + "; semantic=" + semantic.Status + "; follower=" + follower.Status, false); return;
            }
            var p = drive.Body.position;
            if (!Finite(p.x) || !Finite(p.y) || !Finite(p.z) || p.x < -25 || p.x > 90 || p.z < -75 || p.z > 55
                || p.y < -1 || p.y > 3 || Vector3.Dot(drive.Body.rotation * Vector3.up, Vector3.up) < .7f) {
                BeginFinish("CRITICAL: gross bounds exceeded or vehicle overturned", false); return;
            }
            if (travelled > 600) { BeginFinish("600 m watchdog", false); return; }
            if (stableDetectorWorker >= 0 && (stableDetectorWorker != detector.WorkerId || stableLaneEpoch != detector.LaneEpoch)) {
                BeginFinish("Detector worker/epoch changed during continuous-odom case", false); return;
            }
            if (stage == "restart_partial" || stage == "restart_reference") { ObserveRestart(now); return; }
            if (!semantic.Running || semantic.Result == null) {
                if (!semantic.Requested || wall > 35) BeginFinish("Semantic startup produced no result within 35 wall seconds", false);
                return;
            }
            var status = semantic.Result["status"] as JObject;
            if (status == null || semantic.ResultAge > .25) {
                if ((string)request["case_kind"] == "traffic") {
                    waitStart = resumedAt = -1; redConfirmed = greenConfirmed = false;
                }
                return;
            }
            if ((string)status["traffic_policy"] != "stop_wait" || (bool?)status["signal_approach"]?["enabled"] != false) {
                BeginFinish("Mode separation failure: expected stop_wait and approach disabled", false); return;
            }
            if (stableDetectorWorker < 0 && detector.FrameSequence > 0) {
                stableDetectorWorker = detector.WorkerId; stableLaneEpoch = detector.LaneEpoch;
                Event("continuous_chain_identity", new JObject { ["detector_worker"] = stableDetectorWorker, ["lane_epoch"] = stableLaneEpoch });
            }
            if (!semanticReadySeen) { semanticReadySeen = true; Event("semantic_ready"); }
            float speed = Vector3.Dot(drive.Body.velocity, drive.Body.rotation * Vector3.forward);
            if (movementStart < 0 && Mathf.Abs(speed) > .05f) { movementStart = now; Event("movement_started"); }
            if ((string)request["case_kind"] == "traffic") { EvaluateTraffic(now, speed, status); return; }
            if (signal.State != TrainingFlagPanel.SignalState.Green) { BeginFinish("Unexpected lamp schedule change", false); return; }
            string persistence = (string)status["persistence"]?["state"];
            if (persistence == "restore_rejected" || persistence == "save_failed") { BeginFinish("Production checkpoint failed: " + status["persistence"], false); return; }
            if (stage == "hold_partial" || stage == "hold_reference") { ObserveHold(now, status); return; }
            double length = (double?)status["reference_length_m"] ?? double.NaN;
            bool closed = (string)status["closure"]?["state"] == "complete" && !double.IsNaN(length) && !double.IsInfinity(length) && length > 0;
            if (closed && !firstLapClosed) {
                firstLapClosed = true; learnedStatus = (JObject)status.DeepClone(); Event("topo_closed", status);
                if ((string)request["case_kind"] == "closure") { BeginFinish("Production topo closure complete", true); return; }
                if (!partialRestored) { BeginFinish("Closed before scheduled partial restart; partial recovery unverified", false); return; }
                StartHold("hold_reference"); return;
            }
            if ((string)request["case_kind"] == "recovery" && !partialRestored && stage == "learning"
                && ((double?)status["recorded_distance_m"] ?? 0) >= (double)request["partial_restart_distance_m"]
                && ((int?)status["recorded_samples"] ?? 0) >= 30) { StartHold("hold_partial"); return; }
            if (stage == "reference_resumed" && referenceRestored && travelled - restoredDistance >= 3
                && speed > .1f && drive.RequestedVelocity.x > .1f && (bool?)status["traffic_stop_required"] == false) {
                BeginFinish("Partial map restored, topo closed, reference restored and LYA resumed 3 m", true); return;
            }
            if (movementStart >= 0 && now - movementStart > 260) { BeginFinish("260 simulation-second watchdog", false); return; }
            if (movementStart < 0 && wall > 100) { BeginFinish("No motion within 100 wall seconds", false); return; }
            if (Mathf.Abs(speed) < .05f) {
                if (noProgressSince < 0 || travelled - noProgressDistance >= .1f) { noProgressSince = now; noProgressDistance = travelled; }
                if (now - noProgressSince >= 15) BeginFinish("No 0.1 m progress for 15 seconds outside planned hold", false);
            } else noProgressSince = -1;
        }

        static void EvaluateTraffic(double now, float speed, JObject status)
        {
            bool redPhase = stage == "red_wait";
            var expected = redPhase ? TrainingFlagPanel.SignalState.Red : TrainingFlagPanel.SignalState.Green;
            if (signal.State != expected) { BeginFinish("Unexpected lamp schedule change", false); return; }
            if (redPhase && FrontGap() < 0) { redViolation = true; BeginFinish("CRITICAL: front crossed red signal plane", false); return; }
            bool fresh = detector.FrameSequence > 0 && detector.FrameAge >= 0
                && detector.FrameAge <= AIFormulaVisionLaneFollower.MaximumCommandAge
                && now >= detector.FrameSimulationTime
                && now - detector.FrameSimulationTime <= AIFormulaVisionLaneFollower.MaximumCommandAge;
            bool zero = drive.Body.velocity.magnitude <= .03f && Mathf.Abs(drive.RequestedVelocity.x) <= .001f
                && Mathf.Abs(drive.RequestedVelocity.y) <= .001f && drive.Body.angularVelocity.magnitude <= .03f;
            if (redPhase) {
                bool waitLatched = (string)status["traffic_mode"] == "wait" && (bool?)status["traffic_stop_required"] == true;
                bool currentRed = fresh && detector.TrafficLight == "red";
                if (!waitLatched || !currentRed) { redConfirmed = false; waitStart = -1; return; }
                // The native latch outlives a sampled vote. Require a real red confirmation first,
                // then assess that latch and physical stop; intermediate voted unknown is not release.
                if (!redConfirmed && (string)status["traffic_light"] == "red") {
                    redConfirmed = true;
                    var proof = TrafficConfirmation("red");
                    if (firstRedConfirmation == null) firstRedConfirmation = (JObject)proof.DeepClone();
                    Event("native_red_confirmed", proof);
                }
                if (!redConfirmed || !zero) { waitStart = -1; return; }
                if (waitStart < 0) { waitStart = now; Event("red_zero_started"); }
                if (now - waitStart < 3) return;
                stoppedForRed = true; stoppedFrontGap = FrontGap();
                Event("red_stop_held_3_seconds", new JObject { ["actual_front_gap_m_evaluation_only"] = stoppedFrontGap, ["status"] = status.DeepClone() });
                signal.SetState(TrainingFlagPanel.SignalState.Green); greenAt = now; greenDistance = travelled;
                SetStage("green_resume"); Event("signal_changed_green"); return;
            }
            bool normal = (string)status["traffic_mode"] == "normal" && (bool?)status["traffic_stop_required"] == false;
            bool currentGreen = fresh && detector.TrafficLight == "green";
            if (!normal || !currentGreen) greenConfirmed = false;
            if (!greenConfirmed && normal && currentGreen && (string)status["traffic_light"] == "green") {
                greenConfirmed = true;
                var proof = TrafficConfirmation("green");
                if (firstGreenConfirmation == null) firstGreenConfirmation = (JObject)proof.DeepClone();
                Event("native_green_confirmed", proof);
            }
            bool released = greenConfirmed && normal && currentGreen && drive.RequestedVelocity.x > .1f && speed > .1f;
            if (released) {
                if (resumedAt < 0) resumedAt = now;
                if (!greenResumed && now - resumedAt >= 1) { greenResumed = true; Event("green_native_motion_resumed", status); }
            } else resumedAt = -1;
            if (stoppedForRed && greenResumed && released && travelled - greenDistance >= 5) { BeginFinish("Red zero held 3 s; green native LYA resumed 5 m", !redViolation); return; }
            if (now - greenAt > 45) BeginFinish("Green failed to resume 5 m within 45 simulation seconds", false);
        }

        static JObject TrafficConfirmation(string color)
        {
            return new JObject { ["confirmed_color"] = color, ["wall"] = EditorApplication.timeSinceStartup - startWall,
                ["absolute_sim_time"] = Time.timeAsDouble, ["detection_sequence"] = detector.FrameSequence,
                ["lane_epoch"] = detector.LaneEpoch, ["frame_sim_time"] = detector.FrameSimulationTime,
                ["frame_wall_age"] = detector.FrameAge, ["semantic_result_age"] = semantic.ResultAge,
                ["semantic_response"] = semantic.Result.DeepClone() };
        }
        static void StartHold(string value)
        {
            semantic.DiagnosticHold = true; waitStart = -1; noProgressSince = -1;
            SetStage(value); Event("restart_brake_requested", new JObject { ["role"] = "Test interruption only; preserve follower/detector/Body/clock" });
        }
        static void SetStage(string value) { stage = value; stageAt = EditorApplication.timeSinceStartup; Event("stage_changed"); }
        static void ObserveHold(double now, JObject status)
        {
            if (EditorApplication.timeSinceStartup - stageAt > 15) { BeginFinish("Settle/persist exceeded 15 wall seconds", false); return; }
            if (drive.Body.velocity.magnitude >= .03f || drive.Body.angularVelocity.magnitude >= .03f || drive.RequestedVelocity.magnitude >= .001f) { waitStart = -1; return; }
            if (waitStart < 0) waitStart = now;
            if (now - waitStart < 1) return;
            var persistence = status["persistence"] as JObject;
            if ((string)persistence?["state"] != "saved" || (persistence?["error"] != null && persistence["error"].Type != JTokenType.Null)) return;
            int count = (int?)status["recorded_samples"] ?? 0;
            if (count < 30 || ((int?)persistence?["saved_samples"] ?? -1) < count || !File.Exists(semantic.CheckpointPath)) return;
            var envelope = JObject.Parse(File.ReadAllText(semantic.CheckpointPath)); var saved = envelope["state"] as JObject;
            var rows = saved?["samples"] as JArray;
            if (rows == null || rows.Count < count) throw new InvalidDataException("Checkpoint saved sample count mismatch.");
            bool reference = stage == "hold_reference", savedClosed = saved["closed_at"] != null && saved["closed_at"].Type != JTokenType.Null;
            if (reference && !savedClosed) return;
            if (!reference && savedClosed) throw new InvalidDataException("Partial restart checkpoint is already closed.");
            oldGeneration = semantic.WorkerGeneration; oldBoot = semantic.WorkerBootId;
            oldReferenceLength = (double?)status["reference_length_m"] ?? 0;
            restartPosition = drive.Body.position; restartRotation = drive.Body.rotation; restartSim = now;
            restartBefore = new JObject { ["kind"] = reference ? "reference" : "partial", ["status"] = status.DeepClone(),
                ["worker_generation"] = oldGeneration, ["worker_boot_id"] = oldBoot, ["worker_pid"] = semantic.WorkerId,
                ["detector_worker"] = detector.WorkerId, ["lane_epoch"] = detector.LaneEpoch, ["absolute_sim_time"] = now,
                ["body_position"] = Vector(restartPosition), ["body_rotation"] = Rotation(restartRotation), ["checkpoint"] = envelope.DeepClone() };
            File.WriteAllText(Path.Combine(output, "restart_" + (reference ? "reference" : "partial") + "_before.json"), restartBefore.ToString(Formatting.Indented));
            Event("semantic_restart_requested", new JObject { ["kind"] = reference ? "reference" : "partial", ["generation"] = oldGeneration, ["boot_id"] = oldBoot });
            SetStage(reference ? "restart_reference" : "restart_partial");
            if (!semantic.RestartWorkerPreservingOdom()) BeginFinish("Semantic restart did not initiate", false);
        }
        static void ObserveRestart(double now)
        {
            if (EditorApplication.timeSinceStartup - stageAt > 35) { BeginFinish("Restart restore exceeded 35 wall seconds", false); return; }
            if (now < restartSim || Vector3.Distance(drive.Body.position, restartPosition) > .1f || Quaternion.Angle(drive.Body.rotation, restartRotation) > 2) {
                BeginFinish("Body or odometry continuity violated during stationary restart", false); return;
            }
            if (!semantic.Running || semantic.Result == null || semantic.ResultAge > .25) return;
            if (semantic.WorkerGeneration <= oldGeneration || string.IsNullOrEmpty(semantic.WorkerBootId)
                || semantic.WorkerBootId == oldBoot || (string)semantic.Result["worker_boot_id"] != semantic.WorkerBootId) return;
            var status = semantic.Result["status"] as JObject; var persistence = status?["persistence"] as JObject;
            string state = (string)persistence?["state"];
            if (state == "restore_rejected" || state == "save_failed") { BeginFinish("Restart rejected: " + persistence, false); return; }
            // restored may become saved before a status sample; this records the actual _on_odom restore.
            var restoreEvent = semantic.Result["restore_event"] as JObject;
            if ((bool?)restoreEvent?["observed"] != true || (string)restoreEvent["worker_boot_id"] != semantic.WorkerBootId
                || (string)restoreEvent["persistence"]?["state"] != "restored") return;
            var restoreError = restoreEvent["persistence"]?["error"];
            if (restoreError != null && restoreError.Type != JTokenType.Null) throw new InvalidDataException("Restore event contains error.");
            var original = (JArray)restartBefore["checkpoint"]["state"]["samples"];
            if (((int?)status["recorded_samples"] ?? -1) < original.Count
                || ((double?)status["recorded_distance_m"] ?? -1) + .01 < (double)restartBefore["checkpoint"]["state"]["recorded_distance"])
                throw new InvalidDataException("Restored map lost samples/distance.");
            bool reference = stage == "restart_reference";
            if (reference && ((string)status["closure"]?["state"] != "complete" || Math.Abs(((double?)status["reference_length_m"] ?? -1) - oldReferenceLength) > .05))
                throw new InvalidDataException("Closed reference did not restore with matching length.");
            if (!reference && ((string)status["phase"] != "learning" || (string)status["closure"]?["state"] == "complete"))
                throw new InvalidDataException("Partial checkpoint did not resume learning.");
            var after = JObject.Parse(File.ReadAllText(semantic.CheckpointPath)); var rows = after["state"]?["samples"] as JArray;
            if (rows == null || rows.Count < original.Count) throw new InvalidDataException("Checkpoint prefix lost.");
            for (int i = 0; i < original.Count; i++) if (!JToken.DeepEquals(original[i], rows[i])) throw new InvalidDataException("Checkpoint prefix changed at " + i);
            var receipt = new JObject { ["kind"] = reference ? "reference" : "partial", ["before_generation"] = oldGeneration, ["after_generation"] = semantic.WorkerGeneration,
                ["before_boot_id"] = oldBoot, ["after_boot_id"] = semantic.WorkerBootId, ["status"] = status.DeepClone(), ["preserved_prefix_samples"] = original.Count,
                ["detector_worker"] = detector.WorkerId, ["lane_epoch"] = detector.LaneEpoch,
                ["position_drift_m"] = Vector3.Distance(drive.Body.position, restartPosition), ["rotation_drift_deg"] = Quaternion.Angle(drive.Body.rotation, restartRotation),
                ["sim_elapsed_sec"] = now - restartSim, ["wall_elapsed_sec"] = EditorApplication.timeSinceStartup - stageAt,
                ["restore_event"] = restoreEvent.DeepClone(), ["shutdown_receipt"] = semantic.ShutdownReceipt?.DeepClone() };
            restartReceipts.Add(receipt); File.WriteAllText(Path.Combine(output, "restart_receipts.json"), restartReceipts.ToString(Formatting.Indented));
            Event("semantic_restart_restored", receipt); if (reference) referenceRestored = true; else partialRestored = true;
            semantic.DiagnosticHold = false; noProgressSince = -1; restoredDistance = travelled;
            SetStage(reference ? "reference_resumed" : "learning_resumed");
        }
        static float FrontGap()
        {
            // Evaluation only. Model bumper bounds, never sent to either controller.
            var p = drive.Body.position; var q = drive.Body.rotation;
            var left = p + q * new Vector3(-.3275f, 0, .216f);
            var right = p + q * new Vector3(.3275f, 0, .216f);
            return SignalPlaneZ - Mathf.Max(left.z, right.z);
        }

        static void Event(string name, JObject detail = null)
        {
            var value = new JObject { ["event"] = name, ["stage"] = stage, ["wall"] = EditorApplication.timeSinceStartup - startWall,
                ["t"] = Time.timeAsDouble - startSim, ["detail"] = detail };
            if (drive) { value["x"] = drive.Body.position.x; value["z"] = drive.Body.position.z; }
            events?.WriteLine(value.ToString(Formatting.None));
        }

        static void WriteStatus()
        {
            File.WriteAllText(Path.Combine(output, "status.json"), JsonConvert.SerializeObject(new {
                phase = finishing ? "stopped_tail" : "running", stage, firstLapClosed, partialRestored, referenceRestored, greenResumed, reason = finishReason, imageCount, detectionCount, motionCount,
                travelled, stoppedForRed, redViolation, detector = detector.Status, semantic = semantic.Status,
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

        static void StopFollowerAndRestoreFocus()
        {
            try {
                if (follower) { follower.Stop(finishReason); if (settingsCaptured) follower.RequireFirstFrameHeadroom = originalFirstFrameHeadroom; }
            } finally {
                // Restore even when Stop throws; this policy belongs only to this trial.
                if (applicationFocusOverridden) {
                    if (follower) follower.RequireApplicationFocus = originalApplicationFocus;
                    applicationFocusOverridden = false;
                }
            }
        }

        static void Complete()
        {
            if (!active) return;
            var cleanupErrors = new List<string>();
            JObject finalShutdownReceipt = null;
            try {
                try { StopFollowerAndRestoreFocus(); } catch (Exception error) { cleanupErrors.Add("follower: " + error.Message); }
                try { if (semantic) semantic.Stop(); } catch (Exception error) { cleanupErrors.Add("semantic: " + error.Message); }
                if (semantic && settingsCaptured) {
                    finalShutdownReceipt = semantic.ShutdownReceipt == null ? null : (JObject)semantic.ShutdownReceipt.DeepClone();
                    if (semantic.WorkerGeneration > initialSemanticGeneration && ((bool?)finalShutdownReceipt?["clean"] != true
                        || (bool?)finalShutdownReceipt?["exited"] != true || (bool?)finalShutdownReceipt?["forced"] != false))
                        cleanupErrors.Add("semantic shutdown/checkpoint close was not confirmed clean");
                }
                try {
                    if (settingsCaptured && semantic) {
                        semantic.DiagnosticHold = originalHold; semantic.EnableSignalApproach = originalApproach;
                        semantic.SourceRoot = originalSourceRoot; semantic.MotorGateSource = originalMotorGate;
                        semantic.CheckpointRoot = originalCheckpointRoot; semantic.CheckpointPath = originalCheckpointPath;
                        semantic.NeoStopWaitMode = originalNeoMode; semantic.WorkerRoot = originalWorkerRoot;
                        semantic.DiagnosticRouteOutput = originalDiagnosticRouteOutput;
                    }
                } catch (Exception error) { cleanupErrors.Add("semantic settings restore: " + error.Message); }
                try {
                    if (detector) { detector.SetRunning(false); if (settingsCaptured) { detector.Configuration.output = originalDetectorOutput; detector.Configuration.frequency = originalDetectorFrequency; detector.Configuration.enableSignalPanelRgb = originalPanelRgb; detector.RecoverProjectedComponents = originalProjectedComponentRecovery; detector.UseCurrentCorridor = originalCurrentCorridor; } }
                } catch (Exception error) { cleanupErrors.Add("detector: " + error.Message); }
                try { if (drive) drive.SetInput(0, 0, true); } catch (Exception error) { cleanupErrors.Add("motor: " + error.Message); }
                if (cleanupErrors.Count > 0) accepted = false;
                File.WriteAllText(Path.Combine(output, "result.json"), JsonConvert.SerializeObject(new {
                    stage, case_kind = (string)request["case_kind"], lane_recovery_mode = (string)request["lane_recovery_mode"], traffic_policy = "stop_wait", distance_enabled = false, bbox_enabled = false, automatic_detour_enabled = false,
                    firstLapClosed, partialRestored, referenceRestored, greenResumed, learnedStatus, restartReceipts,
                    firstRedConfirmation, firstGreenConfirmation,
                    nativeLyaParametersChanged = false, runtimeFirstFrameHeadroom = true, movingRedBrakingDistanceValidated = false,
                    accepted, reason = finishReason, @case = (string)request["case"], imageCount, detectionCount, motionCount, supportedCount,
                    travelled, stoppedForRed, redViolation,
                    stoppedFrontGapEvaluationOnly = Finite(stoppedFrontGap) ? (float?)stoppedFrontGap : null,
                    green_t = greenAt < 0 ? (double?)null : greenAt - startSim,
                    seconds = Time.timeAsDouble - startSim, wallSeconds = EditorApplication.timeSinceStartup - startWall,
                    captureHz = 15, motionHz = 20, framesUseActualTimestamps = true, signalPlaneZ = SignalPlaneZ,
                    initialPoseChangedOnce = true, articulatedBodiesMovedTogether = true,
                    signalTruthFedToControl = false, evaluatorRouteFedToControl = false,
                    odometrySource = "Unity Rigidbody pose and velocity; ideal noiseless simulated odometry",
                    physicsParametersChanged = false, cameraIntrinsicsChanged = false,
                    detectorWorker = detector ? detector.WorkerId : 0, semanticWorker = semantic ? semantic.WorkerId : 0,
                    cleanupErrors, finalShutdownReceipt, terminalBodySpeed = drive ? drive.Body.velocity.magnitude : (float?)null,
                    evidence = "Live Unity Play, genuine RGB model output, native LYA and semantic result. Main video uses an observer-camera clone. Remains in Play with E-STOP."
                }, Formatting.Indented));
                File.WriteAllText(Path.Combine(output, "play_after.json"), Configuration(drive).ToString(Formatting.Indented));
            } finally {
                trace?.Dispose(); motion?.Dispose(); frames?.Dispose(); events?.Dispose(); semanticTicks?.Dispose(); semanticTicks = null;
                trace = motion = frames = events = null;
                if (target) { target.Release(); Object.DestroyImmediate(target); }
                if (pixels) Object.DestroyImmediate(pixels);
                if (capture) Object.DestroyImmediate(capture.gameObject);
                target = null; pixels = null; capture = null; active = false;
                SessionState.EraseString(Key);
            }
            Debug.Log("NEO_TOPO_FINISHED " + finishReason + " " + output);
        }

        static StreamWriter Writer(string name) => new StreamWriter(Path.Combine(output, name)) { AutoFlush = true };
        static JArray Vector(Vector3 value) => new JArray(value.x, value.y, value.z);
        static JArray Rotation(Quaternion value) => new JArray(value.x, value.y, value.z, value.w);
        static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
