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
    // 独立诊断：真实绿灯学习；可选同一链路第二圈真实红灯 detour 观察，不改变 Runtime/core。
    // 位置和灯色真值仅用于试验初始化、评测与录像；不进入语义规划或 LYA。
    [InitializeOnLoad]
    public static class AIFormulaSemanticLeftTrial
    {
        const string Key = "AIFormula.SemanticLeftTrial";
        const double CapturePeriod = 1.0 / 15;
        const double MotionPeriod = 1.0 / 20;
        const float SignalPlaneZ = 15;
        const float ApproachHalfWidth = 4, ApproachHalfDepth = 3, ApproachHeadingTolerance = 30;
        const float LeftYawEvidenceDeg = 45, LeftDisplacementEvidenceM = 1;
        static AIFormulaDifferentialDrive drive;
        static AIFormulaVisionLaneFollower follower;
        static AIFormulaRoadDetector detector;
        static AIFormulaSemanticPlanning semantic;
        static TrainingFlagPanel signal;
        static Camera source, capture;
        static RenderTexture target;
        static Texture2D pixels;
        static StreamWriter trace, motion, frames, events;
        static StreamWriter semanticTicks;
        static JObject request, lastPerception;
        static string output, overlayPath, rgbPath, finishReason;
        static double startWall, startSim, nextImage, nextMotion, nextStatus;
        static double movementStart, waitStart, greenAt, finishAt, tailWallDeadline;
        static double noProgressSince, learnedAt;
        static float noProgressDistance;
        static bool firstLapLearned;
        static JObject learnedStatus;
        static bool continueAfterLearning, redScheduled, detourEntered, detourCompleted, leftTurnObserved;
        static double redAt, detourAt, detourExitAt;
        static long observedSemanticSequence;
        static int semanticSampleCount, semanticSequenceGaps;
        static string previousTrafficMode, previousTrafficReason, previousSemanticPhase;
        static int chainSemanticWorker, chainDetectorWorker, chainLaneEpoch, chainSemanticEpoch;
        static bool chainCaptured;
        static Vector3 detourOrigin, detourRight;
        static float previousDetourYaw, signedLeftYaw, maximumLeftYaw, maximumLeftDisplacement;
        static JObject detourEntryStatus, detourExitStatus;
        static long sequence;
        static int imageCount, detectionCount, motionCount, supportedCount;
        static float travelled, minimumStopZ;
        static float stoppedFrontGap;
        static bool redStopInTarget;
        static Vector3 lastPosition;
        static bool active, finishing, accepted, stoppedForRed, crossedSignal, redViolation, semanticReadySeen;
        static int originalDetectorOutput;
        static float originalDetectorFrequency;
        static bool originalPanelRgb;
        static bool originalProjectedComponentRecovery;
        static bool originalCurrentCorridor;
        static bool originalCaptureMetadata;
        static bool captureMetadataOverridden;
        static bool originalFirstFrameHeadroom;
        static bool originalApplicationFocus, applicationFocusOverridden;
        static string originalDiagnosticRouteOutput;
        static bool diagnosticRouteOutputOverridden;

        static AIFormulaSemanticLeftTrial()
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
                                   bool initialRed, bool releaseGreen, float perceptionHz = 5, bool panelRgb = false,
                                   bool signalApproach = false, bool projectedRecovery = true, bool currentCorridor = true,
                                   bool continueAfterLearning = false, bool captureMetadata = false)
        {
            var scene = SceneManager.GetActiveScene();
            if (Application.isBatchMode || EditorApplication.isPlayingOrWillChangePlaymode || scene.isDirty || active)
                throw new InvalidOperationException("Requires clean GUI Edit Mode without another traffic trial.");
            if (!scene.path.EndsWith("/MotorControllerValidation.unity", StringComparison.Ordinal))
                throw new InvalidOperationException("Open the saved MotorControllerValidation scene first.");
            if (!Finite(laneX) || !Finite(startZ) || string.IsNullOrWhiteSpace(caseName))
                throw new ArgumentException("Finite start coordinates and a case name are required.");
            if (initialRed)
                throw new ArgumentException("This diagnostic stage requires initialRed=false for genuine green first-lap learning.");
            if (continueAfterLearning && releaseGreen)
                throw new ArgumentException("The continuation diagnostic keeps the actual panel red; releaseGreen must be false.");
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
                ["signal_approach"] = signalApproach,
                ["automated_background_focus_opt_out"] = signalApproach,
                ["recover_projected_components"] = projectedRecovery,
                ["use_current_corridor"] = currentCorridor,
                ["continue_after_learning"] = continueAfterLearning,
                ["capture_metadata"] = captureMetadata,
                ["diagnostic_route_output"] = Path.Combine(directory, "route_fusion_exact.json"),
                ["route_export_role"] = "Read-only actual fusion-call inputs/results from this same worker; no route injection or control change",
                ["lane_recovery_role"] = "Opt-in Python adapter current-component projection; native upstream LLP and LYA are unchanged",
                ["diagnostic_stage"] = continueAfterLearning ? "same_chain_learning_then_red_detour_observation" : "green_full_lap_learning_only",
                ["requested_front_gap_m"] = null,
                ["front_geometry"] = "FBX front rubber corners: local x +/-0.3275 m, z +0.216 m; simulation model measurement",
                ["scene"] = scene.path, ["requested_utc"] = DateTime.UtcNow.ToString("o"),
                ["requested_editor_wall"] = EditorApplication.timeSinceStartup,
                ["signal_plane_z"] = SignalPlaneZ,
                ["finish_region"] = continueAfterLearning
                    ? "Actual first-lap reference, scheduled real red, observed detour entry and explicit completion, plus measured leftward motion; missing evidence fails"
                    : "Stop when semantic phase is guarding/odom_only with a finite positive reference_length_m; fusion result reported independently",
                ["second_approach_trigger"] = new JObject { ["enabled"] = continueAfterLearning,
                    ["x_center"] = laneX, ["x_half_width_m"] = ApproachHalfWidth,
                    ["z_center"] = startZ, ["z_half_depth_m"] = ApproachHalfDepth,
                    ["north_heading_tolerance_deg"] = ApproachHeadingTolerance,
                    ["requires_first_green_pass_and_reference"] = true,
                    ["role"] = "Evaluation-only real panel color schedule; never sent as control pose, route or light observation" },
                ["left_motion_evidence"] = new JObject { ["min_left_yaw_deg"] = LeftYawEvidenceDeg,
                    ["min_left_displacement_m"] = LeftDisplacementEvidenceM,
                    ["role"] = "Measured Rigidbody motion during the observed detour; not proof of lane safety or successful route exit by itself" },
                ["evaluation_bounds"] = new JObject { ["xmin"] = -25, ["xmax"] = 90, ["zmin"] = -75, ["zmax"] = 55 },
                ["wall_timeout_sec"] = 420, ["motion_timeout_sec"] = 240,
                ["distance_limit_m"] = 600, ["stopped_no_progress_timeout_sec"] = 15,
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
            captureMetadataOverridden = false;
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
            continueAfterLearning = (bool?)request["continue_after_learning"] ?? false;
            semanticTicks = continueAfterLearning ? Writer("semantic_ticks.jsonl") : null;
            diagnosticRouteOutputOverridden = false;
            active = true; finishing = accepted = stoppedForRed = crossedSignal = redViolation = semanticReadySeen = false;
            firstLapLearned = false; learnedStatus = null; learnedAt = noProgressSince = -1; noProgressDistance = 0;
            redScheduled = detourEntered = detourCompleted = leftTurnObserved = chainCaptured = false;
            redAt = detourAt = detourExitAt = -1; observedSemanticSequence = 0;
            semanticSampleCount = semanticSequenceGaps = 0;
            previousTrafficMode = previousTrafficReason = previousSemanticPhase = null;
            chainSemanticWorker = chainDetectorWorker = chainLaneEpoch = chainSemanticEpoch = -1;
            detourEntryStatus = detourExitStatus = null;
            signedLeftYaw = maximumLeftYaw = maximumLeftDisplacement = 0;
            redStopInTarget = false; stoppedFrontGap = float.NaN;
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
            var holder = new GameObject("Semantic first-lap read-only observer") { hideFlags = HideFlags.HideAndDontSave };
            capture = holder.AddComponent<Camera>(); capture.enabled = false;
            target = new RenderTexture(1280, 720, 24, RenderTextureFormat.ARGB32) { hideFlags = HideFlags.HideAndDontSave };
            target.Create();
            pixels = new Texture2D(1280, 720, TextureFormat.RGB24, false) { hideFlags = HideFlags.HideAndDontSave };
            panel.SetVisible(false);
            detector.Initialize(); originalDetectorOutput = detector.Configuration.output;
            originalDetectorFrequency = detector.Configuration.frequency;
            originalPanelRgb = detector.Configuration.enableSignalPanelRgb;
            originalProjectedComponentRecovery = detector.RecoverProjectedComponents;
            originalCurrentCorridor = detector.UseCurrentCorridor;
            originalCaptureMetadata = detector.EnableJunctionCaptureMetadata;
            captureMetadataOverridden = true;
            detector.EnableJunctionCaptureMetadata = (bool?)request["capture_metadata"] ?? false;
            detector.Configuration.frequency = (float?)request["perception_hz"] ?? 5;
            detector.Configuration.enableSignalPanelRgb = (bool?)request["signal_panel_rgb"] ?? false;
            detector.RecoverProjectedComponents = (bool?)request["recover_projected_components"] ?? false;
            detector.UseCurrentCorridor = (bool?)request["use_current_corridor"] ?? true;
            AIFormulaRoadDetector.Validate(detector.Configuration);
            detector.Configuration.output = 0; // 仅本次 Play 选择既有识别 overlay，不改持久设置。
            File.WriteAllText(Path.Combine(output, "perception_settings.json"), JsonConvert.SerializeObject(detector.Configuration, Formatting.Indented));
            var calibration = typeof(AIFormulaRoadDetector).GetMethod("CameraCalibration", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);
            File.WriteAllText(Path.Combine(output, "camera_calibration.json"), JsonConvert.SerializeObject(calibration.Invoke(detector, null), Formatting.Indented));
            detector.SetRunning(true);
            semantic.EnableSignalApproach = (bool?)request["signal_approach"] ?? false;
            originalDiagnosticRouteOutput = semantic.DiagnosticRouteOutput;
            semantic.DiagnosticRouteOutput = Path.Combine(output, "route_fusion_exact.json");
            diagnosticRouteOutputOverridden = true;
            semantic.Begin();
            originalFirstFrameHeadroom = follower.RequireFirstFrameHeadroom;
            follower.RequireFirstFrameHeadroom = (bool?)request["signal_approach"] ?? false;
            ConfigureFollowerFocus((bool?)request["signal_approach"] ?? false);
            Event("follower_focus_policy", new JObject { ["required"] = follower.RequireApplicationFocus, ["automated_override"] = applicationFocusOverridden });
            follower.Begin();
            Debug.Log("SEMANTIC_LEFT_STARTED " + output);
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
            if ((bool?)request["capture_metadata"] == true && lastPerception?["lane"] != null) {
                // Preserve the exact same-inference encoded mask, before UI decode/re-encoding.
                byte[] packet = Convert.FromBase64String((string)lastPerception["lane"]);
                string packetName = "lane_packet_" + detectionCount.ToString("D5") + ".png";
                File.WriteAllBytes(Path.Combine(output, packetName), packet);
                lastPerception["lane_packet_file"] = packetName;
                using(var hash=System.Security.Cryptography.SHA256.Create())
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

        static JObject Sample(string kind)
        {
            var position = drive.Body.position;
            var perception = lastPerception;
            return new JObject {
                ["kind"] = kind, ["wall"] = EditorApplication.timeSinceStartup - startWall,
                ["t"] = Time.timeAsDouble - startSim, ["sequence"] = detector.FrameSequence,
                ["frame_sim_time"] = detector.FrameSimulationTime, ["epoch"] = detector.LaneEpoch,
                ["absolute_sim_time"] = Time.timeAsDouble, ["start_sim_time"] = startSim,
                ["junction_capture"] = perception?["junction_capture"]?.DeepClone(),
                ["lane_packet_file"] = perception?["lane_packet_file"], ["lane_packet_sha256"] = perception?["lane_packet_sha256"],
                ["rgb_input_sha256"] = perception?["input_sha256"],
                ["x"] = position.x, ["z"] = position.z, ["yaw"] = drive.Body.rotation.eulerAngles.y,
                ["actual_v"] = Vector3.Dot(drive.Body.velocity, drive.Body.rotation * Vector3.forward),
                ["actual_w"] = -drive.Body.angularVelocity.y,
                ["applied_v"] = drive.RequestedVelocity.x, ["applied_w"] = -drive.RequestedVelocity.y,
                ["signal_truth"] = signal.State.ToString().ToLowerInvariant(), ["detected_light"] = detector.TrafficLight,
                ["detected_light_original"] = perception?["traffic_light_original"],
                ["detected_light_source"] = perception?["traffic_light_source"],
                ["signal_panel"] = perception?["signal_panel"]?.DeepClone(),
                ["signal_range"] = perception?["signal_range"]?.DeepClone(),
                ["actual_front_gap_m"] = FrontGap(),
                ["detections"] = perception?["detections"]?.DeepClone(),
                ["semantic"] = semantic.Result?.DeepClone(), ["candidate"] = detector.LaneControl?.DeepClone(),
                ["semantic_running"] = semantic.Running, ["semantic_runtime_status"] = semantic.Status,
                ["semantic_worker"] = semantic.WorkerId, ["detector_worker"] = detector.WorkerId,
                ["diagnostic_stage"] = !continueAfterLearning ? "green_full_lap_learning_only"
                    : !firstLapLearned ? "learning" : !redScheduled ? "await_second_east_approach"
                    : !detourEntered ? "red_await_detour" : !detourCompleted ? "detour_observation" : "detour_exit_observed",
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
            if (wallElapsed > 420) { BeginFinish("Wall-time watchdog after 420 seconds", false); return; }
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
            var expectedSignal = redScheduled ? TrainingFlagPanel.SignalState.Red : TrainingFlagPanel.SignalState.Green;
            if (signal.State != expectedSignal) {
                BeginFinish("Actual signal changed outside the diagnostic schedule", false); return;
            }
            if (p.z > SignalPlaneZ && !crossedSignal) {
                crossedSignal = true; Event("green_signal_passed");
            }
            // 仅评测用全圈 gross bounds；不提供控制路线或精确道路边界。
            if (!Finite(p.x) || !Finite(p.y) || !Finite(p.z) || p.x < -25 || p.x > 90 || p.z < -75 || p.z > 55
                || p.y < -1 || p.y > 3 || Vector3.Dot(drive.Body.rotation * Vector3.up, Vector3.up) < .7f) {
                BeginFinish("CRITICAL: vehicle left the full-lap diagnostic bounds or overturned", false); return;
            }
            var status = semantic.Result?["status"] as JObject;
            string phase = (string)status?["phase"];
            double referenceLength = (double?)status?["reference_length_m"] ?? double.NaN;
            if (!firstLapLearned && (phase == "guarding" || phase == "odom_only") && !double.IsNaN(referenceLength)
                && !double.IsInfinity(referenceLength) && referenceLength > 0) {
                firstLapLearned = true; learnedAt = now; learnedStatus = (JObject)status.DeepClone();
                Event("first_lap_learned", new JObject {
                    ["semantic_phase"] = phase, ["reference_length_m"] = referenceLength,
                    ["map_fusion"] = status["map_fusion"]?.DeepClone(),
                    ["topology_mapping"] = status["topology_mapping"]?.DeepClone(),
                    ["route_matched"] = status["route_matched"]?.DeepClone(),
                    ["left_turn_validated"] = false });
                if (!continueAfterLearning) { BeginFinish("first_lap_learned", true); return; }
            }
            if (continueAfterLearning) {
                ObserveContinuation(now, status);
                if (finishing) return;
            }
            if (travelled >= 600) { BeginFinish("Diagnostic exceeded 600 m before completion", false); return; }
            if (movementStart >= 0 && Mathf.Abs(speed) < .05f) {
                if (noProgressSince < 0 || travelled - noProgressDistance >= .1f) {
                    noProgressSince = now; noProgressDistance = travelled;
                }
                if (now - noProgressSince >= 15) {
                    Event("stopped_no_progress", new JObject { ["stationary_seconds"] = now - noProgressSince,
                        ["distance_since_window_start_m"] = travelled - noProgressDistance });
                    BeginFinish("Stopped without 0.1 m progress for 15 simulation seconds", false); return;
                }
            } else noProgressSince = -1;
            if (movementStart >= 0 && now - movementStart > 240) { BeginFinish("Motion timeout after 240 simulation seconds", false); return; }
            if (movementStart < 0 && wallElapsed > 100) BeginFinish("No motion within 100 wall seconds", false);
        }

        static void ObserveContinuation(double now, JObject status)
        {
            var result = semantic.Result;
            if (result == null || status == null) return;
            int resultEpoch = (int?)result["epoch"] ?? -1;
            if (!chainCaptured && resultEpoch >= 0) {
                chainCaptured = true; chainSemanticWorker = semantic.WorkerId; chainDetectorWorker = detector.WorkerId;
                chainLaneEpoch = detector.LaneEpoch; chainSemanticEpoch = resultEpoch;
                Event("same_chain_identity_captured", new JObject { ["semantic_worker"] = chainSemanticWorker,
                    ["detector_worker"] = chainDetectorWorker, ["lane_epoch"] = chainLaneEpoch, ["semantic_epoch"] = chainSemanticEpoch });
            }
            if (chainCaptured && (semantic.WorkerId != chainSemanticWorker || detector.WorkerId != chainDetectorWorker
                || detector.LaneEpoch != chainLaneEpoch || resultEpoch != chainSemanticEpoch)) {
                BeginFinish("Worker/epoch changed; same-chain learning continuation is invalid", false); return;
            }
            long tickSequence = (long?)result["sequence"] ?? 0;
            string mode = (string)status["traffic_mode"], reason = (string)status["traffic_reason"];
            string phase = (string)status["phase"];
            if (tickSequence != observedSemanticSequence) {
                if (tickSequence <= observedSemanticSequence) {
                    BeginFinish("Semantic response sequence rolled back during continuation", false); return;
                }
                if (observedSemanticSequence > 0 && tickSequence > observedSemanticSequence + 1) {
                    semanticSequenceGaps++;
                    Event("semantic_response_sampling_gap", new JObject { ["previous"] = observedSemanticSequence, ["current"] = tickSequence });
                }
                observedSemanticSequence = tickSequence;
                var sample = Sample("semantic_response"); sample["entry_diagnostics"] = EntryDiagnostics(status);
                semanticTicks.WriteLine(sample.ToString(Formatting.None)); semanticSampleCount++;
                if (mode != previousTrafficMode || reason != previousTrafficReason || phase != previousSemanticPhase) {
                    Event("semantic_transition", new JObject { ["sequence"] = tickSequence,
                        ["previous_mode"] = previousTrafficMode, ["previous_reason"] = previousTrafficReason,
                        ["status"] = status.DeepClone(), ["entry_diagnostics"] = EntryDiagnostics(status) });
                    previousTrafficMode = mode; previousTrafficReason = reason; previousSemanticPhase = phase;
                }
            }
            if (!firstLapLearned) return;
            if (phase != "guarding" && phase != "odom_only") {
                BeginFinish("Learned reference disappeared during same-chain continuation", false); return;
            }
            var p = drive.Body.position;
            float yaw = drive.Body.rotation.eulerAngles.y;
            bool northFacing = Mathf.Abs(Mathf.DeltaAngle(0, yaw)) <= ApproachHeadingTolerance;
            bool eastApproach = Mathf.Abs(p.x - (float)request["lane_x"]) <= ApproachHalfWidth
                && Mathf.Abs(p.z - (float)request["start_z"]) <= ApproachHalfDepth && northFacing;
            if (!redScheduled && mode == "detour") {
                BeginFinish("Detour observed before the scheduled second-approach red", false); return;
            }
            if (!redScheduled && crossedSignal && eastApproach && chainCaptured) {
                // Only the physical renderer changes. The next genuine camera frames carry the new color.
                signal.SetState(TrainingFlagPanel.SignalState.Red); redScheduled = true; redAt = now;
                Event("second_approach_panel_red", new JObject { ["status_before_switch"] = status.DeepClone(),
                    ["entry_diagnostics"] = EntryDiagnostics(status), ["actual_front_gap_m"] = FrontGap(),
                    ["signal_position"] = Vector(signal.transform.position), ["signal_rotation"] = Rotation(signal.transform.rotation) });
            }
            if (!redScheduled) return;
            if (!detourEntered && mode == "detour") {
                var nodes = status["detour_node_ids"] as JArray;
                if (nodes == null || nodes.Count < 2) {
                    BeginFinish("Detour mode has no recorded node route", false); return;
                }
                detourEntered = true; detourAt = now; detourEntryStatus = (JObject)status.DeepClone();
                detourOrigin = p; detourRight = drive.Body.rotation * Vector3.right; previousDetourYaw = yaw;
                Event("detour_entered", new JObject { ["status"] = status.DeepClone(), ["entry_diagnostics"] = EntryDiagnostics(status),
                    ["semantic_sequence"] = tickSequence, ["pose_yaw_deg"] = yaw });
            }
            if (detourEntered && !detourCompleted) {
                // Unity positive yaw turns right. These evaluation metrics are never used to steer.
                signedLeftYaw -= Mathf.DeltaAngle(previousDetourYaw, yaw); previousDetourYaw = yaw;
                maximumLeftYaw = Mathf.Max(maximumLeftYaw, signedLeftYaw);
                maximumLeftDisplacement = Mathf.Max(maximumLeftDisplacement, -Vector3.Dot(p - detourOrigin, detourRight));
                if (!leftTurnObserved && signedLeftYaw >= LeftYawEvidenceDeg
                    && -Vector3.Dot(p - detourOrigin, detourRight) >= LeftDisplacementEvidenceM) {
                    leftTurnObserved = true;
                    Event("leftward_motion_observed", new JObject { ["signed_left_yaw_deg"] = signedLeftYaw,
                        ["left_displacement_m"] = -Vector3.Dot(p - detourOrigin, detourRight), ["semantic_status"] = status.DeepClone() });
                }
                if (mode != "detour") {
                    bool explicitCompletion = reason == "detour_complete_resume_vision" || reason == "red_after_detour";
                    if (!explicitCompletion) {
                        BeginFinish("Detour exited without an observed explicit completion: " + reason, false); return;
                    }
                    detourCompleted = true; detourExitAt = now; detourExitStatus = (JObject)status.DeepClone();
                    Event("detour_explicit_completion", new JObject { ["status"] = status.DeepClone(),
                        ["leftward_motion_observed"] = leftTurnObserved, ["maximum_left_yaw_deg"] = maximumLeftYaw,
                        ["maximum_left_displacement_m"] = maximumLeftDisplacement });
                    BeginFinish(leftTurnObserved ? "detour_completed_with_observed_leftward_motion"
                        : "Detour completed without the required measured leftward motion evidence", leftTurnObserved);
                    return;
                }
            }
            float speed = Vector3.Dot(drive.Body.velocity, drive.Body.rotation * Vector3.forward);
            if (mode == "wait" && Mathf.Abs(speed) < .05f) {
                if (waitStart < 0) waitStart = now;
                if (!stoppedForRed && now - waitStart >= 3) {
                    stoppedForRed = true; minimumStopZ = p.z; stoppedFrontGap = FrontGap();
                    redStopInTarget = stoppedFrontGap >= 8 && stoppedFrontGap <= 9;
                    Event("red_wait_stationary_three_seconds", new JObject { ["actual_front_gap_m"] = stoppedFrontGap,
                        ["status"] = status.DeepClone(), ["entry_diagnostics"] = EntryDiagnostics(status) });
                }
            } else waitStart = -1;
            if (!detourEntered && northFacing && Mathf.Abs(p.x - (float)request["lane_x"]) <= ApproachHalfWidth && FrontGap() < 0) {
                redViolation = true; BeginFinish("CRITICAL: crossed the original east signal plane on red before detour entry", false);
            }
        }

        static JObject EntryDiagnostics(JObject status)
        {
            // Compare current evaluation pose with published fused nodes, never supply any of this to control.
            // The current pose can be newer than status; semantic_result_age records that sampling offset.
            var nodes = status?["official_node_positions_odom"] as JObject;
            var progressByNode = status?["official_node_progress_m"] as JObject;
            var route = status?["official_route_node_ids"] as JArray;
            double length = (double?)status?["reference_length_m"] ?? double.NaN;
            double progress = (double?)status?["progress_m"] ?? double.NaN;
            var candidates = new JArray(); string nearest = null; double nearestDistance = double.PositiveInfinity;
            if (nodes != null && route != null) for (int i = 0; i < route.Count - 1; i++) {
                string id = (string)route[i]; var xy = nodes[id] as JArray;
                if (xy == null || xy.Count < 2) continue;
                double x = (double)xy[0], y = (double)xy[1];
                double distance = Math.Sqrt(Math.Pow(drive.Body.position.z - x, 2) + Math.Pow(-drive.Body.position.x - y, 2));
                double nodeProgress = (double?)progressByNode?[id] ?? double.NaN;
                double delta = nodeProgress - progress;
                if (!double.IsNaN(delta) && !double.IsInfinity(delta) && length > 0 && !double.IsInfinity(length))
                    delta -= Math.Floor((delta + length * .5) / length) * length;
                else delta = double.NaN;
                if (distance < nearestDistance) { nearestDistance = distance; nearest = id; }
                candidates.Add(new JObject { ["node_id"] = id, ["current_pose_distance_m"] = distance,
                    ["cyclic_progress_delta_m"] = double.IsNaN(delta) ? (double?)null : delta,
                    ["within_default_entry_radius_3m"] = distance <= 3,
                    ["within_default_progress_window_4m"] = !double.IsNaN(delta) && Math.Abs(delta) <= 4 });
            }
            return new JObject { ["evaluation_only"] = true,
                ["pose_source"] = "Current Unity Rigidbody evaluated as ROS odom (z,-x); may be newer than the sampled status",
                ["semantic_result_age_sec"] = semantic.ResultAge,
                ["default_core_entry_radius_m"] = 3, ["default_core_progress_tolerance_m"] = 1,
                ["nearest_published_outer_node"] = nearest,
                ["nearest_distance_m"] = double.IsPositiveInfinity(nearestDistance) ? (double?)null : nearestDistance,
                ["nodes"] = candidates, ["map_fusion"] = status?["map_fusion"]?.DeepClone(),
                ["topology_mapping"] = status?["topology_mapping"]?.DeepClone(),
                ["route_matched"] = status?["route_matched"]?.DeepClone(),
                ["traffic_reason"] = status?["traffic_reason"]?.DeepClone(),
                ["signal_approach"] = status?["signal_approach"]?.DeepClone() };
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
                continueAfterLearning, firstLapLearned, redScheduled, detourEntered, detourCompleted, leftTurnObserved,
                semanticSampleCount, semanticSequenceGaps,
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
            if (drive && detector && semantic && follower && signal) {
                var terminal = Sample("terminal");
                if (continueAfterLearning) terminal["entry_diagnostics"] = EntryDiagnostics(semantic.Result?["status"] as JObject);
                File.WriteAllText(Path.Combine(output, "terminal_sample.json"), terminal.ToString(Formatting.Indented));
            }
            if (follower) follower.Stop(reason);
            if (drive) drive.SetInput(0, 0, true);
            var driver = drive ? drive.GetComponent<AIFormulaDriverInput>() : null;
            if (driver) driver.SetEmergencyStop(true);
            WriteStatus();
        }

        static void ConfigureFollowerFocus(bool signalApproach)
        {
            if (!signalApproach) return;
            originalApplicationFocus = follower.RequireApplicationFocus;
            applicationFocusOverridden = true;
            follower.RequireApplicationFocus = false;
        }

        static void StopFollowerAndRestoreFocus()
        {
            try {
                if (follower) { follower.Stop(finishReason); follower.RequireFirstFrameHeadroom = originalFirstFrameHeadroom; }
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
            try {
                try { StopFollowerAndRestoreFocus(); } catch (Exception error) { cleanupErrors.Add("follower: " + error.Message); }
                try { if (semantic) semantic.Stop(); } catch (Exception error) { cleanupErrors.Add("semantic: " + error.Message); }
                finally {
                    try {
                        if (diagnosticRouteOutputOverridden && semantic)
                            semantic.DiagnosticRouteOutput = originalDiagnosticRouteOutput;
                    } catch (Exception error) { cleanupErrors.Add("semantic diagnostic output restore: " + error.Message); }
                    finally { diagnosticRouteOutputOverridden = false; }
                }
                try {
                    if (detector) { detector.SetRunning(false); detector.Configuration.output = originalDetectorOutput; detector.Configuration.frequency = originalDetectorFrequency; detector.Configuration.enableSignalPanelRgb = originalPanelRgb; detector.RecoverProjectedComponents = originalProjectedComponentRecovery; detector.UseCurrentCorridor = originalCurrentCorridor; }
                } catch (Exception error) { cleanupErrors.Add("detector: " + error.Message); }
                finally {
                    try {
                        if (captureMetadataOverridden && detector)
                            detector.EnableJunctionCaptureMetadata = originalCaptureMetadata;
                    } catch (Exception error) { cleanupErrors.Add("capture metadata restore: " + error.Message); }
                    finally { captureMetadataOverridden = false; }
                }
                try { if (drive) drive.SetInput(0, 0, true); } catch (Exception error) { cleanupErrors.Add("motor: " + error.Message); }
                if (cleanupErrors.Count > 0) accepted = false;
                File.WriteAllText(Path.Combine(output, "result.json"), JsonConvert.SerializeObject(new {
                    accepted, reason = finishReason, @case = (string)request["case"], imageCount, detectionCount, motionCount, supportedCount,
                    diagnosticStage = (string)request["diagnostic_stage"], firstLapLearned,
                    firstLapLearnedTime = learnedAt < 0 ? (double?)null : learnedAt - startSim,
                    learnedStatus, leftTurnValidated = false,
                    continueAfterLearning, redScheduled, detourEntered, detourCompleted, leftTurnObserved,
                    redScheduledTime = redAt < 0 ? (double?)null : redAt - startSim,
                    detourEntryTime = detourAt < 0 ? (double?)null : detourAt - startSim,
                    detourExitTime = detourExitAt < 0 ? (double?)null : detourExitAt - startSim,
                    detourEntryStatus, detourExitStatus, maximumLeftYaw, maximumLeftDisplacement,
                    sameChainIdentityCaptured = chainCaptured, chainSemanticWorker, chainDetectorWorker, chainLaneEpoch, chainSemanticEpoch,
                    semanticSampleCount, semanticSequenceGaps,
                    acceptanceScope = continueAfterLearning
                        ? "Bounded diagnostic: same live chain learned, actual red scheduled, detour nodes/explicit completion and measured leftward motion observed. Lane/obstacle safety and video review remain separate."
                        : "First-lap reference learned; fusion reported independently; no left-turn validation",
                    leftMotionEvidenceThresholds = new { yawDegrees = LeftYawEvidenceDeg, lateralMeters = LeftDisplacementEvidenceM },
                    travelled, stoppedForRed, crossedSignal, redViolation,
                    stop_z = float.IsPositiveInfinity(minimumStopZ) ? (float?)null : minimumStopZ,
                    stoppedFrontGap = Finite(stoppedFrontGap) ? (float?)stoppedFrontGap : null, redStopInTarget,
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
                trace?.Dispose(); motion?.Dispose(); frames?.Dispose(); events?.Dispose(); semanticTicks?.Dispose();
                trace = motion = frames = events = null;
                semanticTicks = null;
                if (target) { target.Release(); Object.DestroyImmediate(target); }
                if (pixels) Object.DestroyImmediate(pixels);
                if (capture) Object.DestroyImmediate(capture.gameObject);
                target = null; pixels = null; capture = null; active = false;
                SessionState.EraseString(Key);
            }
            Debug.Log("SEMANTIC_LEFT_FINISHED " + finishReason + " " + output);
        }

        static StreamWriter Writer(string name) => new StreamWriter(Path.Combine(output, name)) { AutoFlush = true };
        static JArray Vector(Vector3 value) => new JArray(value.x, value.y, value.z);
        static JArray Rotation(Quaternion value) => new JArray(value.x, value.y, value.z, value.w);
        static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
