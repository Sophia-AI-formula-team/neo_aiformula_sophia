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
    // Physical pose and lamp scheduling are evaluation-only. The worker receives camera results and odometry.
    [InitializeOnLoad]
    public static class AIFormulaJointTopoTrial
    {
        const string Key = "AIFormula.JointTopoTrial1115";
        static AIFormulaDifferentialDrive drive;
        static AIFormulaVisionLaneFollower follower;
        static AIFormulaRoadDetector detector;
        static AIFormulaSemanticPlanning semantic;
        static TrainingFlagPanel signal;
        static StreamWriter trace, motion, frames, events, semanticTicks;
        static JObject request, lastPerception, learnedStatus, terminalStatus;
        static string output, overlayPath, rgbPath, finishReason, stage, semanticIdentity, redFailureReason;
        static double startWall, startSim, nextImage, nextMotion, nextStatus, finishAt, tailDeadline;
        static double redAt, waitAt, noProgressAt, motionControlStartedAt;
        static float travelled, lastProgressDistance, firstLapDistance, lastLapDistance, leftYaw, maxLeftYaw, maxLeftDisplacement, previousYaw;
        static Vector3 lastPosition, detourOrigin, detourRight;
        static int imageCount, detectionCount, motionCount, completedLaps, stableDetector, stableEpoch, stableSemantic;
        static long sequence;
        static bool active, finishing, accepted, firstLapClosed, lapArmed, redScheduled, redObserved, detourEntered, detourCompleted, leftObserved;
        static bool redDetourFailed, recoveryGreenScheduled;
        static bool motionControlStarted;
        static bool settingsCaptured, originalFocus, originalHeadroom, originalHold, originalApproach, originalNeo, originalJoint, originalPanelRgb;
        static bool originalProjectedRecovery, originalCurrentCorridor;
        static bool originalRejectConflictingBelowHorizonLights;
        static int originalOutput;
        static string originalWorkerRoot, originalSourceRoot, originalJointOutput, originalDiagnostic;
        static TrainingFlagPanel.SignalState originalSignal;

        static AIFormulaJointTopoTrial()
        {
            EditorApplication.update += Tick;
            EditorApplication.playModeStateChanged += state => {
                if (state == PlayModeStateChange.ExitingPlayMode && active) { BeginFinish("Play interrupted", false); Complete(); }
            };
            AssemblyReloadEvents.beforeAssemblyReload += () => {
                if (active) { BeginFinish("Assembly reload interrupted capture", false); Complete(); }
            };
        }

        public static string Start(string directory, string caseName, float laneX, float startZ,
                                   string sourceRoot, string workerRoot, double wallTimeout = 900, bool panelRgb = true, string laneRecoveryMode = "baseline")
        {
            var scene = SceneManager.GetActiveScene();
            if (Application.isBatchMode || EditorApplication.isPlayingOrWillChangePlaymode || scene.isDirty || active)
                throw new InvalidOperationException("Requires exclusive clean GUI Edit Mode.");
            if (!scene.path.EndsWith("/MotorControllerValidation.unity", StringComparison.Ordinal))
                throw new InvalidOperationException("Open saved MotorControllerValidation scene first.");
            if (!Finite(laneX) || !Finite(startZ) || double.IsNaN(wallTimeout) || double.IsInfinity(wallTimeout) || wallTimeout < 60 || wallTimeout > 3600)
                throw new ArgumentException("Finite pose and bounded 60-3600 s wall timeout required.");
            if (laneRecoveryMode != "baseline" && laneRecoveryMode != "component-only") throw new ArgumentException("Unknown lane recovery mode.");
            if (string.IsNullOrWhiteSpace(caseName) || caseName.IndexOfAny(Path.GetInvalidFileNameChars()) >= 0 || caseName == "." || caseName == "..")
                throw new ArgumentException("A safe case name is required.");
            sourceRoot = Path.GetFullPath(sourceRoot); workerRoot = Path.GetFullPath(workerRoot);
            if (!Directory.Exists(sourceRoot) || !File.Exists(Path.Combine(workerRoot, "semantic_joint_worker.py")))
                throw new ArgumentException("Pinned joint source and worker paths missing.");
            var car = Object.FindObjectOfType<AIFormulaDifferentialDrive>();
            if (!car || !car.GetComponent<AIFormulaDebugPanel>() || !car.GetComponent<AIFormulaRgbCameraTopic>() || !car.GetComponent<AIFormulaDriverInput>())
                throw new InvalidOperationException("Existing vehicle, panel, RGB topic and driver required.");
            var bridge = car.GetComponent<AIFormulaLaneRuntimeBridge>(); var demo = car.GetComponent<AIFormulaE2EDemo>();
            if ((bridge && bridge.isActiveAndEnabled) || (demo && demo.isActiveAndEnabled)) throw new InvalidOperationException("Another autonomous mode owns the scene.");
            directory = Path.GetFullPath(directory);
            if (Directory.Exists(directory)) throw new InvalidOperationException("Preserve evidence: output already exists.");
            Directory.CreateDirectory(directory);
            var config = new JObject {
                ["directory"] = directory, ["case"] = caseName, ["lane_x"] = laneX, ["start_z"] = startZ,
                ["source_root"] = sourceRoot, ["worker_root"] = workerRoot, ["experiment_mode"] = "semantic_joint_1115",
                ["minimum_laps"] = 2, ["wall_timeout_sec"] = wallTimeout, ["signal_panel_rgb"] = panelRgb,
                ["startup_timeout_wall_sec"] = 90, ["no_progress_timeout_sim_sec"] = 35,
                ["signal_panel_reject_conflicting_below_horizon_lights"] = true,
                ["lane_recovery_mode"] = laneRecoveryMode,
                ["bbox_enabled"] = false, ["neo_stop_wait_enabled"] = false,
                ["scene"] = scene.path, ["requested_utc"] = DateTime.UtcNow.ToString("o"), ["requested_editor_wall"] = EditorApplication.timeSinceStartup,
                ["capture_role"] = "Actual detector.Input first-person RGB preview with original frame time; no extra render or changed intrinsics",
                ["control_inputs"] = "Existing RGB perception ABC/light and Rigidbody odometry; no evaluator route, lap, signal truth or distance",
                ["baseline"] = Configuration(car)
            };
            File.WriteAllText(Path.Combine(directory, "configuration.json"), config.ToString(Formatting.Indented));
            SessionState.SetString(Key, config.ToString(Formatting.None)); EditorApplication.isPlaying = true;
            return directory;
        }

        public static string Status() => active ? JsonConvert.SerializeObject(new { stage, travelled, completedLaps, firstLapClosed, redScheduled, detourEntered, output }) : "Idle; inspect result.json.";
        public static void Stop(string reason = "Operator stopped") { if (active) BeginFinish(reason, false); else SessionState.EraseString(Key); }

        static void Tick()
        {
            if (!EditorApplication.isPlaying || EditorApplication.isCompiling || EditorApplication.isPaused) return;
            try {
                if (!active) {
                    string pending = SessionState.GetString(Key, ""); if (string.IsNullOrEmpty(pending)) return;
                    request = JObject.Parse(pending); output = (string)request["directory"];
                    var panel = Object.FindObjectOfType<AIFormulaDebugPanel>();
                    if (!panel || !panel.Canvas) {
                        if (EditorApplication.timeSinceStartup - (double)request["requested_editor_wall"] > 30) throw new TimeoutException("Debug panel initialization timed out.");
                        return;
                    }
                    SessionState.EraseString(Key); Initialize(panel);
                }
                double now = Time.timeAsDouble;
                if (!drive || !detector || !semantic || !follower || !signal) { BeginFinish("Required component lost", false); Complete(); return; }
                var p = drive.Body.position; travelled += new Vector2(p.x - lastPosition.x, p.z - lastPosition.z).magnitude; lastPosition = p;
                if (detector.FrameSequence > 0 && detector.FrameSequence != sequence) RecordDetection();
                if (semantic.Result != null) {
                    string identity = semantic.WorkerGeneration + ":" + semantic.Result["sequence"];
                    if (identity != semanticIdentity) { semanticIdentity = identity; semanticTicks.WriteLine(Sample("semantic_response").ToString(Formatting.None)); }
                }
                if (now >= nextMotion) { nextMotion = now + .05; motion.WriteLine(Sample("motion").ToString(Formatting.None)); motionCount++; }
                if (now >= nextImage && rgbPath != null) { nextImage = now + 1.0 / 15; Capture(); }
                if (EditorApplication.timeSinceStartup >= nextStatus) { nextStatus = EditorApplication.timeSinceStartup + 1; WriteStatus(); }
                if (finishing) { if (now >= finishAt || EditorApplication.timeSinceStartup >= tailDeadline) Complete(); return; }
                Evaluate(now);
            } catch (Exception error) {
                Debug.LogException(error);
                if (active) { BeginFinish(error.ToString(), false); Complete(); }
                else { SessionState.EraseString(Key); if (!string.IsNullOrEmpty(output)) File.WriteAllText(Path.Combine(output, "result.json"), JsonConvert.SerializeObject(new { accepted = false, reason = error.ToString() }, Formatting.Indented)); }
            }
        }

        static void Initialize(AIFormulaDebugPanel panel)
        {
            drive = panel.GetComponent<AIFormulaDifferentialDrive>(); detector = panel.GetComponent<AIFormulaRoadDetector>(); follower = panel.GetComponent<AIFormulaVisionLaneFollower>();
            if (!drive || !detector || !follower || detector.Requested || follower.Running) throw new InvalidOperationException("Existing native chain unavailable or already owned.");
            semantic = panel.GetComponent<AIFormulaSemanticPlanning>(); if (!semantic) semantic = panel.gameObject.AddComponent<AIFormulaSemanticPlanning>();
            if (semantic.Requested) throw new InvalidOperationException("Semantic worker already owned.");
            var signals = Object.FindObjectsOfType<TrainingFlagPanel>(); if (signals.Length != 1) throw new InvalidOperationException("Expected one existing training lamp."); signal = signals[0];
            trace = Writer("trace.jsonl"); motion = Writer("motion.jsonl"); frames = Writer("frames.jsonl"); events = Writer("events.jsonl"); semanticTicks = Writer("semantic_ticks.jsonl");
            active = true; finishing = accepted = firstLapClosed = lapArmed = redScheduled = redObserved = detourEntered = detourCompleted = leftObserved = settingsCaptured = false;
            redDetourFailed = recoveryGreenScheduled = false; redFailureReason = null;
            motionControlStarted = false; motionControlStartedAt = -1;
            imageCount = detectionCount = motionCount = completedLaps = 0; sequence = 0; semanticIdentity = null;
            stableDetector = stableEpoch = stableSemantic = -1; travelled = firstLapDistance = lastLapDistance = lastProgressDistance = leftYaw = maxLeftYaw = maxLeftDisplacement = 0;
            redAt = waitAt = -1; startWall = EditorApplication.timeSinceStartup; startSim = Time.timeAsDouble; noProgressAt = startSim;
            nextImage = nextMotion = nextStatus = 0; stage = "learning_lap_1"; finishReason = overlayPath = rgbPath = null; learnedStatus = terminalStatus = lastPerception = null;
            File.WriteAllText(Path.Combine(output, "play_before.json"), Configuration(drive).ToString(Formatting.Indented));
            detector.Initialize(); originalOutput = detector.Configuration.output; originalPanelRgb = detector.Configuration.enableSignalPanelRgb;
            originalProjectedRecovery = detector.RecoverProjectedComponents; originalCurrentCorridor = detector.UseCurrentCorridor;
            originalRejectConflictingBelowHorizonLights = detector.RejectConflictingBelowHorizonLights;
            originalFocus = follower.RequireApplicationFocus; originalHeadroom = follower.RequireFirstFrameHeadroom;
            originalHold = semantic.DiagnosticHold; originalApproach = semantic.EnableSignalApproach; originalNeo = semantic.NeoStopWaitMode; originalJoint = semantic.JointExperimentMode;
            originalWorkerRoot = semantic.WorkerRoot; originalSourceRoot = semantic.SourceRoot; originalJointOutput = semantic.JointOutputRoot; originalDiagnostic = semantic.DiagnosticRouteOutput;
            originalSignal = signal.State; settingsCaptured = true;
            MoveArticulatedVehicle((float)request["lane_x"], (float)request["start_z"]); lastPosition = drive.Body.position;
            signal.SetState(TrainingFlagPanel.SignalState.Green); panel.SetVisible(false);
            detector.Configuration.output = 0; detector.Configuration.enableSignalPanelRgb = (bool)request["signal_panel_rgb"];
            detector.RejectConflictingBelowHorizonLights = true;
            if ((string)request["lane_recovery_mode"] == "component-only") { detector.RecoverProjectedComponents = true; detector.UseCurrentCorridor = false; }
            AIFormulaRoadDetector.Validate(detector.Configuration);
            follower.RequireApplicationFocus = false; follower.RequireFirstFrameHeadroom = true;
            semantic.NeoStopWaitMode = false; semantic.JointExperimentMode = true; semantic.EnableSignalApproach = false; semantic.DiagnosticHold = false;
            semantic.WorkerRoot = (string)request["worker_root"]; semantic.SourceRoot = (string)request["source_root"]; semantic.JointOutputRoot = Path.Combine(output, "worker"); semantic.DiagnosticRouteOutput = null;
            var calibration = typeof(AIFormulaRoadDetector).GetMethod("CameraCalibration", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);
            if (calibration == null) throw new MissingMethodException("CameraCalibration missing.");
            File.WriteAllText(Path.Combine(output, "camera_calibration.json"), JsonConvert.SerializeObject(calibration.Invoke(detector, null), Formatting.Indented));
            File.WriteAllText(Path.Combine(output, "perception_settings.json"), JsonConvert.SerializeObject(detector.Configuration, Formatting.Indented));
            File.WriteAllText(Path.Combine(output, "play_start.json"), Configuration(drive).ToString(Formatting.Indented));
            detector.SetRunning(true); semantic.Begin(); follower.Begin();
            Event("started_green_first_lap", new JObject { ["signal_position"] = Vector(signal.transform.position), ["physics_changed"] = false, ["intrinsics_changed"] = false,
                ["signal_panel_reject_conflicting_below_horizon_lights"] = detector.RejectConflictingBelowHorizonLights,
                ["original_reject_conflicting_below_horizon_lights"] = originalRejectConflictingBelowHorizonLights });
            Debug.Log("JOINT_TOPO_STARTED " + output);
        }

        static void Evaluate(double now)
        {
            double wall = EditorApplication.timeSinceStartup - startWall;
            if (wall > (double)request["wall_timeout_sec"]) { BeginFinish("Wall-time watchdog", false); return; }
            if (detector.Faulted || semantic.Faulted || !follower.Running) { BeginFinish("Chain stopped: " + detector.Status + "; " + semantic.Status + "; " + follower.Status, false); return; }
            var p = drive.Body.position; float yaw = drive.Body.rotation.eulerAngles.y;
            if (!Finite(p.x) || !Finite(p.z) || p.x < -25 || p.x > 90 || p.z < -75 || p.z > 55 || p.y < -1 || p.y > 3 || Vector3.Dot(drive.Body.rotation * Vector3.up, Vector3.up) < .7f) {
                BeginFinish("CRITICAL: gross bounds exceeded or overturned", false); return;
            }
            if (travelled > 1200) { BeginFinish("1200 m watchdog", false); return; }
            if (!motionControlStarted && (drive.RequestedVelocity.x != 0 || drive.RequestedVelocity.y != 0)) {
                motionControlStarted = true; motionControlStartedAt = now;
                noProgressAt = now; lastProgressDistance = travelled;
                Event("motion_control_started", new JObject { ["travelled"] = travelled,
                    ["applied_v"] = drive.RequestedVelocity.x, ["applied_w"] = -drive.RequestedVelocity.y });
            }
            if (!motionControlStarted && wall >= 90) { BeginFinish("No applied autonomous command within 90 wall seconds of startup", false); return; }
            if (semantic.Result == null || !semantic.Running) return;
            if (semantic.ResultAge > .25) return;
            var status = semantic.Result["status"] as JObject; if (status == null) throw new InvalidDataException("Missing joint status.");
            if ((string)semantic.Result["experiment_mode"] != "semantic_joint_1115") throw new InvalidDataException("Wrong experiment mode.");
            if (stableDetector < 0) { stableDetector = detector.WorkerId; stableEpoch = detector.LaneEpoch; stableSemantic = semantic.WorkerId; Event("same_chain_identity", new JObject { ["detector"] = stableDetector, ["semantic"] = stableSemantic, ["epoch"] = stableEpoch }); }
            if (stableDetector != detector.WorkerId || stableEpoch != detector.LaneEpoch || stableSemantic != semantic.WorkerId) { BeginFinish("Worker or epoch changed during continuous two-lap trial", false); return; }
            if (motionControlStarted && travelled - lastProgressDistance >= 1) { noProgressAt = now; lastProgressDistance = travelled; }
            if (motionControlStarted && now - noProgressAt > 35) { BeginFinish("No physical progress for 35 simulation seconds after motion control started", false); return; }
            double length = (double?)status["reference_length_m"] ?? 0;
            if (!firstLapClosed && (bool?)status["odom_reference_ready"] == true && length > 0 && !double.IsNaN(length) && !double.IsInfinity(length)) {
                firstLapClosed = true; learnedStatus = (JObject)status.DeepClone(); stage = "learned_continuing"; Event("package_reference_closed", status);
            }
            bool north = Mathf.Abs(Mathf.DeltaAngle(0, yaw)) <= 35;
            float gateDistance = new Vector2(p.x - (float)request["lane_x"], p.z - (float)request["start_z"]).magnitude;
            if (gateDistance > 18) lapArmed = true;
            // Count only after crossing the original start plane northbound, not merely approaching it.
            bool gate = north && Mathf.Abs(p.x - (float)request["lane_x"]) < 7 && p.z >= (float)request["start_z"] && p.z < (float)request["start_z"] + 6;
            if (lapArmed && gate && travelled - lastLapDistance > 120) {
                completedLaps++; lapArmed = false; lastLapDistance = travelled; if (completedLaps == 1) firstLapDistance = travelled;
                Event("physical_lap_completed", new JObject { ["completed_laps"] = completedLaps, ["travelled"] = travelled, ["reference_ready"] = firstLapClosed });
            }
            if (firstLapClosed && !redScheduled && completedLaps >= 1 && gate) {
                signal.SetState(TrainingFlagPanel.SignalState.Red); redScheduled = true; redAt = now; stage = "red_second_lap";
                Event("physical_panel_red", new JObject { ["status_before_switch"] = status.DeepClone(), ["front_gap_evaluation_only"] = FrontGap() });
            }
            string mode = (string)status["traffic_mode"], reason = (string)status["traffic_reason"];
            if (redScheduled && !redObserved && (string)status["traffic_light"] == "red") { redObserved = true; Event("package_observed_red", status); }
            if (!redScheduled && mode == "detour") { BeginFinish("Unexpected detour before scheduled second-lap red", false); return; }
            if (redScheduled && mode == "detour" && !detourEntered) {
                detourEntered = true; detourOrigin = p; detourRight = drive.Body.rotation * Vector3.right; previousYaw = yaw; stage = "detour_second_lap";
                Event("package_detour_entered", status);
            }
            if (detourEntered && !detourCompleted) {
                leftYaw -= Mathf.DeltaAngle(previousYaw, yaw); previousYaw = yaw;
                maxLeftYaw = Mathf.Max(maxLeftYaw, leftYaw); maxLeftDisplacement = Mathf.Max(maxLeftDisplacement, -Vector3.Dot(p - detourOrigin, detourRight));
                if (!leftObserved && maxLeftYaw >= 35 && maxLeftDisplacement >= 1) { leftObserved = true; Event("measured_leftward_motion", new JObject { ["yaw_deg"] = maxLeftYaw, ["displacement_m"] = maxLeftDisplacement }); }
                if (mode != "detour") {
                    if (reason == "detour_complete_resume_vision" || reason == "red_after_detour") { detourCompleted = true; stage = "detour_complete_continuing"; Event("package_detour_completed", status); }
                    else { BeginFinish("Detour exited without explicit completion: " + reason, false); return; }
                }
            }
            if (completedLaps >= 2 && (!redScheduled || now - redAt >= 15)) {
                bool passed = !redDetourFailed && firstLapClosed && redObserved && detourEntered && detourCompleted && leftObserved;
                BeginFinish(passed ? "Two physical laps with built map and completed red-triggered left turn"
                    : redDetourFailed ? "Two physical laps completed after green recovery; red detour failed: " + redFailureReason
                    : "Two laps complete; completed red-triggered interior turn not demonstrated", passed); return;
            }
            float speed = drive.Body.velocity.magnitude;
            if (redScheduled && !recoveryGreenScheduled && mode == "wait" && speed < .05f) {
                if (waitAt < 0) waitAt = now;
                if (now - waitAt >= 12) {
                    redDetourFailed = !(detourEntered && detourCompleted && leftObserved);
                    redFailureReason = redDetourFailed ? "Red wait persisted for 12 s: " + reason : null;
                    File.WriteAllText(Path.Combine(output, "red_wait_snapshot.json"), Sample("red_wait_before_green_recovery").ToString(Formatting.Indented));
                    Event(redDetourFailed ? "red_detour_failed" : "completed_detour_waiting_at_red", new JObject {
                        ["reason"] = reason, ["red_detour_failed"] = redDetourFailed, ["completed_laps"] = completedLaps, ["status"] = status.DeepClone() });
                    // Continue the requested physical second lap through genuine future green camera frames only.
                    signal.SetState(TrainingFlagPanel.SignalState.Green); recoveryGreenScheduled = true; stage = "green_recovery_second_lap";
                    noProgressAt = now; lastProgressDistance = travelled;
                    Event("physical_panel_green_for_second_lap_completion", new JObject { ["red_detour_failed"] = redDetourFailed, ["reason"] = redFailureReason });
                }
            } else waitAt = -1;
            if (redScheduled && now - redAt > 400) BeginFinish("Second-lap response exceeded 400 simulation seconds", false);
        }

        static void RecordDetection()
        {
            sequence = detector.FrameSequence; detectionCount++;
            lastPerception = string.IsNullOrEmpty(detector.LastPayload) ? null : JObject.Parse(detector.LastPayload);
            if (lastPerception != null) foreach (string name in new[] { "overlay", "lane", "obstacles" }) lastPerception.Remove(name);
            if (detector.Output) { overlayPath = "overlay_" + detectionCount.ToString("D5") + ".png"; File.WriteAllBytes(Path.Combine(output, overlayPath), detector.Output.EncodeToPNG()); }
            if (detector.Input) { rgbPath = "rgb_" + detectionCount.ToString("D5") + ".jpg"; File.WriteAllBytes(Path.Combine(output, rgbPath), detector.Input.EncodeToJPG(95)); }
            trace.WriteLine(Sample("detection").ToString(Formatting.None));
        }

        static void Capture()
        {
            frames.WriteLine(JsonConvert.SerializeObject(new { index = imageCount++, t = Time.timeAsDouble - startSim, wall = EditorApplication.timeSinceStartup - startWall,
                path = rgbPath, rgb = rgbPath, overlay = overlayPath, sequence = detector.FrameSequence, frame_sim_time = detector.FrameSimulationTime,
                stage, phase = finishing ? "stopped_tail" : "running", source = "actual_first_person_detector_input" }));
        }

        static JObject Sample(string kind)
        {
            var p = drive.Body.position;
            return new JObject {
                ["kind"] = kind, ["stage"] = stage, ["t"] = Time.timeAsDouble - startSim, ["wall"] = EditorApplication.timeSinceStartup - startWall,
                ["absolute_sim_time"] = Time.timeAsDouble, ["start_sim_time"] = startSim,
                ["x"] = p.x, ["z"] = p.z, ["yaw"] = drive.Body.rotation.eulerAngles.y, ["distance"] = travelled, ["completed_laps"] = completedLaps,
                ["actual_v"] = Vector3.Dot(drive.Body.velocity, drive.Body.rotation * Vector3.forward), ["actual_w"] = -drive.Body.angularVelocity.y,
                ["applied_v"] = drive.RequestedVelocity.x, ["applied_w"] = -drive.RequestedVelocity.y,
                ["sequence"] = detector.FrameSequence, ["frame_sim_time"] = detector.FrameSimulationTime, ["epoch"] = detector.LaneEpoch,
                ["signal_truth"] = signal.State.ToString().ToLowerInvariant(), ["detected_light"] = detector.TrafficLight,
                ["signal_panel"] = lastPerception?["signal_panel"]?.DeepClone(), ["detections"] = lastPerception?["detections"]?.DeepClone(),
                ["semantic"] = semantic.Result?.DeepClone(), ["candidate"] = detector.LaneControl?.DeepClone(),
                ["semantic_worker"] = semantic.WorkerId, ["worker_generation"] = semantic.WorkerGeneration, ["worker_boot_id"] = semantic.WorkerBootId,
                ["semantic_runtime_status"] = semantic.Status, ["semantic_result_age"] = semantic.Result == null ? (double?)null : semantic.ResultAge,
                ["follower_status"] = follower.Status, ["follower_stop_evidence"] = follower.StopEvidence?.DeepClone(),
                ["overlay"] = overlayPath, ["rgb"] = rgbPath, ["inference_ms"] = detector.InferenceMilliseconds, ["age"] = detector.FrameAge
            };
        }

        static void WriteStatus() => File.WriteAllText(Path.Combine(output, "status.json"), JsonConvert.SerializeObject(new {
            phase = finishing ? "stopped_tail" : "running", stage, travelled, completedLaps, firstLapClosed, redScheduled, redObserved, detourEntered, detourCompleted, leftObserved, redDetourFailed, recoveryGreenScheduled, motionControlStarted,
            reason = finishReason, imageCount, detectionCount, wall = EditorApplication.timeSinceStartup - startWall, t = Time.timeAsDouble - startSim,
            semantic = semantic.Status, follower = follower.Status
        }, Formatting.Indented));

        static void BeginFinish(string reason, bool pass)
        {
            if (!active || finishing) return; finishing = true; accepted = pass; finishReason = reason; finishAt = Time.timeAsDouble + 1; tailDeadline = EditorApplication.timeSinceStartup + 10;
            terminalStatus = semantic && semantic.Result != null ? (JObject)semantic.Result.DeepClone() : null;
            Event("trial_stop", new JObject { ["accepted"] = pass, ["reason"] = reason });
            if (drive && detector && semantic && follower && signal) File.WriteAllText(Path.Combine(output, "terminal_sample.json"), Sample("terminal").ToString(Formatting.Indented));
            if (follower) follower.Stop(reason); if (drive) drive.SetInput(0, 0, true);
            var driver = drive ? drive.GetComponent<AIFormulaDriverInput>() : null; if (driver) driver.SetEmergencyStop(true);
        }

        static void Complete()
        {
            if (!active) return; var cleanupErrors = new List<string>();
            try {
                try { if (follower) follower.Stop(finishReason); if (semantic) semantic.Stop(); } catch (Exception e) { cleanupErrors.Add("stop: " + e.Message); }
                try {
                    if (settingsCaptured) {
                        follower.RequireApplicationFocus = originalFocus; follower.RequireFirstFrameHeadroom = originalHeadroom;
                        semantic.DiagnosticHold = originalHold; semantic.EnableSignalApproach = originalApproach; semantic.NeoStopWaitMode = originalNeo; semantic.JointExperimentMode = originalJoint;
                        semantic.WorkerRoot = originalWorkerRoot; semantic.SourceRoot = originalSourceRoot; semantic.JointOutputRoot = originalJointOutput; semantic.DiagnosticRouteOutput = originalDiagnostic;
                        detector.Configuration.output = originalOutput; detector.Configuration.enableSignalPanelRgb = originalPanelRgb; signal.SetState(originalSignal);
                        detector.RecoverProjectedComponents = originalProjectedRecovery; detector.UseCurrentCorridor = originalCurrentCorridor;
                        detector.RejectConflictingBelowHorizonLights = originalRejectConflictingBelowHorizonLights;
                    }
                    if (detector) detector.SetRunning(false); if (drive) drive.SetInput(0, 0, true);
                } catch (Exception e) { cleanupErrors.Add("restore: " + e.Message); }
                if (semantic && ((bool?)semantic.ShutdownReceipt?["clean"] != true || (bool?)semantic.ShutdownReceipt?["forced"] != false)) cleanupErrors.Add("Worker clean shutdown not confirmed");
                if (cleanupErrors.Count > 0) accepted = false;
                File.WriteAllText(Path.Combine(output, "result.json"), JsonConvert.SerializeObject(new {
                    accepted, reason = finishReason, stage, laneRecoveryMode = (string)request["lane_recovery_mode"], completedLaps, firstLapClosed, firstLapDistance, redScheduled, redObserved, detourEntered, detourCompleted, leftObserved,
                    red_detour_failed = redDetourFailed, redFailureReason, recoveryGreenScheduled,
                    motionControlStarted, motionControlStartedTime = motionControlStartedAt < 0 ? (double?)null : motionControlStartedAt - startSim,
                    maxLeftYaw, maxLeftDisplacement, travelled, imageCount, detectionCount, motionCount, learnedStatus, terminalStatus,
                    seconds = Time.timeAsDouble - startSim, wallSeconds = EditorApplication.timeSinceStartup - startWall,
                    minimumRequestedLaps = 2, bboxEnabled = false, neoStopWaitEnabled = false, physicsParametersChanged = false, cameraIntrinsicsChanged = false,
                    signalTruthFedToControl = false, evaluatorRouteFedToControl = false, idealSimulatedOdometry = true,
                    captureSource = "actual_first_person_detector_input", cleanupErrors, shutdown = semantic ? semantic.ShutdownReceipt : null
                }, Formatting.Indented));
                if (drive) File.WriteAllText(Path.Combine(output, "play_after.json"), Configuration(drive).ToString(Formatting.Indented));
            } finally {
                trace?.Dispose(); motion?.Dispose(); frames?.Dispose(); events?.Dispose(); semanticTicks?.Dispose();
                trace = motion = frames = events = semanticTicks = null; active = false; SessionState.EraseString(Key);
            }
            Debug.Log("JOINT_TOPO_FINISHED " + finishReason + " " + output);
        }

        static JObject Configuration(AIFormulaDifferentialDrive car)
        {
            var bodies = new JArray(); foreach (var body in car.GetComponentsInChildren<Rigidbody>(true)) bodies.Add(new JObject { ["name"] = body.name, ["position"] = Vector(body.position), ["rotation"] = Rotation(body.rotation), ["mass"] = body.mass, ["constraints"] = body.constraints.ToString(), ["solverIterations"] = body.solverIterations, ["solverVelocityIterations"] = body.solverVelocityIterations });
            var feed = car.GetComponent<AIFormulaRgbCameraTopic>();
            return new JObject { ["bodies"] = bodies, ["drive"] = JsonUtility.ToJson(car), ["rgb_topic"] = feed ? JsonUtility.ToJson(feed) : null,
                ["rgb_camera"] = feed && feed.rgbCamera ? EditorJsonUtility.ToJson(feed.rgbCamera) : null,
                ["preset"] = car.GetComponent<AIFormulaRuntimeTuning>() ? car.GetComponent<AIFormulaRuntimeTuning>().SerializePreset() : null,
                ["fixedDeltaTime"] = Time.fixedDeltaTime, ["timeScale"] = Time.timeScale };
        }

        static void MoveArticulatedVehicle(float x, float z)
        {
            drive.StopMotion(); var bodies = drive.GetComponentsInChildren<Rigidbody>(true); if (bodies.Length != 3) throw new InvalidOperationException("Articulated body count changed.");
            var origin = drive.Body.position; var delta = Quaternion.Euler(0, -drive.Body.rotation.eulerAngles.y, 0); var destination = new Vector3(x, origin.y, z);
            var positions = new Vector3[bodies.Length]; var rotations = new Quaternion[bodies.Length];
            for (int i = 0; i < bodies.Length; i++) { positions[i] = destination + delta * (bodies[i].position - origin); rotations[i] = delta * bodies[i].rotation; }
            for (int i = 0; i < bodies.Length; i++) { bodies[i].position = positions[i]; bodies[i].rotation = rotations[i]; bodies[i].velocity = Vector3.zero; bodies[i].angularVelocity = Vector3.zero; }
            Physics.SyncTransforms(); drive.StopMotion();
        }

        static float FrontGap() { var p = drive.Body.position; var q = drive.Body.rotation; return 15 - Mathf.Max((p + q * new Vector3(-.3275f, 0, .216f)).z, (p + q * new Vector3(.3275f, 0, .216f)).z); }
        static void Event(string name, JObject detail = null) { var value = new JObject { ["event"] = name, ["stage"] = stage, ["t"] = Time.timeAsDouble - startSim, ["wall"] = EditorApplication.timeSinceStartup - startWall, ["detail"] = detail }; if (drive) { value["x"] = drive.Body.position.x; value["z"] = drive.Body.position.z; } events?.WriteLine(value.ToString(Formatting.None)); }
        static StreamWriter Writer(string name) => new StreamWriter(Path.Combine(output, name)) { AutoFlush = true };
        static JArray Vector(Vector3 v) => new JArray(v.x, v.y, v.z);
        static JArray Rotation(Quaternion q) => new JArray(q.x, q.y, q.z, q.w);
        static bool Finite(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
    }
}
