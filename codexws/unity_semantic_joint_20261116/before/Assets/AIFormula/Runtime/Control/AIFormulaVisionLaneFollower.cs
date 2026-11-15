using System;
using Newtonsoft.Json.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

namespace AIFormula
{
    [DefaultExecutionOrder(-550), DisallowMultipleComponent]
    public sealed class AIFormulaVisionLaneFollower : MonoBehaviour
    {
        public const double MaximumCommandAge = .75;
        public const double FirstAcceptMaximumAge = .5;
        // Runtime opt-in only: properties are not persisted by Unity serialization.
        public bool RequireFirstFrameHeadroom { get; set; }
        public bool RequireApplicationFocus { get; set; } = true;
        public bool Running { get; private set; }
        public string Status { get; private set; } = "Off";
        public float CommandV { get; private set; }
        public float CommandW { get; private set; }
        public string LaneMode { get; private set; } = "--";
        public JObject LastCandidate => detector ? detector.LaneControl : null;
        public JObject StopEvidence { get; private set; }
        AIFormulaRoadDetector detector;
        AIFormulaDriverInput driver;
        AIFormulaDifferentialDrive drive;
        AIFormulaNativeBagReplay replay;
        bool driverWasEnabled, replayWasEnabled, semanticRequired;
        int gamepadId;
        double began, lastSupport;
        long acceptedSequence;

        public void Begin()
        {
            if (Running) return;
            if (!Application.isPlaying || !isActiveAndEnabled) throw new InvalidOperationException("Enter Play first.");
            detector = GetComponent<AIFormulaRoadDetector>();
            drive = GetComponent<AIFormulaDifferentialDrive>();
            driver = GetComponent<AIFormulaDriverInput>();
            replay = GetComponent<AIFormulaNativeBagReplay>();
            var bridge = GetComponent<AIFormulaLaneRuntimeBridge>();
            var demo = GetComponent<AIFormulaE2EDemo>();
            if (!detector || !drive || !driver || !drive.UsesRepositoryMotor || !drive.isActiveAndEnabled)
                throw new InvalidOperationException("Calibrated RGB and repository motor chain are required.");
            if (!drive.keyboardControl || !driver.enabled || (replay && replay.Running) || (bridge && bridge.isActiveAndEnabled)
                || (demo && demo.isActiveAndEnabled) || driver.source == AIFormulaDriverInput.Source.EndToEnd)
                throw new InvalidOperationException("Another autonomous mode owns this scene. Use MotorControllerValidation with manual input selected.");
            if (ManualActivity()) throw new InvalidOperationException("Release manual controls first.");
            detector.EnableLaneControl(true);
            if (!detector.Requested || detector.Faulted) detector.SetRunning(true);
            if (detector.Faulted) { detector.EnableLaneControl(false); throw new InvalidOperationException(detector.Status); }
            driverWasEnabled = driver.enabled; replayWasEnabled = replay && replay.enabled;
            if (replay) replay.enabled = false;
            gamepadId = driver.SelectGamepad()?.deviceId ?? 0;
            driver.SetEmergencyStop(false); driver.enabled = false;
            var semantic = GetComponent<AIFormulaSemanticPlanning>();
            semanticRequired = semantic && semantic.Requested;
            Running = true; acceptedSequence = 0; CommandV = CommandW = 0;
            StopEvidence = null;
            began = lastSupport = Time.realtimeSinceStartupAsDouble;
            Status = "Waiting for fresh predicted lanes";
        }

        public void Stop(string reason = "Operator stopped")
        {
            if (!Running) return;
            StopEvidence = new JObject { ["reason"] = reason, ["sim_time"] = Time.timeAsDouble,
                ["frame_sim_time"] = detector ? detector.FrameSimulationTime : 0,
                ["sequence"] = detector ? detector.FrameSequence : 0,
                ["epoch"] = detector ? detector.LaneEpoch : 0,
                ["wall_age"] = detector && detector.FrameSequence > 0 ? detector.FrameAge : (double?)null,
                ["candidate"] = detector ? detector.LaneControl?.DeepClone() : null };
            Running = false; Status = reason; CommandV = CommandW = 0;
            if (drive) drive.SetInput(0, 0, true);
            if (detector) detector.EnableLaneControl(false);
            var semantic = GetComponent<AIFormulaSemanticPlanning>();
            if (semantic && semantic.Requested) semantic.Stop();
            if (driver) { driver.SetEmergencyStop(true); driver.enabled = driverWasEnabled; }
            if (replay) replay.enabled = replayWasEnabled;
        }

        public static bool Fresh(JObject value, long sequence, int epoch, double frameStamp, double now, double wallAge)
        {
            try { return sequence > 0 && wallAge >= 0 && value != null && (long?)value["sequence"] == sequence && (int?)value["epoch"] == epoch
                && (bool?)value["valid"] == true && Math.Abs(((double?)value["sim_time"] ?? double.NaN) - frameStamp) <= 1e-9
                && now >= frameStamp && now-frameStamp <= MaximumCommandAge && wallAge <= MaximumCommandAge
                && AIFormulaE2EInference.Finite((float?)value["v"] ?? float.NaN)
                && AIFormulaE2EInference.Finite((float?)value["w"] ?? float.NaN); }
            catch (Exception) { return false; }
        }

        void FixedUpdate()
        {
            if (!Running) return;
            var semantic = GetComponent<AIFormulaSemanticPlanning>();
            if (semanticRequired && (!semantic || !semantic.isActiveAndEnabled || !semantic.Requested || semantic.Faulted))
            { Stop("Semantic command ownership lost or transport failed"); return; }
            var other = GetComponent<AIFormulaLaneRuntimeBridge>();
            var demo = GetComponent<AIFormulaE2EDemo>();
            if (!detector.Requested || detector.Faulted || driver.enabled || !drive.isActiveAndEnabled || !drive.keyboardControl
                || (other && other.isActiveAndEnabled) || (replay && replay.Running) || (demo && demo.isActiveAndEnabled)
                || driver.source == AIFormulaDriverInput.Source.EndToEnd)
            { Stop("Perception or command ownership changed"); return; }
            if (RequireApplicationFocus && !Application.isFocused)
            { Stop("Application focus lost"); return; }
            if (driver.EmergencyStop)
            { Stop("Driver E-STOP asserted"); return; }
            if (driver.UIBlocked)
            { Stop("Driving UI input blocked"); return; }
            if (ManualActivity())
            { Stop("Manual input detected"); return; }
            if ((driver.SelectGamepad()?.deviceId ?? 0) != gamepadId)
            { Stop("Selected gamepad changed"); return; }
            double now = Time.realtimeSinceStartupAsDouble;
            var value = detector.LaneControl;
            if(value != null && value["error"] != null) {Stop("Lane pipeline: "+(string)value["error"]);return;}
            if (value == null || (int?)value["epoch"] != detector.LaneEpoch)
            {
                drive.SetInput(0, 0, true);
                if (now-began > 90) Stop("No lane response within 90 seconds");
                return;
            }
            double simulationNow = Time.timeAsDouble, frameAge = detector.FrameAge;
            if (!Fresh(value, detector.FrameSequence, detector.LaneEpoch, detector.FrameSimulationTime,
                       simulationNow, frameAge))
            {
                // CUDA's first inference may be slow. Before the first accepted
                // command, remain braked and wait for a fresh frame, never drive it.
                if(acceptedSequence==0 && now-began<90) {drive.SetInput(0,0,true);return;}
                Stop("Invalid or expired lane command: " + ((string)value["error"] ?? "timestamp/sequence")); return;
            }
            // Do not arm the post-start watchdog with a nearly expired cold frame.
            // Both clocks match the existing semantic point TTL; the active .75 s
            // command TTL and every default caller remain unchanged.
            if (RequireFirstFrameHeadroom && acceptedSequence == 0
                && Math.Max(simulationNow - detector.FrameSimulationTime, frameAge) > FirstAcceptMaximumAge)
            {
                drive.SetInput(0, 0, true);
                Status = "Waiting for first lane frame with freshness headroom";
                if (now-began >= 90) Stop("No startup lane frame with freshness headroom within 90 seconds");
                return;
            }
            if (acceptedSequence != detector.FrameSequence)
            {
                acceptedSequence = detector.FrameSequence;
                LaneMode = (string)value["mode"];
                if ((bool?)value["supported"] == true) lastSupport = now;
                CommandV = (float)value["v"]; CommandW = (float)value["w"];
            }
            // Upstream's default-straight output remains observable, but is not a
            // substitute for perception. Do not drive blind beyond this watchdog.
            if ((bool?)value["supported"] != true && now-lastSupport > MaximumCommandAge)
            { Stop("Lost predicted lanes; upstream default-straight rejected"); return; }
            if (!driver.AllowInput(true, true, false, false)) { drive.SetInput(0, 0, true); return; }
            if (semanticRequired) {
                if (semantic.Faulted) { Stop("Semantic transport: " + semantic.Status); return; }
                if (!semantic.TryGetCommand(out float v, out float w)) { drive.SetInput(0, 0, true); return; }
                CommandV = v; CommandW = w;
                Status = "YOLOP -> LLP -> semantic -> LYA | " + semantic.Status;
            } else Status = "YOLOP -> lane publisher -> LYA | " + LaneMode;
            drive.SetAutonomousVelocityCommand(CommandV, -CommandW, false);
        }

        bool ManualActivity()
        {
            var key = Keyboard.current;
            if (key != null && (key.wKey.isPressed || key.aKey.isPressed || key.sKey.isPressed || key.dKey.isPressed
                || key.spaceKey.isPressed || key.escapeKey.isPressed)) return true;
            var pad = driver.SelectGamepad();
            if (pad == null) return false;
            var raw = driver.Read(pad);
            return raw.l2 > .02f || raw.r2 > .02f || Mathf.Abs(raw.leftY) > driver.axisDeadzone
                || Mathf.Abs(raw.rightX) > driver.axisDeadzone || raw.l1 || raw.l2Button;
        }
        void OnDisable() { Stop("Follower disabled"); }
    }
}
