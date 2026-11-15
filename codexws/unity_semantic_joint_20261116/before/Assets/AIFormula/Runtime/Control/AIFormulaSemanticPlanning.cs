using System;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace AIFormula
{
    // Neo stop_wait adapter: existing LYA command, traffic-only gate, topo observer.
    [DefaultExecutionOrder(-600), DisallowMultipleComponent]
    public sealed class AIFormulaSemanticPlanning : MonoBehaviour
    {
        public bool Requested { get; private set; }
        public bool EnableSignalApproach { get; set; }
        public bool NeoStopWaitMode { get; set; }
        public string WorkerRoot { get; set; }
        public string SourceRoot { get; set; }
        public string MotorGateSource { get; set; }
        public string CheckpointRoot { get; set; }
        public string CheckpointPath { get; set; }
        public string TrafficPolicy { get; set; } = "stop_wait";
        public bool DiagnosticHold { get; set; }
        public int WorkerGeneration { get; private set; }
        public string WorkerBootId { get; private set; }
        public JObject ReadyReceipt { get; private set; }
        public JObject ShutdownReceipt { get; private set; }
        // Runtime-only opt-in diagnostic path. Never serialized into the scene.
        public string DiagnosticRouteOutput { get; set; }
        public bool Running { get; private set; }
        public bool Faulted { get; private set; }
        public string Status { get; private set; } = "Off";
        public JObject Result { get; private set; }
        public int WorkerId { get { try { return process != null && !process.HasExited ? process.Id : 0; } catch { return 0; } } }
        public double ResultAge => Result == null ? double.PositiveInfinity : Time.realtimeSinceStartupAsDouble - resultAt;
        AIFormulaRoadDetector detector;
        AIFormulaDifferentialDrive drive;
        Process process;
        Task<string> pending;
        readonly StringBuilder stderr = new StringBuilder();
        long tickSequence, pendingSequence, visionSequence;
        long corridorPointSequence;
        int corridorEpoch;
        double requestAt, resultAt, nextTick;
        double corridorDeadline = double.PositiveInfinity;
        bool ready, activeNeoStopWaitMode;

        public void Begin()
        {
            if (Requested) return;
            if (!Application.isPlaying) throw new InvalidOperationException("Enter Play before semantic planning.");
            if (NeoStopWaitMode && (EnableSignalApproach || TrafficPolicy != "stop_wait"))
                throw new InvalidOperationException("Neo transport requires stop_wait; bbox/legacy distance is a separate trial.");
            if (NeoStopWaitMode && !string.IsNullOrWhiteSpace(DiagnosticRouteOutput))
                throw new InvalidOperationException("Legacy route diagnostic capture belongs to its separate trial.");
            if (NeoStopWaitMode && (string.IsNullOrWhiteSpace(SourceRoot) || !Path.IsPathRooted(SourceRoot)
                || string.IsNullOrWhiteSpace(MotorGateSource) || !Path.IsPathRooted(MotorGateSource)
                || string.IsNullOrWhiteSpace(CheckpointRoot) || !Path.IsPathRooted(CheckpointRoot)
                || string.IsNullOrWhiteSpace(CheckpointPath) || !Path.IsPathRooted(CheckpointPath)
                || string.IsNullOrWhiteSpace(WorkerRoot) || !Path.IsPathRooted(WorkerRoot)))
                throw new InvalidOperationException("Explicit absolute source and isolated per-case checkpoint paths are required.");
            detector = GetComponent<AIFormulaRoadDetector>();
            drive = GetComponent<AIFormulaDifferentialDrive>();
            if (!detector || !drive) throw new InvalidOperationException("Existing perception and vehicle are required.");
            detector.Initialize();
            string root = NeoStopWaitMode ? Path.GetFullPath(WorkerRoot)
                : Path.Combine(Application.streamingAssetsPath, "SemanticPlanning");
            string worker = Path.Combine(root, "semantic_worker.py");
            if (!File.Exists(worker)) throw new FileNotFoundException("Semantic transport worker is missing.", worker);
            activeNeoStopWaitMode = NeoStopWaitMode;
            Requested = true; Running = false; Faulted = false; ready = false; Result = null;
            WorkerGeneration++; WorkerBootId = null; ReadyReceipt = null;
            tickSequence = pendingSequence = visionSequence = 0; nextTick = 0;
            corridorPointSequence = 0; corridorEpoch = -1; corridorDeadline = double.PositiveInfinity;
            try {
                string arguments = NeoStopWaitMode
                    ? "-B -u " + QuoteArgument(worker)
                        + " --source-root " + QuoteArgument(Path.GetFullPath(SourceRoot))
                        + " --vision-root " + QuoteArgument(Path.Combine(Application.streamingAssetsPath, "VisionLaneFollower"))
                        + " --motor-gate-source " + QuoteArgument(Path.GetFullPath(MotorGateSource))
                        + " --checkpoint-root " + QuoteArgument(Path.GetFullPath(CheckpointRoot))
                        + " --checkpoint-path " + QuoteArgument(Path.GetFullPath(CheckpointPath))
                        + " --traffic-policy stop_wait"
                    : "-B -u \"" + worker + "\" --source-root \"" + Path.Combine(root, "source")
                        + "\" --vision-root \"" + Path.Combine(Application.streamingAssetsPath, "VisionLaneFollower") + "\""
                        + (EnableSignalApproach ? " --signal-approach" : "")
                        + (string.IsNullOrWhiteSpace(DiagnosticRouteOutput) ? ""
                            : " --diagnostic-route-output " + QuoteArgument(DiagnosticRouteOutput));
                var start = new ProcessStartInfo {
                    FileName = detector.Configuration.python.Trim(),
                    Arguments = arguments,
                    WorkingDirectory = root, UseShellExecute = false, CreateNoWindow = true,
                    RedirectStandardInput = true, RedirectStandardOutput = true, RedirectStandardError = true,
                    StandardOutputEncoding = Encoding.UTF8, StandardErrorEncoding = Encoding.UTF8
                };
                start.EnvironmentVariables["PYTHONDONTWRITEBYTECODE"] = "1";
                start.EnvironmentVariables["PYTHONUTF8"] = "1";
                start.EnvironmentVariables["OPENBLAS_NUM_THREADS"] = "1";
                process = new Process { StartInfo = start };
                lock (stderr) stderr.Clear();
                process.ErrorDataReceived += (_, e) => { if (e.Data == null) return; lock (stderr) { stderr.AppendLine(e.Data); if (stderr.Length > 4000) stderr.Remove(0, stderr.Length - 4000); } };
                if (!process.Start()) throw new InvalidOperationException("Could not start semantic transport.");
                process.BeginErrorReadLine();
                pending = Observe(process.StandardOutput.ReadLineAsync());
                requestAt = Time.realtimeSinceStartupAsDouble; Status = "Loading semantic package";
            } catch (Exception error) { Fail(error.GetBaseException().Message); }
        }

        void FixedUpdate()
        {
            if (!Requested || Faulted) return;
            try {
                if (NeoStopWaitMode != activeNeoStopWaitMode)
                    throw new InvalidOperationException("Stop the semantic worker before changing transport mode.");
                double now = Time.realtimeSinceStartupAsDouble;
                if (pending != null) {
                    if (!pending.IsCompleted) {
                        if (now - requestAt > (ready ? 2 : 30)) throw new TimeoutException("Semantic transport response timed out.");
                        return;
                    }
                    string line = pending.GetAwaiter().GetResult(); pending = null;
                    if (line == null) { string detail; lock (stderr) detail = stderr.ToString(); throw new IOException("Semantic worker exited: " + detail); }
                    var reply = JObject.Parse(line);
                    if ((string)reply["type"] == "error") throw new InvalidDataException((string)reply["error"] ?? reply.ToString());
                    if (!ready) {
                        if ((string)reply["type"] != "ready") throw new InvalidDataException("Semantic handshake mismatch.");
                        if (NeoStopWaitMode && ((string)reply["traffic_policy"] != "stop_wait" || (bool?)reply["signal_approach"] != false))
                            throw new InvalidDataException("Wrong semantic policy: distance trial cannot enter neo stop_wait.");
                        WorkerBootId = (string)reply["worker_boot_id"];
                        if (NeoStopWaitMode && string.IsNullOrWhiteSpace(WorkerBootId)) throw new InvalidDataException("Missing worker boot identity.");
                        ReadyReceipt = reply;
                        ready = Running = true; Status = "Waiting for predicted lane points";
                    } else {
                        if ((long?)reply["sequence"] != pendingSequence || (bool?)reply["valid"] != true)
                            throw new InvalidDataException("Semantic tick identity or validity mismatch: " + reply.ToString(Formatting.None));
                        if (NeoStopWaitMode && ((string)reply["worker_boot_id"] != WorkerBootId
                            || (string)reply["status"]?["traffic_policy"] != "stop_wait"
                            || (bool?)reply["status"]?["signal_approach"]?["enabled"] != false))
                            throw new InvalidDataException("Worker identity or stop_wait policy changed.");
                        float v = (float?)reply["v"] ?? float.NaN, w = (float?)reply["w"] ?? float.NaN;
                        if (!AIFormulaE2EInference.Finite(v) || !AIFormulaE2EInference.Finite(w))
                            throw new InvalidDataException("Nonfinite semantic command.");
                        reply["worker_generation"] = WorkerGeneration;
                        Result = reply; resultAt = requestAt;
                        UpdateCorridorDeadline(reply);
                        Status = NeoStopWaitMode ? ((string)reply["command_gate"]?["reason"] ?? "Semantic active")
                            : ((string)reply["status"]?["velocity_gate_reason"] ?? (string)reply["status"]?["reason"] ?? "Semantic active");
                    }
                }
                if (!ready || now < nextTick) return;
                var p = drive.Body.position; var f = drive.Body.rotation * Vector3.forward;
                var request = new JObject {
                    ["type"] = "tick", ["sequence"] = ++tickSequence, ["sim_time"] = Time.timeAsDouble,
                    ["odom"] = new JObject { ["x"] = p.z, ["y"] = -p.x, ["yaw"] = -Math.Atan2(f.x, f.z),
                        ["v"] = Vector3.Dot(drive.Body.velocity, f), ["w"] = -drive.Body.angularVelocity.y },
                    ["vision"] = null
                };
                if (NeoStopWaitMode) request["lane_epoch"] = detector.LaneEpoch;
                // 每帧仅送一次检测（包括 unknown）；20 Hz 控制tick不能重复投票。
                var lane = detector.LaneControl;
                if (detector.FrameSequence > visionSequence && lane != null && (int?)lane["epoch"] == detector.LaneEpoch) {
                    visionSequence = detector.FrameSequence;
                    request["vision"] = new JObject {
                        ["sequence"] = visionSequence, ["epoch"] = detector.LaneEpoch,
                        ["sim_time"] = detector.FrameSimulationTime, ["age"] = detector.FrameAge,
                        ["supported"] = (bool?)lane["valid"] == true && (bool?)lane["supported"] == true,
                        ["selected_points"] = lane["selected_points"]?.DeepClone(),
                        ["corridor"] = lane["corridor"]?.DeepClone(),
                        ["traffic_light_state"] = detector.TrafficLight
                    };
                    if (NeoStopWaitMode) {
                        request["vision"]["vision_commit"] = lane["commit"]?.DeepClone();
                        request["vision"]["controller"] = new JObject { ["v"] = lane["v"]?.DeepClone(),
                            ["w"] = lane["w"]?.DeepClone(), ["target"] = lane["target"]?.DeepClone() };
                    } else {
                        var perception = string.IsNullOrEmpty(detector.LastPayload) ? null : JObject.Parse(detector.LastPayload);
                        request["vision"]["signal_range"] = perception?["signal_range"]?.DeepClone();
                    }
                }
                string json = request.ToString(Formatting.None);
                pendingSequence = tickSequence; requestAt = now;
                nextTick = nextTick <= 0 || now - nextTick > .1 ? now + .05 : nextTick + .05;
                var worker = process;
                pending = Observe(Task.Run(async () => {
                    await worker.StandardInput.WriteLineAsync(json).ConfigureAwait(false);
                    await worker.StandardInput.FlushAsync().ConfigureAwait(false);
                    return await worker.StandardOutput.ReadLineAsync().ConfigureAwait(false);
                }));
            } catch (Exception error) { Fail(error.GetBaseException().Message); }
        }

        void UpdateCorridorDeadline(JObject reply)
        {
            var corridor = reply["corridor_guard"] as JObject;
            if (corridor == null) {
                corridorPointSequence = 0; corridorEpoch = -1; corridorDeadline = double.PositiveInfinity;
                return;
            }
            long source = (long?)corridor["source_sequence"] ?? 0;
            int epoch = (int?)reply["epoch"] ?? -1;
            double remaining = (double?)corridor["remaining_sec"] ?? double.NaN;
            if (source <= 0 || epoch < 0 || double.IsNaN(remaining) || double.IsInfinity(remaining) || remaining < 0)
                throw new InvalidDataException("Invalid corridor command deadline.");
            double deadline = requestAt + remaining;
            // Bind to the actual ABC source. Repeated odom-only/unsupported
            // ticks cannot renew it when simulation time pauses or is clamped.
            corridorDeadline = source == corridorPointSequence && epoch == corridorEpoch
                ? Math.Min(corridorDeadline, deadline) : deadline;
            corridorPointSequence = source; corridorEpoch = epoch;
        }

        public bool TryGetCommand(out float v, out float w)
        {
            v = w = 0;
            if (activeNeoStopWaitMode && DiagnosticHold) return false;
            if (Requested && NeoStopWaitMode != activeNeoStopWaitMode) return false;
            if (!Running || Faulted || ResultAge > .25) return false;
            if (activeNeoStopWaitMode) {
                double remaining = (double?)Result["command_gate"]?["remaining_sec"] ?? 0;
                if (double.IsNaN(remaining) || double.IsInfinity(remaining) || remaining < 0 || ResultAge > remaining)
                    return false;
            }
            // The producer's width prior also expires across transport latency.
            var corridor = Result["corridor_guard"] as JObject;
            if (corridor != null && ((bool?)corridor["valid"] != true
                || ResultAge > ((double?)corridor["remaining_sec"] ?? 0)
                || Time.realtimeSinceStartupAsDouble > corridorDeadline)) {
                Status = "Corridor width prior expired before command consumption";
                return false;
            }
            v = (float)Result["v"]; w = (float)Result["w"]; return true;
        }

        static Task<string> Observe(Task<string> task)
        {
            task.ContinueWith(t => { var observed = t.Exception; }, TaskContinuationOptions.OnlyOnFaulted);
            return task;
        }
        static string QuoteArgument(string value)
        {
            // Windows argv quoting: double backslashes before quotes and the
            // closing delimiter. UseShellExecute stays false; no shell runs.
            var quoted = new StringBuilder("\"");
            int slashes = 0;
            foreach (char ch in value) {
                if (ch == '\\') { slashes++; continue; }
                quoted.Append('\\', ch == '"' ? slashes * 2 + 1 : slashes);
                quoted.Append(ch); slashes = 0;
            }
            quoted.Append('\\', slashes * 2).Append('"');
            return quoted.ToString();
        }
        public bool RestartWorkerPreservingOdom()
        {
            if (!activeNeoStopWaitMode || !NeoStopWaitMode || !Requested || Faulted || !Application.isPlaying) return false;
            // The caller holds the existing follower at zero via DiagnosticHold.
            // No detector, lane epoch, vehicle state or simulation clock changes.
            if (!DiagnosticHold) throw new InvalidOperationException("Hold the existing follower before semantic-only restart.");
            Running = false; Result = null;
            bool clean = Shutdown();
            if (!clean) { Faulted = true; Status = "Semantic restart refused after unclean shutdown"; return false; }
            Requested = false;
            Begin();
            return Requested && !Faulted;
        }

        void Fail(string reason) { Faulted = true; Running = false; Status = reason; Shutdown(); }
        public void Stop() { Requested = Running = false; bool clean = Shutdown(); Status = clean ? "Off" : "Off; checkpoint close unconfirmed"; }
        bool Shutdown()
        {
            var priorPending = pending;
            pending = null; ready = false;
            var worker = process; process = null;
            if (worker == null) return true;
            if (!activeNeoStopWaitMode) {
                // Preserve the existing historical transport lifecycle exactly.
                try { if (!worker.HasExited) { worker.Kill(); worker.WaitForExit(200); } }
                catch (InvalidOperationException) { }
                finally { worker.Dispose(); }
                return true;
            }
            int id = 0; bool clean = false, forced = false; string error = null;
            JObject acknowledgement = null;
            var elapsed = Stopwatch.StartNew();
            try {
                id = worker.Id;
                if (!worker.HasExited) {
                    // One outstanding tick/handshake only. Finish it before the
                    // shutdown message so two readers/writers cannot race.
                    if (priorPending != null && !priorPending.Wait(1000))
                        throw new TimeoutException("Pending semantic response did not finish before shutdown.");
                    if (priorPending != null) priorPending.GetAwaiter().GetResult();
                    worker.StandardInput.WriteLine("{\"type\":\"shutdown\"}");
                    worker.StandardInput.Flush();
                    var closing = Observe(worker.StandardOutput.ReadLineAsync());
                    if (!closing.Wait(2000)) throw new TimeoutException("Checkpoint close acknowledgement timed out.");
                    string line = closing.GetAwaiter().GetResult();
                    if (line != null) acknowledgement = JObject.Parse(line);
                    if (!worker.WaitForExit(1000)) throw new TimeoutException("Semantic worker did not exit after checkpoint close.");
                    clean = worker.ExitCode == 0 && (string)acknowledgement?["type"] == "shutdown"
                        && (bool?)acknowledgement?["clean"] == true
                        && (bool?)acknowledgement?["checkpoint_ok"] == true
                        && (WorkerBootId == null || (string)acknowledgement?["worker_boot_id"] == WorkerBootId);
                }
            } catch (Exception failure) { error = failure.GetBaseException().Message; }
            finally {
                try { if (!worker.HasExited) { forced = true; worker.Kill(); worker.WaitForExit(1000); } }
                catch (Exception failure) { error = failure.GetBaseException().Message; }
                bool exited = false; int? exitCode = null;
                try { exited = worker.HasExited; if (exited) exitCode = worker.ExitCode; } catch { }
                ShutdownReceipt = new JObject { ["clean"] = clean && !forced && exited,
                    ["before_worker_id"] = id, ["worker_boot_id"] = WorkerBootId,
                    ["forced"] = forced, ["exited"] = exited, ["exit_code"] = exitCode,
                    ["elapsed_ms"] = elapsed.Elapsed.TotalMilliseconds, ["error"] = error,
                    ["acknowledgement"] = acknowledgement };
                worker.Dispose();
            }
            return (bool?)ShutdownReceipt["clean"] == true;
        }
        void OnDisable() { Stop(); }
        void OnDestroy() { Shutdown(); }
        void OnApplicationQuit() { Shutdown(); }
    }
}
