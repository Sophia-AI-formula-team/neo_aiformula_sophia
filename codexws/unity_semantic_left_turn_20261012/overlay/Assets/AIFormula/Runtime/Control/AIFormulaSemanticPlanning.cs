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
    // 只适配消息与时钟；地图、票决和可选测距接近由 Python 节点执行。
    [DefaultExecutionOrder(-600), DisallowMultipleComponent]
    public sealed class AIFormulaSemanticPlanning : MonoBehaviour
    {
        public bool Requested { get; private set; }
        public bool EnableSignalApproach { get; set; }
        // Runtime-only opt-in diagnostic path. Never serialized into the scene.
        public string DiagnosticRouteOutput { get; set; }
        // Explicit simulator control priors; empty retains the original chain.
        public string CalibratedMapPath { get; set; }
        public string RecordedReferencePath { get; set; }
        public bool EnableCalibratedDetourOwnership { get; set; }
        public bool HasFreshCalibratedDetourPoints
        {
            get {
                try {
                if (!EnableCalibratedDetourOwnership || !Running || Faulted || ResultAge > .25 || Result == null) return false;
                var status = Result["status"] as JObject;
                if ((string)status?["traffic_mode"] != "detour"
                    || (string)status?["map_fusion"]?["mode"] != "calibrated_map"
                    || (bool?)status?["map_fusion"]?["available"] != true
                    || (bool?)Result["selected_points_new"] != true || (bool?)Result["command_new"] != true
                    || (string)status?["velocity_gate_reason"] != "forwarding_controller") return false;
                var points = Result["selected_points"] as JArray;
                if (points == null || points.Count != 3 || !TryGetCommand(out float v, out float w)
                    || float.IsNaN(v) || float.IsInfinity(v) || float.IsNaN(w) || float.IsInfinity(w)) return false;
                foreach (var point in points) {
                    if (!(point is JArray pair) || pair.Count != 2) return false;
                    double x = (double)pair[0], y = (double)pair[1];
                    if (double.IsNaN(x) || double.IsInfinity(x) || double.IsNaN(y) || double.IsInfinity(y) || x < 0) return false;
                }
                return true;
                } catch (Exception) { return false; }
            }
        }
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
        bool ready;

        public void Begin()
        {
            if (Requested) return;
            if (!Application.isPlaying) throw new InvalidOperationException("Enter Play before semantic planning.");
            detector = GetComponent<AIFormulaRoadDetector>();
            drive = GetComponent<AIFormulaDifferentialDrive>();
            if (!detector || !drive) throw new InvalidOperationException("Existing perception and vehicle are required.");
            detector.Initialize();
            string root = Path.Combine(Application.streamingAssetsPath, "SemanticPlanning");
            string worker = Path.Combine(root, "semantic_worker.py");
            if (!File.Exists(worker)) throw new FileNotFoundException("Semantic transport worker is missing.", worker);
            Requested = true; Running = false; Faulted = false; ready = false; Result = null;
            tickSequence = pendingSequence = visionSequence = 0; nextTick = 0;
            corridorPointSequence = 0; corridorEpoch = -1; corridorDeadline = double.PositiveInfinity;
            try {
                var start = new ProcessStartInfo {
                    FileName = detector.Configuration.python.Trim(),
                    Arguments = "-B -u \"" + worker + "\" --source-root \"" + Path.Combine(root, "source")
                        + "\" --vision-root \"" + Path.Combine(Application.streamingAssetsPath, "VisionLaneFollower") + "\""
                        + (EnableSignalApproach ? " --signal-approach" : "")
                        + (string.IsNullOrWhiteSpace(DiagnosticRouteOutput) ? ""
                            : " --diagnostic-route-output " + QuoteArgument(DiagnosticRouteOutput))
                        + (string.IsNullOrWhiteSpace(CalibratedMapPath) ? ""
                            : " --calibrated-map " + QuoteArgument(CalibratedMapPath))
                        + (string.IsNullOrWhiteSpace(RecordedReferencePath) ? ""
                            : " --recorded-reference " + QuoteArgument(RecordedReferencePath)),
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
                double now = Time.realtimeSinceStartupAsDouble;
                if (pending != null) {
                    if (!pending.IsCompleted) {
                        if (now - requestAt > (ready ? 2 : 30)) throw new TimeoutException("Semantic transport response timed out.");
                        return;
                    }
                    string line = pending.GetAwaiter().GetResult(); pending = null;
                    if (line == null) { string detail; lock (stderr) detail = stderr.ToString(); throw new IOException("Semantic worker exited: " + detail); }
                    var reply = JObject.Parse(line);
                    if ((string)reply["type"] == "error") throw new InvalidDataException((string)reply["message"] ?? reply.ToString());
                    if (!ready) {
                        if ((string)reply["type"] != "ready") throw new InvalidDataException("Semantic handshake mismatch.");
                        ready = Running = true; Status = "Waiting for predicted lane points";
                    } else {
                        if ((long?)reply["sequence"] != pendingSequence || (bool?)reply["valid"] != true)
                            throw new InvalidDataException("Semantic tick identity or validity mismatch: " + reply.ToString(Formatting.None));
                        float v = (float?)reply["v"] ?? float.NaN, w = (float?)reply["w"] ?? float.NaN;
                        if (!AIFormulaE2EInference.Finite(v) || !AIFormulaE2EInference.Finite(w))
                            throw new InvalidDataException("Nonfinite semantic command.");
                        Result = reply; resultAt = requestAt;
                        UpdateCorridorDeadline(reply);
                        Status = (string)reply["status"]?["velocity_gate_reason"] ?? (string)reply["status"]?["reason"] ?? "Semantic active";
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
                // 每帧仅送一次检测（包括 unknown）；20 Hz 控制tick不能重复投票。
                var lane = detector.LaneControl;
                if (detector.FrameSequence > visionSequence && lane != null && (int?)lane["epoch"] == detector.LaneEpoch) {
                    visionSequence = detector.FrameSequence;
                    var perception = string.IsNullOrEmpty(detector.LastPayload) ? null : JObject.Parse(detector.LastPayload);
                    request["vision"] = new JObject {
                        ["sequence"] = visionSequence, ["epoch"] = detector.LaneEpoch,
                        ["sim_time"] = detector.FrameSimulationTime, ["age"] = detector.FrameAge,
                        ["supported"] = (bool?)lane["valid"] == true && (bool?)lane["supported"] == true,
                        ["selected_points"] = lane["selected_points"]?.DeepClone(),
                        ["corridor"] = lane["corridor"]?.DeepClone(),
                        ["traffic_light_state"] = detector.TrafficLight,
                        ["signal_range"] = perception?["signal_range"]?.DeepClone()
                    };
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
            if (!Running || Faulted || ResultAge > .25) return false;
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
        void Fail(string reason) { Faulted = true; Running = false; Status = reason; Shutdown(); }
        public void Stop() { Requested = Running = false; Shutdown(); Status = "Off"; }
        void Shutdown()
        {
            pending = null; ready = false;
            var worker = process; process = null;
            if (worker == null) return;
            try { if (!worker.HasExited) { worker.Kill(); worker.WaitForExit(200); } }
            catch (InvalidOperationException) { }
            finally { worker.Dispose(); }
        }
        void OnDisable() { Stop(); }
        void OnDestroy() { Shutdown(); }
        void OnApplicationQuit() { Shutdown(); }
    }
}
