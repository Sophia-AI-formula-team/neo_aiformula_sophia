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
    // 只适配消息与时钟；地图、票决和速度门控由原 Python 节点执行。
    [DefaultExecutionOrder(-600), DisallowMultipleComponent]
    public sealed class AIFormulaSemanticPlanning : MonoBehaviour
    {
        public bool Requested { get; private set; }
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
        double requestAt, resultAt, nextTick;
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
            try {
                var start = new ProcessStartInfo {
                    FileName = detector.Configuration.python.Trim(),
                    Arguments = "-B -u \"" + worker + "\" --source-root \"" + Path.Combine(root, "source")
                        + "\" --vision-root \"" + Path.Combine(Application.streamingAssetsPath, "VisionLaneFollower") + "\"",
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
                requestAt = Time.realtimeSinceStartupAsDouble; Status = "Loading unchanged semantic package";
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
                    request["vision"] = new JObject {
                        ["sequence"] = visionSequence, ["epoch"] = detector.LaneEpoch,
                        ["sim_time"] = detector.FrameSimulationTime, ["age"] = detector.FrameAge,
                        ["supported"] = (bool?)lane["valid"] == true && (bool?)lane["supported"] == true,
                        ["selected_points"] = lane["selected_points"]?.DeepClone(),
                        ["traffic_light_state"] = detector.TrafficLight
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

        public bool TryGetCommand(out float v, out float w)
        {
            v = w = 0;
            if (!Running || Faulted || ResultAge > .25) return false;
            v = (float)Result["v"]; w = (float)Result["w"]; return true;
        }

        static Task<string> Observe(Task<string> task)
        {
            task.ContinueWith(t => { var observed = t.Exception; }, TaskContinuationOptions.OnlyOnFaulted);
            return task;
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
