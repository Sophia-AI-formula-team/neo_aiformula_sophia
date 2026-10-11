using System;
using System.Diagnostics;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace AIFormula
{
    // RGB plus camera calibration only. Optional control candidates never actuate here.
    [DefaultExecutionOrder(950), DisallowMultipleComponent]
    public sealed class AIFormulaRoadDetector : MonoBehaviour
    {
        public const string ModelTag = "a3-v14";
        public const string WeightSha256 = "a411c983b661d6b26217019a968251c89a6b66c95400e4e7cb78f04a644ca10c";
        [Serializable] public sealed class Settings
        {
            public string python = "python";
            public string device = "auto";
            public float frequency = 5;
            public int output;
            public bool enableSignalPanelRgb;
            public bool showVisualization = true;
            public bool showLanePlot;
            public bool showOutputOverlay = true;
            public float visualizationScale = 1;
        }
        public Settings Configuration { get; private set; } = new Settings();
        public bool Requested { get; private set; }
        public bool Ready { get; private set; }
        public bool Faulted { get; private set; }
        public string Status { get; private set; } = "Off";
        public string Device { get; private set; } = "--";
        public string TrafficLight { get; private set; } = "unknown";
        public int DetectionCount { get; private set; }
        public float InferenceMilliseconds { get; private set; }
        public float RoundTripMilliseconds { get; private set; }
        public long CompletedFrames { get; private set; }
        public long FrameSequence { get; private set; }
        public double FrameAge => FrameSequence > 0 ? Math.Max(0, Time.realtimeSinceStartupAsDouble - displayedAt) : double.PositiveInfinity;
        public bool Stale => FrameAge > Math.Max(2, 3 / Configuration.frequency);
        public Texture2D Input => input;
        public Texture2D LaneMask => lane;
        public Texture2D Output => Configuration.output == 1 ? lane : Configuration.output == 2 ? obstacles : overlay;
        public string LastPayload { get; private set; }
        public JObject LaneControl { get; private set; }
        public bool LaneControlEnabled { get; private set; }
        // Explicit trial-only adapter option; not persisted in user settings.
        public bool RecoverProjectedComponents { get; set; }
        public bool UseCurrentCorridor { get; set; } = true;
        public int LaneEpoch { get; private set; }
        public bool CanProjectLanes => feed && feed.useRepositoryIntrinsics;
        public bool LanePreviewRequested => Configuration.showVisualization && Configuration.showLanePlot;
        public double FrameSimulationTime { get; private set; }
        public int WorkerId { get { try { return process != null && !process.HasExited ? process.Id : 0; } catch { return 0; } } }
        public static string SettingsPath => Path.Combine(Application.isEditor ? Path.GetFullPath(Path.Combine(Application.dataPath, "../UserSettings")) : Application.persistentDataPath, "AIFormulaRoadDetector.json");
        AIFormulaRgbCameraTopic feed;
        Process process;
        Task<string> pending;
        readonly StringBuilder stderr = new StringBuilder();
        Texture2D input, overlay, lane, obstacles;
        byte[] pendingPng;
        long pendingSequence;
        double pendingAt, requestAt, nextCapture, displayedAt, pendingSimulationTime;
        bool initialized, requestedLaneLastFrame;

        public void Initialize()
        {
            if (initialized) return;
            initialized = true;
            feed = GetComponent<AIFormulaRgbCameraTopic>();
            try {
                if (File.Exists(SettingsPath)) Configuration = JsonConvert.DeserializeObject<Settings>(File.ReadAllText(SettingsPath)) ?? new Settings();
                else {
                    string configured = Environment.GetEnvironmentVariable("AIFORMULA_PERCEPTION_PYTHON");
                    if (!string.IsNullOrWhiteSpace(configured)) Configuration.python = configured;
                }
                Validate(Configuration);
            } catch (Exception error) { Configuration = new Settings(); Status = "Settings: " + error.Message; }
        }
        public static void Validate(Settings value)
        {
            if (value == null || string.IsNullOrWhiteSpace(value.python) || value.python.IndexOfAny(new[]{'\r','\n','\"'}) >= 0)
                throw new ArgumentException("Choose a Python executable, without command arguments.");
            if (value.device != "auto" && value.device != "cpu" && value.device != "cuda") throw new ArgumentException("Device must be auto, cpu or cuda.");
            if (float.IsNaN(value.frequency) || float.IsInfinity(value.frequency) || value.frequency < 1 || value.frequency > 15)
                throw new ArgumentException("Perception rate must be 1 to 15 Hz.");
            if (value.output < 0 || value.output > 2) throw new ArgumentException("Unknown perception output.");
            if (float.IsNaN(value.visualizationScale) || float.IsInfinity(value.visualizationScale)
                || value.visualizationScale < .75f || value.visualizationScale > 2)
                throw new ArgumentException("Visualization scale must be 0.75 to 2.");
        }
        public void SaveSettings()
        {
            Validate(Configuration);
            Directory.CreateDirectory(Path.GetDirectoryName(SettingsPath));
            File.WriteAllText(SettingsPath, JsonConvert.SerializeObject(Configuration, Formatting.Indented));
        }
        public void SetRunning(bool value)
        {
            Initialize();
            if (!value) {
                // Withdraw the request first, including when process teardown throws.
                Requested = false; Faulted = false; Status = "Off";
                try {
                    var follower = GetComponent<AIFormulaVisionLaneFollower>();
                    if (follower && follower.Running) follower.Stop("Recognition disabled");
                    if (LaneControlEnabled) EnableLaneControl(false);
                    Shutdown();
                } finally { ClearImages(); }
                return;
            }
            if (Requested && !Faulted) return;
            Shutdown(); ClearImages(); Requested = true; Faulted = false; CompletedFrames = 0;
            try {
                if (!Application.isPlaying) throw new InvalidOperationException("Enter Play to enable perception.");
                if (!feed || !feed.rgbCamera) throw new InvalidOperationException("The existing RGB camera topic is not bound.");
                Validate(Configuration);
                string worker = Path.Combine(Application.streamingAssetsPath, "RoadDetector/worker.py");
                if (!File.Exists(worker)) throw new FileNotFoundException("Road-detector worker is missing.");
                var start = new ProcessStartInfo {
                    FileName = Configuration.python.Trim(), Arguments = "-B -u \"" + worker + "\" --device " + Configuration.device,
                    WorkingDirectory = Path.GetDirectoryName(worker), UseShellExecute = false, CreateNoWindow = true,
                    RedirectStandardInput = true, RedirectStandardOutput = true, RedirectStandardError = true,
                    StandardOutputEncoding = Encoding.UTF8, StandardErrorEncoding = Encoding.UTF8
                };
                start.EnvironmentVariables["PYTHONDONTWRITEBYTECODE"] = "1";
                start.EnvironmentVariables["PYTHONUTF8"] = "1";
                process = new Process { StartInfo = start };
                lock (stderr) stderr.Clear();
                process.ErrorDataReceived += (_, e) => { if (e.Data == null) return; lock (stderr) { stderr.AppendLine(e.Data); if (stderr.Length > 3000) stderr.Remove(0, stderr.Length - 3000); } };
                if (!process.Start()) throw new InvalidOperationException("Could not start Python.");
                process.BeginErrorReadLine();
                pending = Observe(process.StandardOutput.ReadLineAsync());
                requestAt = Time.realtimeSinceStartupAsDouble; Status = "Loading a3-v14";
            } catch (Exception error) { Fail(error.Message); }
        }
        public void Restart() { SetRunning(false); SetRunning(true); }
        public void EnableLaneControl(bool value)
        {
            Initialize();
            if (value && (!feed || !feed.useRepositoryIntrinsics))
                throw new InvalidOperationException("LYA requires the existing calibrated RGB projection.");
            LaneControlEnabled = value; LaneEpoch++; LaneControl = null;
        }

        JObject CameraCalibration()
        {
            var k = feed.repositoryIntrinsics;
            float sx = feed.width / 640f, sy = feed.height / 360f;
            var rotation = feed.vehicleLocalRotation;
            Vector3 right = rotation * Vector3.right, down = rotation * Vector3.down, forward = rotation * Vector3.forward;
            var t = feed.vehicleLocalPosition;
            return new JObject {
                ["K"] = new JArray(k.x*sx,0,k.z*sx,0,k.y*sy,k.w*sy,0,0,1),
                ["R"] = new JArray(right.z,down.z,forward.z,-right.x,-down.x,-forward.x,right.y,down.y,forward.y),
                ["t"] = new JArray(t.z,-t.x,t.y)
            };
        }

        void LateUpdate()
        {
            if (!Requested || Faulted) return;
            try {
                double now = Time.realtimeSinceStartupAsDouble;
                if (pending != null) {
                    if (!pending.IsCompleted) {
                        if (now - requestAt > (Ready ? 30 : 90)) throw new TimeoutException("Perception worker timed out.");
                        return;
                    }
                    string line = pending.GetAwaiter().GetResult(); pending = null;
                    if (line == null) { string detail; lock (stderr) detail = stderr.ToString(); throw new IOException("Python exited. " + detail); }
                    AcceptResponse(JObject.Parse(line));
                }
                if (!Ready || now < nextCapture) return;
                Validate(Configuration);
                var frame = feed.Capture();
                pendingAt = feed.CapturedAt; pendingSequence = feed.Sequence;
                pendingSimulationTime = Time.timeAsDouble;
                pendingPng = frame.EncodeToPNG();
                if (pendingPng == null || pendingPng.Length == 0) throw new IOException("RGB PNG encoding failed.");
                var message = new JObject { ["sequence"] = pendingSequence, ["png"] = Convert.ToBase64String(pendingPng) };
                message["signal_panel_rgb"] = Configuration.enableSignalPanelRgb;
                bool requestLane = LaneControlEnabled || LanePreviewRequested && CanProjectLanes;
                if (requestLane && !requestedLaneLastFrame && !LaneControlEnabled) LaneEpoch++;
                requestedLaneLastFrame = requestLane;
                if (requestLane) message["lane_control"] = new JObject {
                    ["camera"] = CameraCalibration(), ["sim_time"] = pendingSimulationTime, ["epoch"] = LaneEpoch,
                    ["recover_projected_components"] = RecoverProjectedComponents,
                    ["use_current_corridor"] = UseCurrentCorridor
                };
                var request = message.ToString(Formatting.None);
                requestAt = now; nextCapture = now + 1 / Configuration.frequency;
                var worker = process;
                pending = Observe(Task.Run(async () => {
                    await worker.StandardInput.WriteLineAsync(request).ConfigureAwait(false);
                    await worker.StandardInput.FlushAsync().ConfigureAwait(false);
                    return await worker.StandardOutput.ReadLineAsync().ConfigureAwait(false);
                }));
            } catch (Exception error) { Fail(error.GetBaseException().Message); }
        }
        static Task<string> Observe(Task<string> task)
        {
            task.ContinueWith(t => { var observed = t.Exception; }, TaskContinuationOptions.OnlyOnFaulted);
            return task;
        }
        void AcceptResponse(JObject reply)
        {
            if ((string)reply["type"] == "error") throw new InvalidOperationException((string)reply["message"]);
            if (!Ready) {
                if ((string)reply["type"] != "ready" || (string)reply["model"] != ModelTag || (string)reply["sha256"] != WeightSha256)
                    throw new InvalidDataException("Worker model identity mismatch.");
                Device = (string)reply["device"]; Ready = true; Status = "Waiting for RGB"; return;
            }
            if ((string)reply["type"] != "frame" || (long?)reply["sequence"] != pendingSequence || pendingPng == null)
                throw new InvalidDataException("Perception frame sequence mismatch.");
            using(var hash=SHA256.Create())
                if((string)reply["input_sha256"]!=BitConverter.ToString(hash.ComputeHash(pendingPng)).Replace("-","").ToLowerInvariant())
                    throw new InvalidDataException("Perception RGB checksum mismatch.");
            int width = (int)reply["width"], height = (int)reply["height"];
            if (width < 2 || height < 2 || width > 4096 || height > 4096) throw new InvalidDataException("Invalid output dimensions.");
            // Display input and output atomically only after matching this response.
            Load(ref input, pendingPng, "YOLOP input", width, height);
            Load(ref overlay, Convert.FromBase64String((string)reply["overlay"]), "YOLOP overlay", width, height);
            Load(ref lane, Convert.FromBase64String((string)reply["lane"]), "YOLOP lane", width, height);
            Load(ref obstacles, Convert.FromBase64String((string)reply["obstacles"]), "YOLOP obstacles", width, height);
            InferenceMilliseconds = (float)reply["inference_ms"];
            RoundTripMilliseconds = (float)((Time.realtimeSinceStartupAsDouble - pendingAt) * 1000);
            TrafficLight = (string)reply["traffic_light_state"]; DetectionCount = ((JArray)reply["detections"]).Count;
            FrameSequence = pendingSequence; displayedAt = pendingAt; pendingPng = null; CompletedFrames++;
            FrameSimulationTime = pendingSimulationTime;
            LaneControl = reply["lane_control"] as JObject;
            LastPayload = reply.ToString(Formatting.None); Status = "Running";
        }
        static void Load(ref Texture2D texture, byte[] png, string name, int width, int height)
        {
            if (!texture) texture = new Texture2D(2, 2, TextureFormat.RGB24, false) { name = name, hideFlags = HideFlags.DontSave };
            if (!texture.LoadImage(png, false) || texture.width != width || texture.height != height)
                throw new InvalidDataException("Perception image dimensions do not match RGB.");
        }
        void Fail(string message) { Shutdown(); ClearImages(); Faulted = true; Status = message; }
        void Shutdown()
        {
            Ready = false; pending = null; pendingPng = null; requestedLaneLastFrame = false;
            var worker = process; process = null;
            if (worker == null) return;
            try { if (!worker.HasExited) { worker.Kill(); worker.WaitForExit(200); } } catch (InvalidOperationException) { }
            finally { worker.Dispose(); }
        }
        void ClearImages()
        {
            foreach (var texture in new[]{input,overlay,lane,obstacles}) if (texture) { if (Application.isPlaying) Destroy(texture); else DestroyImmediate(texture); }
            input = overlay = lane = obstacles = null; FrameSequence = 0; LastPayload = null;
            LaneControl = null;
            TrafficLight = "unknown"; DetectionCount = 0;
        }
        void OnDisable() { SetRunning(false); }
        void OnDestroy() { Shutdown(); ClearImages(); }
        void OnApplicationQuit() { Shutdown(); }
    }
}
