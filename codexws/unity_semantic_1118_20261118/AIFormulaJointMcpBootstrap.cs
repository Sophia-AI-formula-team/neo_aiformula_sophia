using UnityEditor;
using UnityEngine;
using MCPForUnity.Editor.Services.Transport.Transports;

namespace AIFormula.Editor
{
    // Temporary local bootstrap after the project migration; no scene or global preferences change.
    [InitializeOnLoad]
    static class AIFormulaJointMcpBootstrap
    {
        static AIFormulaJointMcpBootstrap()
        {
            EditorApplication.update += StartOnce;
        }

        static void StartOnce()
        {
                if (EditorApplication.isCompiling || EditorApplication.isUpdating) return;
                EditorApplication.update -= StartOnce;
                Debug.Log("JOINT_MCP_BOOTSTRAP " + Application.dataPath);
                if (!string.Equals(Application.dataPath.Replace('\\', '/'), "E:/Mess/School/SOPHIA/Y4T1/Research/Demo_LaneLineTrainer/Simulator/Assets", System.StringComparison.OrdinalIgnoreCase)) return;
                if (!StdioBridgeHost.IsRunning) StdioBridgeHost.Start();
                Debug.Log("JOINT_MCP_BOOTSTRAP_PORT " + StdioBridgeHost.GetCurrentPort());
        }
    }
}
