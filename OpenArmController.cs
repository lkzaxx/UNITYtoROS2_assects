// OpenArmController.cs - TCP 架構版本
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;

public class OpenArmController : MonoBehaviour
{
    private ROSTCPManager tcpManager;
    private ROSConnection rosConnection;
    
    [Header("ROS TCP 設定")]
    public string jointCommandsTopic = "/unity/joint_commands";
    public string jointStatesTopic = "/openarm/joint_states";
    public string statusTopic = "/openarm/status";
    [Header("機械手臂設定")]
    public string[] jointNames = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };
    public float[] currentJointPositions = new float[6];
    public float[] targetJointPositions = new float[6];
    
    [Header("狀態顯示")]
    public bool isConnected = false;
    public bool isReceivingStates = false;
    public string lastStatusMessage = "";
    public float lastStateUpdateTime = 0f;
    
    void Start()
    {
        Debug.Log("🤖 OpenArmController (TCP) 啟動...");
        
        // 尋找 ROSTCPManager
        tcpManager = FindFirstObjectByType<ROSTCPManager>();
        if (tcpManager == null)
        {
            Debug.LogError("❌ OpenArmController: 找不到 ROSTCPManager！請確保場景中有 ROSTCPManager 組件。");
            return;
        }
        
        // 獲取 ROS 連接
        rosConnection = ROSConnection.GetOrCreateInstance();
        
        // 延遲初始化，確保 TCP 連接準備就緒
        Invoke(nameof(InitializeTCPConnection), 2.0f);
    }
    
    void InitializeTCPConnection()
    {
        if (rosConnection == null)
        {
            Debug.LogWarning("⚠️ OpenArmController: ROS 連接未準備就緒，稍後重試...");
            Invoke(nameof(InitializeTCPConnection), 2.0f);
            return;
        }
        
        try
        {
            Debug.Log("🔄 OpenArmController: 初始化 TCP 連接...");
            
            // 訂閱關節狀態（透過 ROSTCPManager 的回調機制）
            // ROSTCPManager 會自動將接收到的關節狀態廣播給這個組件
            Debug.Log($"✅ OpenArmController: 準備接收關節狀態從 {jointStatesTopic}");
            
            // 訂閱狀態訊息
            rosConnection.Subscribe<StringMsg>(statusTopic, OnStatusReceived);
            Debug.Log($"✅ OpenArmController: 訂閱狀態主題: {statusTopic}");
            
            isConnected = true;
            Debug.Log("✅ OpenArmController: TCP 連接初始化完成");
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ OpenArmController: TCP 初始化失敗: {ex.Message}");
            Debug.LogError($"Stack trace: {ex.StackTrace}");
            
            // 延遲重試
            Invoke(nameof(InitializeTCPConnection), 5.0f);
        }
    }
    
    #region 訊息接收回調
    
    /// <summary>
    /// 接收關節狀態（由 ROSTCPManager 廣播）
    /// </summary>
    void OnJointStatesReceived(JointStateMsg jointMsg)
    {
        if (jointMsg == null || jointMsg.name == null || jointMsg.position == null)
            return;
            
        isReceivingStates = true;
        lastStateUpdateTime = Time.time;
        
        // 更新當前關節位置
        for (int i = 0; i < jointMsg.name.Length && i < currentJointPositions.Length; i++)
        {
            if (i < jointMsg.position.Length)
            {
                currentJointPositions[i] = (float)jointMsg.position[i];
            }
        }
        
        Debug.Log($"📥 OpenArmController: 收到關節狀態 - {jointMsg.name.Length} 個關節");
        UpdateArmVisualization();
    }
    
    /// <summary>
    /// 接收狀態訊息
    /// </summary>
    void OnStatusReceived(StringMsg statusMsg)
    {
        if (statusMsg != null && !string.IsNullOrEmpty(statusMsg.data))
        {
            lastStatusMessage = statusMsg.data;
            Debug.Log($"📥 OpenArmController: 收到狀態: {statusMsg.data}");
        }
    }
    
    void UpdateArmVisualization()
    {
        // TODO: 在這裡實現機械手臂視覺化更新
        // 例如：更新關節角度、位置等
        Debug.Log($"🔄 OpenArmController: 更新關節視覺化");
        
        for (int i = 0; i < currentJointPositions.Length; i++)
        {
            Debug.Log($"  關節 {i} ({jointNames[i]}): {currentJointPositions[i]:F3} rad");
        }
    }
    
    #endregion
    
    #region 公共控制方法
    
    /// <summary>
    /// 發送關節命令
    /// </summary>
    public void SendJointCommand(float[] jointPositions)
    {
        if (tcpManager == null)
        {
            Debug.LogWarning("⚠️ OpenArmController: TCPManager 未找到，無法發送命令");
            return;
        }
        
        if (jointPositions == null || jointPositions.Length != jointNames.Length)
        {
            Debug.LogWarning($"⚠️ OpenArmController: 關節位置數量不匹配 (需要 {jointNames.Length} 個)");
            return;
        }
        
        try
        {
            // 更新目標位置
            for (int i = 0; i < jointPositions.Length; i++)
            {
                targetJointPositions[i] = jointPositions[i];
            }
            
            // 透過 TCPManager 發送關節命令
            tcpManager.PublishJointCommands(jointNames, jointPositions);
            Debug.Log($"📤 OpenArmController: 發送關節命令 - {jointPositions.Length} 個關節");
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ OpenArmController: 發送命令失敗: {ex.Message}");
        }
    }
    
    /// <summary>
    /// 發送單個關節命令
    /// </summary>
    public void SendSingleJointCommand(int jointIndex, float position)
    {
        if (jointIndex < 0 || jointIndex >= targetJointPositions.Length)
        {
            Debug.LogWarning($"⚠️ OpenArmController: 關節索引超出範圍: {jointIndex}");
            return;
        }
        
        targetJointPositions[jointIndex] = position;
        SendJointCommand(targetJointPositions);
    }
    
    /// <summary>
    /// 重置所有關節到零位
    /// </summary>
    [ContextMenu("重置關節位置")]
    public void ResetJointPositions()
    {
        float[] zeroPositions = new float[jointNames.Length];
        SendJointCommand(zeroPositions);
    }
    
    /// <summary>
    /// 獲取當前關節位置
    /// </summary>
    public float[] GetCurrentJointPositions()
    {
        return (float[])currentJointPositions.Clone();
    }
    
    /// <summary>
    /// 檢查是否正在接收關節狀態
    /// </summary>
    public bool IsReceivingStates()
    {
        return isReceivingStates && (Time.time - lastStateUpdateTime) < 5.0f;
    }
    
    #endregion
    
    #region GUI 顯示
    
    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(Screen.width - 300, 380, 280, 250));
        
        GUILayout.Label("OpenArm 控制器", GUI.skin.box);
        
        // 連接狀態
        GUI.color = isConnected ? Color.green : Color.red;
        GUILayout.Label($"連接: {(isConnected ? "✅" : "❌")}");
        
        GUI.color = IsReceivingStates() ? Color.green : Color.red;
        GUILayout.Label($"接收狀態: {(IsReceivingStates() ? "✅" : "❌")}");
        GUI.color = Color.white;
        
        // 最後狀態
        if (!string.IsNullOrEmpty(lastStatusMessage))
        {
            GUILayout.Label($"狀態: {lastStatusMessage}");
        }
        
        // 關節位置顯示（簡化版）
        GUILayout.Label("當前關節位置:");
        for (int i = 0; i < System.Math.Min(3, currentJointPositions.Length); i++)
        {
            GUILayout.Label($"  {jointNames[i]}: {currentJointPositions[i]:F2}");
        }
        
        if (currentJointPositions.Length > 3)
        {
            GUILayout.Label($"  ... 共 {currentJointPositions.Length} 個關節");
        }
        
        // 控制按鈕
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("重置"))
        {
            ResetJointPositions();
        }
        
        if (GUILayout.Button("測試移動"))
        {
            float[] testPositions = { 0.1f, -0.1f, 0.2f, -0.2f, 0.1f, -0.1f };
            SendJointCommand(testPositions);
        }
        GUILayout.EndHorizontal();
        
        GUILayout.EndArea();
    }
    
    #endregion
    
    void OnDestroy()
    {
        // 清理資源
        CancelInvoke();
        Debug.Log("🔄 OpenArmController 已停止");
    }
}