// OpenArmController.cs - TCP 架構版本（修正版）
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;

public class OpenArmController : MonoBehaviour
{
    private ROSConnection rosConnection;
    private ROSTCPManager tcpManager;

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

    // 初始化重試計數
    private int initRetryCount = 0;
    private const int MAX_INIT_RETRIES = 5;

    void Start()
    {
        Debug.Log("🤖 OpenArmController (TCP) 啟動...");

        // 延遲初始化，確保其他組件準備就緒
        Invoke(nameof(InitializeController), 1.5f);
    }

    void InitializeController()
    {
        // 尋找 ROSTCPManager
        tcpManager = FindObjectOfType<ROSTCPManager>();
        if (tcpManager == null)
        {
            Debug.LogError("❌ OpenArmController: 找不到 ROSTCPManager！");

            // 重試邏輯
            initRetryCount++;
            if (initRetryCount < MAX_INIT_RETRIES)
            {
                Debug.LogWarning($"⚠️ 重試初始化 ({initRetryCount}/{MAX_INIT_RETRIES})...");
                Invoke(nameof(InitializeController), 2.0f);
            }
            else
            {
                Debug.LogError("❌ 達到最大重試次數，初始化失敗");
            }
            return;
        }

        Debug.Log("✅ 找到 ROSTCPManager，開始設置連接...");

        // 延遲初始化 TCP 連接
        Invoke(nameof(InitializeTCPConnection), 1.0f);
    }

    void InitializeTCPConnection()
    {
        // 獲取 ROS 連接
        rosConnection = ROSConnection.GetOrCreateInstance();

        if (rosConnection == null)
        {
            Debug.LogWarning("⚠️ OpenArmController: ROS 連接未準備就緒，稍後重試...");

            if (initRetryCount < MAX_INIT_RETRIES)
            {
                initRetryCount++;
                Invoke(nameof(InitializeTCPConnection), 2.0f);
            }
            return;
        }

        try
        {
            Debug.Log("🔄 OpenArmController: 初始化 TCP 連接...");

            // 直接訂閱關節狀態
            rosConnection.Subscribe<JointStateMsg>(jointStatesTopic, OnJointStatesReceived);
            Debug.Log($"✅ OpenArmController: 訂閱關節狀態主題: {jointStatesTopic}");

            // 訂閱狀態訊息
            rosConnection.Subscribe<StringMsg>(statusTopic, OnStatusReceived);
            Debug.Log($"✅ OpenArmController: 訂閱狀態主題: {statusTopic}");

            // 註冊發布者（預先註冊可以提高效能）
            rosConnection.RegisterPublisher<JointStateMsg>(jointCommandsTopic);
            Debug.Log($"✅ OpenArmController: 註冊發布者: {jointCommandsTopic}");

            isConnected = true;
            initRetryCount = 0;  // 重置重試計數
            Debug.Log("✅ OpenArmController: TCP 連接初始化完成");

            // 開始定期檢查連接狀態
            InvokeRepeating(nameof(CheckConnectionHealth), 5.0f, 5.0f);
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ OpenArmController: TCP 初始化失敗: {ex.Message}");
            Debug.LogError($"Stack trace: {ex.StackTrace}");

            // 延遲重試
            if (initRetryCount < MAX_INIT_RETRIES)
            {
                initRetryCount++;
                Invoke(nameof(InitializeTCPConnection), 3.0f);
            }
        }
    }

    void CheckConnectionHealth()
    {
        // 檢查是否正在接收資料
        bool wasReceiving = isReceivingStates;
        isReceivingStates = (Time.time - lastStateUpdateTime) < 5.0f;

        if (wasReceiving != isReceivingStates)
        {
            if (!isReceivingStates)
            {
                Debug.LogWarning("⚠️ OpenArmController: 超過5秒未收到關節狀態");
            }
            else
            {
                Debug.Log("✅ OpenArmController: 恢復接收關節狀態");
            }
        }
    }

    #region 訊息接收回調

    /// <summary>
    /// 接收關節狀態（直接訂閱或從 ROSTCPManager 廣播）
    /// </summary>
    public void OnJointStatesReceived(JointStateMsg jointMsg)
    {
        if (jointMsg == null)
        {
            Debug.LogWarning("⚠️ OpenArmController: 收到空的關節狀態訊息");
            return;
        }

        if (jointMsg.name == null || jointMsg.position == null)
        {
            Debug.LogWarning("⚠️ OpenArmController: 關節訊息格式不完整");
            return;
        }

        isReceivingStates = true;
        lastStateUpdateTime = Time.time;

        // 更新當前關節位置
        int updateCount = Mathf.Min(jointMsg.name.Length, currentJointPositions.Length);
        for (int i = 0; i < updateCount; i++)
        {
            if (i < jointMsg.position.Length)
            {
                currentJointPositions[i] = (float)jointMsg.position[i];
            }
        }

        Debug.Log($"📥 OpenArmController: 收到關節狀態 - {jointMsg.name.Length} 個關節");

        // 詳細記錄前3個關節的位置
        for (int i = 0; i < Mathf.Min(3, updateCount); i++)
        {
            Debug.Log($"   {jointMsg.name[i]}: {currentJointPositions[i]:F3} rad");
        }

        UpdateArmVisualization();
    }

    /// <summary>
    /// 接收狀態訊息
    /// </summary>
    public void OnStatusReceived(StringMsg statusMsg)
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
        // 例如：更新3D模型的關節角度

        // 現在只是記錄日誌
        if (Time.frameCount % 60 == 0)  // 每60幀記錄一次，避免過多日誌
        {
            Debug.Log($"🔄 OpenArmController: 更新視覺化");
        }
    }

    #endregion

    #region 公共控制方法

    /// <summary>
    /// 發送關節命令
    /// </summary>
    public void SendJointCommand(float[] jointPositions)
    {
        if (!isConnected)
        {
            Debug.LogWarning("⚠️ OpenArmController: 未連接，無法發送命令");
            return;
        }

        if (tcpManager == null)
        {
            Debug.LogWarning("⚠️ OpenArmController: TCPManager 未找到");

            // 嘗試重新尋找
            tcpManager = FindObjectOfType<ROSTCPManager>();
            if (tcpManager == null)
            {
                Debug.LogError("❌ OpenArmController: 無法找到 TCPManager");
                return;
            }
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
        Debug.Log("🔄 OpenArmController: 重置所有關節到零位");
    }

    /// <summary>
    /// 測試移動到預設位置
    /// </summary>
    [ContextMenu("測試移動")]
    public void TestMove()
    {
        float[] testPositions = { 0.1f, -0.1f, 0.2f, -0.2f, 0.1f, -0.1f };
        SendJointCommand(testPositions);
        Debug.Log("🔄 OpenArmController: 執行測試移動");
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

    /// <summary>
    /// 手動重新初始化連接
    /// </summary>
    [ContextMenu("重新初始化")]
    public void Reinitialize()
    {
        Debug.Log("🔄 OpenArmController: 手動重新初始化...");

        // 重置狀態
        isConnected = false;
        isReceivingStates = false;
        initRetryCount = 0;

        // 取消所有 Invoke
        CancelInvoke();

        // 重新開始初始化
        InitializeController();
    }

    #endregion

    #region GUI 顯示

    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(Screen.width - 300, 380, 280, 280));

        GUILayout.Label("OpenArm 控制器", GUI.skin.box);

        // 連接狀態
        GUI.color = isConnected ? Color.green : Color.red;
        GUILayout.Label($"連接: {(isConnected ? "✅" : "❌")}");

        GUI.color = IsReceivingStates() ? Color.green : Color.red;
        GUILayout.Label($"接收狀態: {(IsReceivingStates() ? "✅" : "❌")}");
        GUI.color = Color.white;

        // TCPManager 狀態
        if (tcpManager != null)
        {
            bool tcpConnected = tcpManager.IsConnected();
            GUI.color = tcpConnected ? Color.green : Color.yellow;
            GUILayout.Label($"TCPManager: {(tcpConnected ? "✅" : "⚠️")}");
            GUI.color = Color.white;
        }
        else
        {
            GUI.color = Color.red;
            GUILayout.Label("TCPManager: ❌ 未找到");
            GUI.color = Color.white;
        }

        // 最後狀態
        if (!string.IsNullOrEmpty(lastStatusMessage))
        {
            GUILayout.Label($"狀態: {lastStatusMessage}");
        }

        // 關節位置顯示
        GUILayout.Label("當前關節位置:");
        for (int i = 0; i < Mathf.Min(3, currentJointPositions.Length); i++)
        {
            GUILayout.Label($"  {jointNames[i]}: {currentJointPositions[i]:F2} rad");
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
            TestMove();
        }

        if (GUILayout.Button("重新初始化"))
        {
            Reinitialize();
        }
        GUILayout.EndHorizontal();

        // 重試狀態
        if (initRetryCount > 0)
        {
            GUI.color = Color.yellow;
            GUILayout.Label($"初始化重試: {initRetryCount}/{MAX_INIT_RETRIES}");
            GUI.color = Color.white;
        }

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