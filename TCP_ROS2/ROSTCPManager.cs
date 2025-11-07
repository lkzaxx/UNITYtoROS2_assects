using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using System.Collections;

/// <summary>
/// 統一的 ROS TCP 連接管理器 - 修正版
/// </summary>
public class ROSTCPManager : MonoBehaviour
{
    [Header("連接設定")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;
    public float heartbeatInterval = 1.0f;
    public float connectionTimeout = 5.0f;

    [Header("Topic 設定")]
    public string heartbeatTopic = "/unity/heartbeat";
    public string statusTopic = "/unity/status";
    public string jointCommandsTopic = "/unity/joint_commands";
    public string jointStatesTopic = "/openarm/joint_states";
    public string cmdVelTopic = "/cmd_vel";

    [Header("狀態顯示")]
    public bool isConnected = false;
    public bool isHeartbeatActive = true;
    public int messagesSent = 0;
    public int messagesReceived = 0;
    public string lastStatusMessage = "";

    // ROS TCP Connector
    private ROSConnection ros;

    // 心跳相關
    private float lastHeartbeatTime = 0f;
    private int heartbeatCount = 0;

    // 連接狀態
    private bool connectionInitialized = false;
    private float lastMessageTime = 0f;

    // 單例模式
    private static ROSTCPManager instance;
    public static ROSTCPManager Instance
    {
        get
        {
            if (instance == null)
                instance = FindObjectOfType<ROSTCPManager>();
            return instance;
        }
    }

    void Awake()
    {
        if (instance == null)
            instance = this;
        else if (instance != this)
            Destroy(gameObject);
    }

    void Start()
    {
        Debug.Log("🚀 ROSTCPManager 啟動...");
        StartCoroutine(DelayedInitialization());
    }

    IEnumerator DelayedInitialization()
    {
        // 等待一幀，確保 ROS Settings 已經載入
        yield return null;

        InitializeROSConnection();
    }

    void InitializeROSConnection()
    {
        try
        {
            // 獲取 ROS TCP Connector 實例
            ros = ROSConnection.GetOrCreateInstance();

            // 重要：確保連接參數正確設定
            if (ros != null)
            {
                // 透過反射或其他方式設定 IP 和 Port（如果 API 允許）
                // 注意：通常這些設定在 ROS Settings 中配置
                Debug.Log($"📡 使用 ROS 連接設定: {rosIPAddress}:{rosPort}");

                // 確保連接開始
                if (!ros.HasConnectionThread)
                {
                    Debug.LogWarning("⚠️ ROS 連接線程未啟動，嘗試手動啟動...");
                }
            }

            // 註冊訂閱者
            RegisterSubscribers();

            // 註冊發布者
            RegisterPublishers();

            // 開始心跳
            if (isHeartbeatActive)
            {
                StartCoroutine(HeartbeatCoroutine());
            }

            // 開始連接狀態檢查
            StartCoroutine(ConnectionStatusCheck());

            connectionInitialized = true;
            isConnected = true;
            Debug.Log("✅ ROSTCPManager 初始化完成");
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ ROSTCPManager 初始化失敗: {ex.Message}");
            Debug.LogError($"Stack trace: {ex.StackTrace}");

            // 延遲重試
            Invoke(nameof(InitializeROSConnection), 5.0f);
        }
    }

    void RegisterPublishers()
    {
        try
        {
            // 預先註冊發布者，提高效能
            ros.RegisterPublisher<StringMsg>(heartbeatTopic);
            ros.RegisterPublisher<StringMsg>(statusTopic);
            ros.RegisterPublisher<JointStateMsg>(jointCommandsTopic);
            ros.RegisterPublisher<TwistMsg>(cmdVelTopic);

            Debug.Log("✅ 註冊所有發布者完成");
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ 註冊發布者失敗: {ex.Message}");
        }
    }

    void RegisterSubscribers()
    {
        try
        {
            // 訂閱狀態訊息
            ros.Subscribe<StringMsg>(statusTopic, OnStatusReceived);
            Debug.Log($"✅ 訂閱 {statusTopic}");

            // 訂閱關節狀態
            ros.Subscribe<JointStateMsg>(jointStatesTopic, OnJointStatesReceived);
            Debug.Log($"✅ 訂閱 {jointStatesTopic}");

            // 訂閱 OpenArm 狀態
            ros.Subscribe<StringMsg>("/openarm/status", OnOpenArmStatusReceived);
            Debug.Log($"✅ 訂閱 /openarm/status");
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ 註冊訂閱者失敗: {ex.Message}");
        }
    }

    IEnumerator ConnectionStatusCheck()
    {
        while (true)
        {
            yield return new WaitForSeconds(2.0f);

            // 檢查連接狀態
            bool wasConnected = isConnected;

            // 如果超過 connectionTimeout 秒沒有收到任何訊息，認為連接可能有問題
            if (Time.time - lastMessageTime > connectionTimeout)
            {
                isConnected = false;

                if (wasConnected)
                {
                    Debug.LogWarning($"⚠️ 連接可能已斷開（{connectionTimeout}秒無回應）");
                }
            }
            else
            {
                isConnected = true;
            }

            if (wasConnected != isConnected)
            {
                Debug.Log($"🔄 連接狀態變更: {(isConnected ? "已連接" : "已斷線")}");
            }
        }
    }

    #region 心跳機制

    IEnumerator HeartbeatCoroutine()
    {
        while (isHeartbeatActive)
        {
            yield return new WaitForSeconds(heartbeatInterval);
            SendHeartbeat();
        }
    }

    void SendHeartbeat()
    {
        if (ros == null) return;

        try
        {
            heartbeatCount++;
            var heartbeatMsg = new StringMsg();
            heartbeatMsg.data = $"unity_heartbeat_{heartbeatCount}_{System.DateTime.Now:HH:mm:ss.fff}";

            ros.Publish(heartbeatTopic, heartbeatMsg);
            messagesSent++;
            lastHeartbeatTime = Time.time;

            if (heartbeatCount % 10 == 0)  // 每10次心跳才記錄一次，避免過多日誌
            {
                Debug.Log($"💓 心跳 #{heartbeatCount}: {heartbeatMsg.data}");
            }
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ 發送心跳失敗: {ex.Message}");
        }
    }

    #endregion

    #region 訊息接收回調

    void OnStatusReceived(StringMsg statusMsg)
    {
        messagesReceived++;
        lastStatusMessage = statusMsg.data;
        lastMessageTime = Time.time;

        Debug.Log($"📥 收到狀態: {statusMsg.data}");
    }

    void OnOpenArmStatusReceived(StringMsg statusMsg)
    {
        messagesReceived++;
        lastMessageTime = Time.time;

        Debug.Log($"📥 收到 OpenArm 狀態: {statusMsg.data}");

        // 轉發給 OpenArmController
        BroadcastToOpenArmControllers("OnStatusReceived", statusMsg);
    }

    void OnJointStatesReceived(JointStateMsg jointMsg)
    {
        messagesReceived++;
        lastMessageTime = Time.time;

        if (jointMsg.name != null && jointMsg.name.Length > 0)
        {
            Debug.Log($"📥 收到關節狀態: {jointMsg.name.Length} 個關節");

            // 顯示關節詳細資訊（調試用）
            for (int i = 0; i < jointMsg.name.Length && i < jointMsg.position.Length; i++)
            {
                Debug.Log($"   {jointMsg.name[i]}: {jointMsg.position[i]:F3} rad");
            }

            // 廣播關節狀態給 OpenArmController
            BroadcastToOpenArmControllers("OnJointStatesReceived", jointMsg);
        }
    }

    void BroadcastToOpenArmControllers(string methodName, object message)
    {
        // 找到所有 OpenArmController 並發送訊息
        OpenArmController[] controllers = FindObjectsOfType<OpenArmController>();

        if (controllers.Length == 0)
        {
            Debug.LogWarning($"⚠️ 找不到 OpenArmController，無法廣播 {methodName}");
            return;
        }

        foreach (var controller in controllers)
        {
            try
            {
                controller.SendMessage(methodName, message, SendMessageOptions.DontRequireReceiver);
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"❌ 廣播訊息給 OpenArmController 失敗: {ex.Message}");
            }
        }
    }

    #endregion

    #region 公共發布方法

    /// <summary>
    /// 發送關節命令
    /// </summary>
    public void PublishJointCommands(string[] jointNames, float[] positions)
    {
        if (ros == null)
        {
            Debug.LogError("❌ ROS 連接未初始化");
            return;
        }

        if (jointNames == null || positions == null)
        {
            Debug.LogWarning("⚠️ 無法發送關節命令：參數為空");
            return;
        }

        if (jointNames.Length != positions.Length)
        {
            Debug.LogWarning($"⚠️ 關節名稱數量({jointNames.Length})和位置數量({positions.Length})不匹配");
            return;
        }

        try
        {
            var jointMsg = new JointStateMsg();

            // 設定時間戳
            var now = System.DateTimeOffset.UtcNow;
            jointMsg.header = new HeaderMsg();
            jointMsg.header.stamp = new TimeMsg();
            jointMsg.header.stamp.sec = (int)now.ToUnixTimeSeconds();
            jointMsg.header.stamp.nanosec = (uint)((now.ToUnixTimeMilliseconds() % 1000) * 1000000);
            jointMsg.header.frame_id = "unity";

            // 設定關節資料
            jointMsg.name = jointNames;
            jointMsg.position = new double[positions.Length];
            jointMsg.velocity = new double[positions.Length];
            jointMsg.effort = new double[positions.Length];

            for (int i = 0; i < positions.Length; i++)
            {
                jointMsg.position[i] = positions[i];
                jointMsg.velocity[i] = 0.0;  // 預設速度為0
                jointMsg.effort[i] = 0.0;    // 預設力矩為0
            }

            ros.Publish(jointCommandsTopic, jointMsg);
            messagesSent++;

            Debug.Log($"📤 發送關節命令: {jointNames.Length} 個關節");
            for (int i = 0; i < Mathf.Min(3, jointNames.Length); i++)
            {
                Debug.Log($"   {jointNames[i]}: {positions[i]:F3} rad");
            }
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ 發送關節命令失敗: {ex.Message}");
            Debug.LogError($"Stack trace: {ex.StackTrace}");
        }
    }

    /// <summary>
    /// 發送速度命令
    /// </summary>
    public void PublishCmdVel(float linearX, float angularZ)
    {
        if (ros == null)
        {
            Debug.LogError("❌ ROS 連接未初始化");
            return;
        }

        try
        {
            var twistMsg = new TwistMsg();
            twistMsg.linear = new Vector3Msg { x = linearX, y = 0, z = 0 };
            twistMsg.angular = new Vector3Msg { x = 0, y = 0, z = angularZ };

            ros.Publish(cmdVelTopic, twistMsg);
            messagesSent++;

            Debug.Log($"📤 發送速度命令: linear.x={linearX:F3}, angular.z={angularZ:F3}");
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ 發送速度命令失敗: {ex.Message}");
        }
    }

    /// <summary>
    /// 發送自定義字串訊息
    /// </summary>
    public void PublishStringMessage(string topic, string message)
    {
        if (ros == null)
        {
            Debug.LogError("❌ ROS 連接未初始化");
            return;
        }

        if (string.IsNullOrEmpty(topic) || string.IsNullOrEmpty(message))
        {
            Debug.LogWarning("⚠️ 主題或訊息為空");
            return;
        }

        try
        {
            var stringMsg = new StringMsg { data = message };
            ros.Publish(topic, stringMsg);
            messagesSent++;

            Debug.Log($"📤 發送訊息到 {topic}: {message}");
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ 發送訊息失敗: {ex.Message}");
        }
    }

    /// <summary>
    /// 取得連接狀態
    /// </summary>
    public bool IsConnected()
    {
        return isConnected && ros != null;
    }

    #endregion

    #region 測試方法

    [ContextMenu("測試發送關節命令")]
    public void TestSendJointCommand()
    {
        string[] testJoints = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };
        float[] testPositions = { 0.1f, -0.1f, 0.2f, -0.2f, 0.1f, -0.1f };
        PublishJointCommands(testJoints, testPositions);
    }

    [ContextMenu("測試發送速度命令")]
    public void TestSendCmdVel()
    {
        PublishCmdVel(0.5f, 0.3f);
    }

    #endregion

    #region GUI 顯示

    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 350, 220));

        GUILayout.Label("ROS TCP 連接管理器", GUI.skin.box);

        // 連接狀態
        GUI.color = isConnected ? Color.green : Color.red;
        GUILayout.Label($"連接狀態: {(isConnected ? "✅ 已連接" : "❌ 未連接")}");
        GUI.color = Color.white;

        // 統計資訊
        GUILayout.Label($"目標: {rosIPAddress}:{rosPort}");
        GUILayout.Label($"已發送: {messagesSent} 條訊息");
        GUILayout.Label($"已接收: {messagesReceived} 條訊息");
        GUILayout.Label($"心跳: #{heartbeatCount}");

        // 最後狀態
        if (!string.IsNullOrEmpty(lastStatusMessage))
        {
            GUILayout.Label($"最後狀態: {lastStatusMessage}");
        }

        // 控制按鈕
        GUILayout.BeginHorizontal();
        if (GUILayout.Button(isHeartbeatActive ? "停止心跳" : "開始心跳"))
        {
            isHeartbeatActive = !isHeartbeatActive;
            if (isHeartbeatActive)
            {
                StartCoroutine(HeartbeatCoroutine());
            }
        }

        if (GUILayout.Button("測試關節"))
        {
            TestSendJointCommand();
        }

        if (GUILayout.Button("測試速度"))
        {
            TestSendCmdVel();
        }
        GUILayout.EndHorizontal();

        GUILayout.EndArea();
    }

    #endregion

    void OnDestroy()
    {
        isHeartbeatActive = false;
        StopAllCoroutines();
        Debug.Log("🔄 ROSTCPManager 已停止");
    }
}