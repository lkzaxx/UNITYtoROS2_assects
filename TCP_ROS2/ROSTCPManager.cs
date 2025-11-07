using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using System.Collections;

/// <summary>
/// 統一的 ROS TCP 連接管理器
/// 整合連接管理、訊息處理、心跳功能
/// 取代：UnityRos2Talker.cs, StatusSubscriber.cs, CmdVelPublisher.cs
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
    private float lastConnectionAttempt = 0f;
    
    void Start()
    {
        Debug.Log("🚀 ROSTCPManager 啟動...");
        InitializeROSConnection();
    }
    
    void InitializeROSConnection()
    {
        try
        {
            // 獲取 ROS TCP Connector 實例
            ros = ROSConnection.GetOrCreateInstance();
            
            // 設定連接參數
            ros.ConnectOnStart = true;
            
            Debug.Log($"📡 設定 ROS 連接: {rosIPAddress}:{rosPort}");
            
            // 註冊訂閱者
            RegisterSubscribers();
            
            // 開始心跳
            if (isHeartbeatActive)
            {
                StartCoroutine(HeartbeatCoroutine());
            }
            
            connectionInitialized = true;
            Debug.Log("✅ ROSTCPManager 初始化完成");
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ ROSTCPManager 初始化失敗: {ex.Message}");
            Debug.LogError($"Stack trace: {ex.StackTrace}");
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
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ 註冊訂閱者失敗: {ex.Message}");
        }
    }
    
    void Update()
    {
        // 檢查連接狀態
        CheckConnectionStatus();
    }
    
    void CheckConnectionStatus()
    {
        if (!connectionInitialized)
            return;
            
        // 簡單的連接狀態檢查
        // 如果超過 connectionTimeout 秒沒有收到任何訊息，認為連接可能有問題
        bool wasConnected = isConnected;
        isConnected = ros != null && Time.time - lastConnectionAttempt < connectionTimeout;
        
        if (wasConnected != isConnected)
        {
            Debug.Log($"🔄 連接狀態變更: {(isConnected ? "已連接" : "已斷線")}");
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
            heartbeatMsg.data = $"unity_heartbeat_{heartbeatCount}_{System.DateTime.Now:HH:mm:ss}";
            
            ros.Publish(heartbeatTopic, heartbeatMsg);
            messagesSent++;
            lastHeartbeatTime = Time.time;
            
            Debug.Log($"💓 發送心跳 #{heartbeatCount}: {heartbeatMsg.data}");
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
        lastConnectionAttempt = Time.time;
        
        Debug.Log($"📥 收到狀態: {statusMsg.data}");
    }
    
    void OnJointStatesReceived(JointStateMsg jointMsg)
    {
        messagesReceived++;
        lastConnectionAttempt = Time.time;
        
        if (jointMsg.name != null && jointMsg.name.Length > 0)
        {
            Debug.Log($"📥 收到關節狀態: {jointMsg.name.Length} 個關節");
            
            // 廣播關節狀態給其他組件
            BroadcastJointStates(jointMsg);
        }
    }
    
    void BroadcastJointStates(JointStateMsg jointMsg)
    {
        // 發送訊息給其他組件（例如 OpenArmController）
        gameObject.SendMessage("OnJointStatesReceived", jointMsg, SendMessageOptions.DontRequireReceiver);
    }
    
    #endregion
    
    #region 公共發布方法
    
    /// <summary>
    /// 發送關節命令
    /// </summary>
    public void PublishJointCommands(string[] jointNames, float[] positions)
    {
        if (ros == null || jointNames == null || positions == null)
        {
            Debug.LogWarning("⚠️ 無法發送關節命令：參數無效");
            return;
        }
        
        if (jointNames.Length != positions.Length)
        {
            Debug.LogWarning("⚠️ 關節名稱和位置數量不匹配");
            return;
        }
        
        try
        {
            var jointMsg = new JointStateMsg();
            var now = System.DateTimeOffset.UtcNow;
            jointMsg.header.stamp.sec = (uint)now.ToUnixTimeSeconds();
            jointMsg.header.stamp.nanosec = (uint)((now.ToUnixTimeMilliseconds() % 1000) * 1000000);
            jointMsg.name = jointNames;
            jointMsg.position = new double[positions.Length];
            
            for (int i = 0; i < positions.Length; i++)
            {
                jointMsg.position[i] = positions[i];
            }
            
            ros.Publish(jointCommandsTopic, jointMsg);
            messagesSent++;
            
            Debug.Log($"📤 發送關節命令: {jointNames.Length} 個關節");
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ 發送關節命令失敗: {ex.Message}");
        }
    }
    
    /// <summary>
    /// 發送速度命令
    /// </summary>
    public void PublishCmdVel(float linearX, float angularZ)
    {
        if (ros == null) return;
        
        try
        {
            var twistMsg = new TwistMsg();
            twistMsg.linear.x = linearX;
            twistMsg.linear.y = 0f;
            twistMsg.linear.z = 0f;
            twistMsg.angular.x = 0f;
            twistMsg.angular.y = 0f;
            twistMsg.angular.z = angularZ;
            
            ros.Publish(cmdVelTopic, twistMsg);
            messagesSent++;
            
            Debug.Log($"📤 發送速度命令: linear.x={linearX}, angular.z={angularZ}");
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
        if (ros == null || string.IsNullOrEmpty(topic) || string.IsNullOrEmpty(message))
            return;
            
        try
        {
            var stringMsg = new StringMsg();
            stringMsg.data = message;
            
            ros.Publish(topic, stringMsg);
            messagesSent++;
            
            Debug.Log($"📤 發送訊息到 {topic}: {message}");
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ 發送訊息失敗: {ex.Message}");
        }
    }
    
    #endregion
    
    #region GUI 顯示
    
    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 350, 200));
        
        GUILayout.Label("ROS TCP 連接管理器", GUI.skin.box);
        
        // 連接狀態
        GUI.color = isConnected ? Color.green : Color.red;
        GUILayout.Label($"連接狀態: {(isConnected ? "✅ 已連接" : "❌ 未連接")}");
        GUI.color = Color.white;
        
        // 統計資訊
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
        
        if (GUILayout.Button("測試速度"))
        {
            PublishCmdVel(0.1f, 0.2f);
        }
        GUILayout.EndHorizontal();
        
        GUILayout.EndArea();
    }
    
    #endregion
    
    void OnDestroy()
    {
        isHeartbeatActive = false;
        Debug.Log("🔄 ROSTCPManager 已停止");
    }
}
