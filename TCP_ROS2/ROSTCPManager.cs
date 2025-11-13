using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using System.Collections;
using UnityEngine.UI;
using TMPro;
using UnityEngine.EventSystems;

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

    [Header("Topic 設定 - 接收 (ROS2 → Unity)")]
    public string heartbeatTopic = "/unity/heartbeat";
    public string jointStatesTopic = "/openarm/joint_states";
    public string endEffectorPoseTopic = "/openarm/end_effector_pose";
    public string openarmStatusTopic = "/openarm/status";

    [Header("Topic 設定 - 發送 (Unity → ROS2)")]
    public string jointCommandsTopic = "/unity/joint_commands";
    public string unityPoseTopic = "/unity/pose";
    public string cmdVelTopic = "/openarm/cmd_vel";

    [Header("OpenArm Retarget 自動發送")]
    public OpenArmRetarget retarget;                 // OpenArmRetarget 引用
    public bool autoSendJointStates = false;         // 是否自動發送關節狀態
    public float jointStateSendInterval = 0.1f;      // 發送間隔（秒）
    public bool showJointValuesOnScreen = true;      // 在螢幕上顯示關節值
    [Tooltip("左臂關節名稱（7個）")]
    public string[] leftJointNames = new string[7] {
        "L_J1", "L_J2", "L_J3", "L_J4", "L_J5", "L_J6", "L_J7"
    };
    [Tooltip("右臂關節名稱（7個）")]
    public string[] rightJointNames = new string[7] {
        "R_J1", "R_J2", "R_J3", "R_J4", "R_J5", "R_J6", "R_J7"
    };

    [Header("Gripper (Prismatic) → JointState")]
    [Tooltip("左手夾爪（GripperHoldToOpenPrismatic）")]
    public GripperHoldToOpenPrismatic leftGripper;
    [Tooltip("右手夾爪（GripperHoldToOpenPrismatic）")]
    public GripperHoldToOpenPrismatic rightGripper;
    public bool autoSendGripperEE = true;            // 是否自動發送 L_EE/R_EE
    public float gripperSendInterval = 0.05f;        // 發送間隔（秒）
    [Tooltip("夾爪 JointState 名稱")]
    public string leftEEName = "L_EE";
    public string rightEEName = "R_EE";
    [Tooltip("夾爪行程限制（公尺）")]
    public float gripperMin = 0f;
    public float gripperMax = 0.0425f;

    [Header("狀態顯示")]
    public bool isConnected = false;
    public bool isHeartbeatActive = true;
    public int messagesSent = 0;
    public int messagesReceived = 0;
    public string lastStatusMessage = "";

    [Header("VR IP 配置界面")]
    [Tooltip("是否顯示 IP 配置界面")]
    public bool showIPConfigUI = true;
    [Tooltip("IP 配置 Canvas Prefab（可選，留空則動態創建）")]
    public GameObject ipConfigCanvasPrefab;
    [Tooltip("虛擬鍵盤 Prefab（可選）")]
    public GameObject virtualKeyboardPrefab;
    [Tooltip("界面位置（相對於主攝像機）")]
    public Vector3 uiPosition = new Vector3(0, 1.6f, 2f);
    [Tooltip("界面縮放")]
    public Vector3 uiScale = new Vector3(0.001f, 0.001f, 0.001f);

    // ROS TCP Connector
    private ROSConnection ros;

    // 心跳相關
    private float lastHeartbeatTime = 0f;
    private int heartbeatCount = 0;

    // 連接狀態
    private bool connectionInitialized = false;
    private float lastMessageTime = 0f;

    // 關節狀態發送
    private float lastJointStateSendTime = 0f;
    private float lastGripperSendTime = 0f;

    // IP 配置界面相關
    private GameObject ipConfigCanvasInstance;
    private TMP_InputField ipAddressInputField;
    private TMP_InputField portInputField;
    private Button applyButton;
    private Button cancelButton;
    private Button toggleButton;
    private VirtualKeyboard virtualKeyboard;
    private bool isIPConfigUIVisible = false;
    private string tempIPAddress;
    private int tempPort;

    // OpenArm 關節上下限（弧度）- 根據實際硬體規格
    private readonly float[] jointMinLimits = new float[7] {
        -3.49f,   // J1
        -3.32f,   // J2
        -1.57f,   // J3
        0f,       // J4
        -1.57f,   // J5
        -0.79f,   // J6
        -1.57f    // J7
    };
    private readonly float[] jointMaxLimits = new float[7] {
        3.48f,    // J1
        3.28f,    // J2
        1.50f,    // J3
        2.4f,     // J4
        1.50f,    // J5
        0.71f,    // J6
        1.50f     // J7
    };

    // 單例模式
    private static ROSTCPManager instance;
    public static ROSTCPManager Instance
    {
        get
        {
            if (instance == null)
                instance = FindFirstObjectByType<ROSTCPManager>();
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

        // 初始化 IP 配置界面
        if (showIPConfigUI)
        {
            InitializeIPConfigUI();
        }

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

            // 啟用自動發送關節狀態
            if (autoSendJointStates && retarget != null)
            {
                Debug.Log("✅ 啟用 OpenArmRetarget 自動發送");
            }
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
            ros.RegisterPublisher<JointStateMsg>(jointCommandsTopic);
            ros.RegisterPublisher<PoseStampedMsg>(unityPoseTopic);
            ros.RegisterPublisher<TwistMsg>(cmdVelTopic);

            Debug.Log("✅ 註冊所有發布者完成");
            Debug.Log($"   - 心跳: {heartbeatTopic}");
            Debug.Log($"   - 關節命令: {jointCommandsTopic}");
            Debug.Log($"   - Unity位置: {unityPoseTopic}");
            Debug.Log($"   - 速度命令: {cmdVelTopic}");
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
            // 訂閱關節狀態 (10Hz)
            ros.Subscribe<JointStateMsg>(jointStatesTopic, OnJointStatesReceived);
            Debug.Log($"✅ 訂閱關節狀態: {jointStatesTopic}");

            // 訂閱 OpenArm 系統狀態  
            ros.Subscribe<StringMsg>(openarmStatusTopic, OnOpenArmStatusReceived);
            Debug.Log($"✅ 訂閱系統狀態: {openarmStatusTopic}");

            // 訂閱末端執行器位置
            ros.Subscribe<PoseStampedMsg>(endEffectorPoseTopic, OnEndEffectorPoseReceived);
            Debug.Log($"✅ 訂閱末端執行器位置: {endEffectorPoseTopic}");

            // 可選：訂閱心跳回音（用於連接測試）
            ros.Subscribe<StringMsg>(heartbeatTopic + "_echo", OnHeartbeatEchoReceived);
            Debug.Log($"✅ 訂閱心跳回音: {heartbeatTopic}_echo");
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

    void OnEndEffectorPoseReceived(PoseStampedMsg poseMsg)
    {
        messagesReceived++;
        lastMessageTime = Time.time;

        if (poseMsg?.pose != null)
        {
            var pos = poseMsg.pose.position;
            var rot = poseMsg.pose.orientation;

            Debug.Log($"📥 收到末端執行器位置: Pos({pos.x:F3}, {pos.y:F3}, {pos.z:F3}) " +
                     $"Rot({rot.x:F3}, {rot.y:F3}, {rot.z:F3}, {rot.w:F3})");

            // 廣播末端執行器位置給 OpenArmController
            BroadcastToOpenArmControllers("OnEndEffectorPoseReceived", poseMsg);
        }
    }

    void OnHeartbeatEchoReceived(StringMsg echoMsg)
    {
        messagesReceived++;
        lastMessageTime = Time.time;

        Debug.Log($"📥 收到心跳回音: {echoMsg.data}");

        // 心跳回音確認連接正常
        isConnected = true;
    }

    void BroadcastToOpenArmControllers(string methodName, object message)
    {
        // 找到所有 OpenArmController 並發送訊息
        OpenArmController[] controllers = FindObjectsByType<OpenArmController>(FindObjectsSortMode.None);

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

        if (jointNames == null || positions == null || jointNames.Length != positions.Length)
        {
            Debug.LogError("❌ 關節名稱和位置數量不匹配");
            return;
        }

        try
        {
            var jointMsg = new JointStateMsg();

            // 設定訊息標頭
            var now = System.DateTimeOffset.Now;
            jointMsg.header = new HeaderMsg();
            jointMsg.header.stamp = new TimeMsg();
            jointMsg.header.stamp.sec = (int)now.ToUnixTimeSeconds();
            // 使用明確的 uint 轉換（nanosec 是 uint 類型）
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
    /// 發送Unity位置命令
    /// </summary>
    public void PublishUnityPose(Vector3 position, Quaternion rotation)
    {
        if (ros == null)
        {
            Debug.LogError("❌ ROS 連接未初始化");
            return;
        }

        try
        {
            var poseMsg = new PoseStampedMsg();

            // 設定訊息標頭
            var now = System.DateTimeOffset.Now;
            poseMsg.header = new HeaderMsg();
            poseMsg.header.stamp = new TimeMsg();
            poseMsg.header.stamp.sec = (int)now.ToUnixTimeSeconds();
            poseMsg.header.stamp.nanosec = (uint)((now.ToUnixTimeMilliseconds() % 1000) * 1000000);
            poseMsg.header.frame_id = "unity";

            // 設定位置和旋轉
            poseMsg.pose = new PoseMsg();
            poseMsg.pose.position = new PointMsg { x = position.x, y = position.y, z = position.z };
            poseMsg.pose.orientation = new QuaternionMsg { x = rotation.x, y = rotation.y, z = rotation.z, w = rotation.w };

            ros.Publish(unityPoseTopic, poseMsg);
            messagesSent++;

            Debug.Log($"📤 發送Unity位置: Pos({position.x:F3}, {position.y:F3}, {position.z:F3}) " +
                     $"Rot({rotation.x:F3}, {rotation.y:F3}, {rotation.z:F3}, {rotation.w:F3})");
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ 發送Unity位置失敗: {ex.Message}");
        }
    }

    /// <summary>
    /// 從左右 GripperHoldToOpenPrismatic 讀取目標位置（公尺），並以 JointState 發送 (L_EE, R_EE)
    /// </summary>
    void PublishGripperEEJointState()
    {
        if (ros == null) return;

        float left = GetJawTargetMeters(leftGripper, leftGripper != null ? leftGripper.leftJaw : null);
        float right = GetJawTargetMeters(rightGripper, rightGripper != null ? rightGripper.leftJaw : null);

        // 夾爪行程限制（0 ~ 0.0425 m）
        left = Mathf.Clamp(left, gripperMin, gripperMax);
        right = Mathf.Clamp(right, gripperMin, gripperMax);

        try
        {
            var jointMsg = new JointStateMsg();

            var now = System.DateTimeOffset.Now;
            jointMsg.header = new HeaderMsg();
            jointMsg.header.stamp = new TimeMsg();
            jointMsg.header.stamp.sec = (int)now.ToUnixTimeSeconds();
            jointMsg.header.stamp.nanosec = (uint)((now.ToUnixTimeMilliseconds() % 1000) * 1000000);
            jointMsg.header.frame_id = "unity";

            jointMsg.name = new string[2] { leftEEName, rightEEName };
            jointMsg.position = new double[2] { left, right };   // 單位：公尺
            jointMsg.velocity = new double[2] { 0.0, 0.0 };
            jointMsg.effort = new double[2] { 0.0, 0.0 };

            ros.Publish(jointCommandsTopic, jointMsg);
            messagesSent++;

            Debug.Log($"📤 發送夾爪 JointState: {leftEEName}={left:F4} m, {rightEEName}={right:F4} m");
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ 發送夾爪 JointState 失敗: {ex.Message}");
        }
    }

    /// <summary>
    /// 根據 gripper 設定的軸向，讀取 ArticulationBody 對應 Drive 的 target（公尺）
    /// </summary>
    float GetJawTargetMeters(GripperHoldToOpenPrismatic gripperRef, ArticulationBody jaw)
    {
        if (jaw == null || gripperRef == null) return 0f;
        switch (gripperRef.axis)
        {
            case GripperHoldToOpenPrismatic.Axis.X:
                return jaw.xDrive.target;
            case GripperHoldToOpenPrismatic.Axis.Y:
                return jaw.yDrive.target;
            case GripperHoldToOpenPrismatic.Axis.Z:
                return jaw.zDrive.target;
            default:
                return 0f;
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

    [ContextMenu("診斷連接狀態")]
    public void DiagnoseConnection()
    {
        Debug.Log("=== ROS TCP 連接診斷 ===");
        Debug.Log($"ROS IP: {rosIPAddress}:{rosPort}");
        Debug.Log($"ROS Connection Instance: {(ros != null ? "存在" : "null")}");

        if (ros != null)
        {
            Debug.Log($"Has Connection Thread: {ros.HasConnectionThread}");
            Debug.Log($"發送訊息數: {messagesSent}");
            Debug.Log($"接收訊息數: {messagesReceived}");
            Debug.Log($"最後訊息時間: {(Time.time - lastMessageTime):F1}秒前");
        }

        Debug.Log("=== Topic 配置 ===");
        Debug.Log($"心跳 Topic: {heartbeatTopic}");
        Debug.Log($"系統狀態 Topic: {openarmStatusTopic}");
        Debug.Log($"關節命令 Topic: {jointCommandsTopic}");
        Debug.Log($"關節狀態 Topic: {jointStatesTopic}");
        Debug.Log($"速度命令 Topic: {cmdVelTopic}");

        Debug.Log("=== 建議檢查 ===");
        Debug.Log("1. 確認 ROS2 ros_tcp_bridge 正在運行");
        Debug.Log("2. 檢查 ROS2 節點是否發布到正確的 topics");
        Debug.Log("3. 使用 'ros2 topic list' 查看可用的 topics");
        Debug.Log("4. 使用 'ros2 topic echo /topic_name' 測試訊息");
    }

    [ContextMenu("測試接收回音")]
    public void TestEcho()
    {
        // 發送測試訊息到狀態topic，看是否有回音
        PublishStringMessage(openarmStatusTopic, "unity_test_echo");
        Debug.Log($"📤 發送測試回音到 {openarmStatusTopic}");
    }

    [ContextMenu("測試Unity位置命令")]
    public void TestUnityPose()
    {
        Vector3 testPos = new Vector3(1.0f, 2.0f, 3.0f);
        Quaternion testRot = Quaternion.Euler(0, 45, 0);
        PublishUnityPose(testPos, testRot);
    }

    [ContextMenu("驗證所有Topic配置")]
    public void VerifyTopicConfiguration()
    {
        Debug.Log("=== Topic 配置驗證 ===");
        Debug.Log("接收端 (ROS2 → Unity):");
        Debug.Log($"  心跳: {heartbeatTopic}");
        Debug.Log($"  關節狀態: {jointStatesTopic}");
        Debug.Log($"  末端執行器位置: {endEffectorPoseTopic}");
        Debug.Log($"  系統狀態: {openarmStatusTopic}");

        Debug.Log("發送端 (Unity → ROS2):");
        Debug.Log($"  關節命令: {jointCommandsTopic}");
        Debug.Log($"  Unity位置: {unityPoseTopic}");
        Debug.Log($"  速度命令: {cmdVelTopic}");
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

        // 診斷按鈕
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("診斷連接"))
        {
            DiagnoseConnection();
        }

        if (GUILayout.Button("驗證配置"))
        {
            VerifyTopicConfiguration();
        }
        GUILayout.EndHorizontal();

        // 新功能測試按鈕
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("測試位置"))
        {
            TestUnityPose();
        }
        GUILayout.EndHorizontal();

        // 連接問題提示
        if (messagesSent > 0 && messagesReceived == 0)
        {
            GUI.color = Color.yellow;
            GUILayout.Label("⚠️ 只能發送無法接收，請檢查ROS2端");
            GUI.color = Color.white;
        }

        GUILayout.EndArea();

        // 顯示關節值面板
        if (showJointValuesOnScreen && retarget != null)
        {
            DrawJointValuesPanel();
        }
    }

    /// <summary>
    /// 繪製關節值顯示面板
    /// </summary>
    void DrawJointValuesPanel()
    {
        // 面板位置和大小（左下角）
        float panelX = 10;
        float panelY = Screen.height - 290;
        float panelWidth = 820;  // 增加寬度以容納兩列
        float panelHeight = 310; // 增加高度以容納夾爪顯示

        GUILayout.BeginArea(new Rect(panelX, panelY, panelWidth, panelHeight));

        // 標題
        GUI.color = Color.cyan;
        GUILayout.Label("OpenArm 關節角度監控", GUI.skin.box);
        GUI.color = Color.white;

        // 夾爪顯示（置於面板上方區域）
        GUILayout.BeginVertical(GUILayout.Width(panelWidth - 20));
        GUILayout.Label("夾爪 (Grippers):", EditorGUIStyle());
        {
            // 左夾爪
            if (leftGripper != null && leftGripper.leftJaw != null)
            {
                float leftMeters = GetJawTargetMeters(leftGripper, leftGripper.leftJaw);
                bool leftOut = leftMeters < gripperMin - 1e-5f || leftMeters > gripperMax + 1e-5f;
                float leftClamped = Mathf.Clamp(leftMeters, gripperMin, gripperMax);
                GUI.color = leftOut ? Color.red : Color.green;
                GUILayout.Label($"  {leftEEName} = {leftClamped,6:F4} m {(leftOut ? "[超出範圍]" : "")}");
                GUI.color = Color.white;
            }
            else
            {
                GUI.color = Color.gray;
                GUILayout.Label($"  {leftEEName} = 未設定");
                GUI.color = Color.white;
            }

            // 右夾爪
            if (rightGripper != null && rightGripper.leftJaw != null)
            {
                float rightMeters = GetJawTargetMeters(rightGripper, rightGripper.leftJaw);
                bool rightOut = rightMeters < gripperMin - 1e-5f || rightMeters > gripperMax + 1e-5f;
                float rightClamped = Mathf.Clamp(rightMeters, gripperMin, gripperMax);
                GUI.color = rightOut ? Color.red : Color.green;
                GUILayout.Label($"  {rightEEName} = {rightClamped,6:F4} m {(rightOut ? "[超出範圍]" : "")}");
                GUI.color = Color.white;
            }
            else
            {
                GUI.color = Color.gray;
                GUILayout.Label($"  {rightEEName} = 未設定");
                GUI.color = Color.white;
            }
        }
        GUILayout.EndVertical();

        GUILayout.Space(4);

        // 並排顯示左右臂
        GUILayout.BeginHorizontal();

        // 左臂關節值（左側）
        GUILayout.BeginVertical(GUILayout.Width(400));
        GUILayout.Label("左臂 (Left Arm):", EditorGUIStyle());
        if (retarget.left != null && retarget.left.Length > 0)
        {
            for (int i = 0; i < retarget.left.Length; i++)
            {
                if (retarget.left[i]?.joint != null)
                {
                    var drive = retarget.left[i].joint.xDrive;
                    float angleDeg = drive.target;
                    float angleRad = angleDeg * Mathf.Deg2Rad;

                    // 檢查是否超出範圍
                    bool outOfRange = false;
                    string rangeStatus = "";
                    if (i < jointMinLimits.Length)
                    {
                        if (angleRad < jointMinLimits[i])
                        {
                            outOfRange = true;
                            rangeStatus = " [低於下限]";
                        }
                        else if (angleRad > jointMaxLimits[i])
                        {
                            outOfRange = true;
                            rangeStatus = " [高於上限]";
                        }
                    }

                    GUI.color = outOfRange ? Color.red : Color.green;
                    GUILayout.Label($"  L_J{i + 1} = {angleDeg,7:F2}° ({angleRad,6:F3} rad){rangeStatus}");
                    GUI.color = Color.white;
                }
                else
                {
                    GUI.color = Color.gray;
                    GUILayout.Label($"  L_J{i + 1} = 未連接");
                    GUI.color = Color.white;
                }
            }
        }
        else
        {
            GUILayout.Label("  左臂未設定");
        }
        GUILayout.EndVertical();

        // 右臂關節值（右側）
        GUILayout.BeginVertical(GUILayout.Width(400));
        GUILayout.Label("右臂 (Right Arm):", EditorGUIStyle());
        if (retarget.right != null && retarget.right.Length > 0)
        {
            for (int i = 0; i < retarget.right.Length; i++)
            {
                if (retarget.right[i]?.joint != null)
                {
                    var drive = retarget.right[i].joint.xDrive;
                    float angleDeg = drive.target;
                    float angleRad = angleDeg * Mathf.Deg2Rad;

                    // 檢查是否超出範圍
                    bool outOfRange = false;
                    string rangeStatus = "";
                    if (i < jointMinLimits.Length)
                    {
                        if (angleRad < jointMinLimits[i])
                        {
                            outOfRange = true;
                            rangeStatus = " [低於下限]";
                        }
                        else if (angleRad > jointMaxLimits[i])
                        {
                            outOfRange = true;
                            rangeStatus = " [高於上限]";
                        }
                    }

                    GUI.color = outOfRange ? Color.red : Color.green;
                    GUILayout.Label($"  R_J{i + 1} = {angleDeg,7:F2}° ({angleRad,6:F3} rad){rangeStatus}");
                    GUI.color = Color.white;
                }
                else
                {
                    GUI.color = Color.gray;
                    GUILayout.Label($"  R_J{i + 1} = 未連接");
                    GUI.color = Color.white;
                }
            }
        }
        else
        {
            GUILayout.Label("  右臂未設定");
        }
        GUILayout.EndVertical();

        GUILayout.EndHorizontal();
        GUILayout.EndArea();
    }

    /// <summary>
    /// 取得編輯器樣式（粗體）
    /// </summary>
    GUIStyle EditorGUIStyle()
    {
        GUIStyle style = new GUIStyle(GUI.skin.label);
        style.fontStyle = FontStyle.Bold;
        return style;
    }

    #endregion

    #region OpenArm Retarget 自動發送

    void FixedUpdate()
    {
        // 自動發送關節狀態
        if (autoSendJointStates && retarget != null && isConnected && ros != null)
        {
            if (Time.time - lastJointStateSendTime >= jointStateSendInterval)
            {
                SendRetargetJointsToROS2("left", retarget.left, leftJointNames);
                SendRetargetJointsToROS2("right", retarget.right, rightJointNames);
                lastJointStateSendTime = Time.time;
            }
        }

        // 自動發送夾爪 L_EE / R_EE
        if (autoSendGripperEE && (leftGripper != null || rightGripper != null) && isConnected && ros != null)
        {
            if (Time.time - lastGripperSendTime >= gripperSendInterval)
            {
                PublishGripperEEJointState();
                lastGripperSendTime = Time.time;
            }
        }
    }

    /// <summary>
    /// 從 OpenArmRetarget 讀取關節角度並發送到 ROS2
    /// </summary>
    void SendRetargetJointsToROS2(string side, OpenArmRetarget.JointMap[] joints, string[] jointNames)
    {
        if (joints == null || joints.Length == 0) return;
        if (jointNames == null || jointNames.Length != joints.Length)
        {
            Debug.LogWarning($"⚠️ {side} 關節名稱數量({jointNames?.Length})與關節數量({joints.Length})不匹配");
            return;
        }

        float[] anglesRad = new float[joints.Length];
        bool hasValidJoints = false;

        // 讀取關節角度並轉換為弧度
        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i]?.joint != null)
            {
                var drive = joints[i].joint.xDrive;
                float angleDeg = drive.target;
                float angleRad = angleDeg * Mathf.Deg2Rad;  // 度 → 弧度

                // 套用上下限檢查
                angleRad = ClampJointAngle(angleRad, i);
                anglesRad[i] = angleRad;
                hasValidJoints = true;
            }
            else
            {
                anglesRad[i] = 0f;
            }
        }

        if (!hasValidJoints) return;

        // 發送到 ROS2
        try
        {
            PublishJointCommands(jointNames, anglesRad);
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ 發送 {side} 關節狀態失敗: {ex.Message}");
        }
    }

    /// <summary>
    /// 限制關節角度在安全範圍內
    /// </summary>
    float ClampJointAngle(float angleRad, int jointIndex)
    {
        if (jointIndex < 0 || jointIndex >= jointMinLimits.Length)
            return angleRad;

        float clamped = Mathf.Clamp(angleRad, jointMinLimits[jointIndex], jointMaxLimits[jointIndex]);

        // 如果超出範圍，記錄警告
        if (Mathf.Abs(clamped - angleRad) > 0.01f)
        {
            Debug.LogWarning($"⚠️ Joint {jointIndex + 1} 角度超出範圍: {angleRad:F3} rad → 限制為 {clamped:F3} rad " +
                           $"(範圍: {jointMinLimits[jointIndex]:F2} ~ {jointMaxLimits[jointIndex]:F2})");
        }

        return clamped;
    }

    /// <summary>
    /// 手動發送當前關節狀態（測試用）
    /// </summary>
    [ContextMenu("發送當前關節狀態")]
    public void SendCurrentJointStates()
    {
        if (retarget == null)
        {
            Debug.LogWarning("⚠️ OpenArmRetarget 未設定");
            return;
        }

        if (!isConnected)
        {
            Debug.LogWarning("⚠️ ROS2 未連接");
            return;
        }

        SendRetargetJointsToROS2("left", retarget.left, leftJointNames);
        SendRetargetJointsToROS2("right", retarget.right, rightJointNames);

        Debug.Log("📤 已發送當前關節狀態到 ROS2");
    }

    /// <summary>
    /// 顯示關節上下限資訊
    /// </summary>
    [ContextMenu("顯示關節上下限")]
    public void ShowJointLimits()
    {
        Debug.Log("=== OpenArm 關節上下限（弧度）===");
        for (int i = 0; i < jointMinLimits.Length; i++)
        {
            Debug.Log($"J{i + 1}: {jointMinLimits[i]:F2} ~ {jointMaxLimits[i]:F2} rad " +
                     $"({jointMinLimits[i] * Mathf.Rad2Deg:F1}° ~ {jointMaxLimits[i] * Mathf.Rad2Deg:F1}°)");
        }
    }

    #endregion

    #region VR IP 配置界面

    /// <summary>
    /// 初始化 IP 配置界面
    /// </summary>
    void InitializeIPConfigUI()
    {
        // 如果提供了 Prefab，使用 Prefab
        if (ipConfigCanvasPrefab != null)
        {
            ipConfigCanvasInstance = Instantiate(ipConfigCanvasPrefab);
            SetupIPConfigUIFromPrefab();
        }
        else
        {
            // 否則動態創建
            CreateIPConfigUI();
        }

        // 初始化臨時值
        tempIPAddress = rosIPAddress;
        tempPort = rosPort;

        // 更新界面顯示
        UpdateIPConfigUI();

        // 在 Play 模式下默認顯示界面
        if (ipConfigCanvasInstance != null)
        {
            ipConfigCanvasInstance.SetActive(true);
            isIPConfigUIVisible = true;

            Debug.Log($"✅ IP 配置界面已創建並顯示");
            Debug.Log($"   位置: {ipConfigCanvasInstance.transform.position}");
            Debug.Log($"   縮放: {ipConfigCanvasInstance.transform.localScale}");
            Debug.Log($"   如果看不到界面，請檢查位置和縮放設置");
        }
        else
        {
            Debug.LogError("❌ IP 配置界面創建失敗！");
        }
    }

    /// <summary>
    /// 從 Prefab 設置 UI（如果提供了 Prefab）
    /// </summary>
    void SetupIPConfigUIFromPrefab()
    {
        // 查找組件
        ipAddressInputField = ipConfigCanvasInstance.GetComponentInChildren<TMP_InputField>();
        if (ipAddressInputField == null)
        {
            TMP_InputField[] inputs = ipConfigCanvasInstance.GetComponentsInChildren<TMP_InputField>();
            if (inputs.Length > 0) ipAddressInputField = inputs[0];
            if (inputs.Length > 1) portInputField = inputs[1];
        }

        Button[] buttons = ipConfigCanvasInstance.GetComponentsInChildren<Button>();
        foreach (Button btn in buttons)
        {
            string btnName = btn.name.ToLower();
            if (btnName.Contains("apply") || btnName.Contains("確認") || btnName.Contains("應用"))
                applyButton = btn;
            else if (btnName.Contains("cancel") || btnName.Contains("取消"))
                cancelButton = btn;
            else if (btnName.Contains("toggle") || btnName.Contains("顯示") || btnName.Contains("隱藏"))
                toggleButton = btn;
        }

        virtualKeyboard = ipConfigCanvasInstance.GetComponentInChildren<VirtualKeyboard>();

        // 綁定按鈕事件
        if (applyButton != null)
            applyButton.onClick.AddListener(OnApplyIPConfig);
        if (cancelButton != null)
            cancelButton.onClick.AddListener(OnCancelIPConfig);
        if (toggleButton != null)
            toggleButton.onClick.AddListener(OnToggleIPConfigUI);
    }

    /// <summary>
    /// 動態創建 IP 配置界面
    /// </summary>
    void CreateIPConfigUI()
    {
        // 創建 Canvas（World Space，適合 VR）
        GameObject canvasObj = new GameObject("IPConfigCanvas");
        Canvas canvas = canvasObj.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;

        // 嘗試找到 XR Camera
        Camera xrCamera = Camera.main;
        if (xrCamera == null)
        {
            xrCamera = FindFirstObjectByType<Camera>();
        }
        canvas.worldCamera = xrCamera;

        // 添加 Canvas Scaler
        CanvasScaler scaler = canvasObj.AddComponent<CanvasScaler>();
        scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1920, 1080);

        // 添加 Graphic Raycaster（用於手柄射線交互）
        canvasObj.AddComponent<GraphicRaycaster>();

        // 確保有 EventSystem（Unity UI 需要）
        if (FindFirstObjectByType<UnityEngine.EventSystems.EventSystem>() == null)
        {
            GameObject eventSystemObj = new GameObject("EventSystem");
            eventSystemObj.AddComponent<UnityEngine.EventSystems.EventSystem>();
            eventSystemObj.AddComponent<UnityEngine.EventSystems.StandaloneInputModule>();
        }

        // 自動配置 XR Ray Interactor（如果存在）
        ConfigureXRRayInteractors();

        // 設置 Canvas 位置和縮放
        canvasObj.transform.position = uiPosition;
        canvasObj.transform.localScale = uiScale;

        // 創建背景面板
        GameObject panel = CreateUIElement("Panel", canvasObj.transform);
        Image panelImage = panel.AddComponent<Image>();
        panelImage.color = new Color(0.1f, 0.1f, 0.1f, 0.9f);
        SetRectTransform(panel, Vector2.zero, Vector2.one, Vector2.zero, Vector2.zero);

        // 創建標題 - 改用 TextAnchor.MiddleCenter
        CreateTextLabel(panel.transform, "Title", "ROS TCP Connection Config",
            new Vector2(0, 200), new Vector2(800, 60), 36, TextAnchor.MiddleCenter);

        // 創建 IP 地址標籤和輸入框 - 改用 TextAnchor.MiddleLeft
        CreateTextLabel(panel.transform, "IPLabel", "IP Address:",
            new Vector2(-250, 120), new Vector2(150, 40), 24, TextAnchor.MiddleLeft);

        GameObject ipInputObj = CreateInputField(panel.transform, "IPInput",
            new Vector2(0, 120), new Vector2(400, 50), rosIPAddress);
        ipAddressInputField = ipInputObj.GetComponent<TMP_InputField>();
        ipAddressInputField.onSelect.AddListener((string value) => ShowVirtualKeyboard(ipAddressInputField));

        // 創建端口標籤和輸入框 - 改用 TextAnchor.MiddleLeft
        CreateTextLabel(panel.transform, "PortLabel", "Port:",
            new Vector2(-250, 40), new Vector2(150, 40), 24, TextAnchor.MiddleLeft);

        GameObject portInputObj = CreateInputField(panel.transform, "PortInput",
            new Vector2(0, 40), new Vector2(200, 50), rosPort.ToString());
        portInputField = portInputObj.GetComponent<TMP_InputField>();
        portInputField.contentType = TMP_InputField.ContentType.IntegerNumber;
        portInputField.onSelect.AddListener((string value) => ShowVirtualKeyboard(portInputField));

        // 創建按鈕
        applyButton = CreateButton(panel.transform, "ApplyButton", "Apply",
            new Vector2(-100, -60), new Vector2(150, 50), OnApplyIPConfig);

        cancelButton = CreateButton(panel.transform, "CancelButton", "Cancel",
            new Vector2(100, -60), new Vector2(150, 50), OnCancelIPConfig);

        // 創建切換按鈕（用於顯示/隱藏界面）
        toggleButton = CreateButton(panel.transform, "ToggleButton", "Show Config",
            new Vector2(0, -140), new Vector2(200, 50), OnToggleIPConfigUI);

        // 添加 VR 交互支持
        AddVRInteractionSupport(ipInputObj);
        AddVRInteractionSupport(portInputObj);
        AddVRInteractionSupport(applyButton.gameObject);
        AddVRInteractionSupport(cancelButton.gameObject);
        AddVRInteractionSupport(toggleButton.gameObject);

        ipConfigCanvasInstance = canvasObj;
    }

    /// <summary>
    /// 創建 UI 元素（通用）
    /// </summary>
    GameObject CreateUIElement(string name, Transform parent)
    {
        GameObject obj = new GameObject(name);
        obj.transform.SetParent(parent, false);
        return obj;
    }

    /// <summary>
    /// 設置 RectTransform
    /// </summary>
    void SetRectTransform(GameObject obj, Vector2 anchorMin, Vector2 anchorMax,
        Vector2 sizeDelta, Vector2 anchoredPosition)
    {
        RectTransform rect = obj.GetComponent<RectTransform>();
        if (rect == null) rect = obj.AddComponent<RectTransform>();
        rect.anchorMin = anchorMin;
        rect.anchorMax = anchorMax;
        rect.sizeDelta = sizeDelta;
        rect.anchoredPosition = anchoredPosition;
    }

    /// <summary>
    /// 創建文字標籤（使用 Unity Text）
    /// </summary>
    GameObject CreateTextLabel(Transform parent, string name, string text,
        Vector2 position, Vector2 size, int fontSize, TextAnchor alignment)
    {
        GameObject labelObj = CreateUIElement(name, parent);

        Text textComp = labelObj.AddComponent<Text>();
        textComp.text = text;
        textComp.fontSize = fontSize;
        textComp.alignment = alignment;
        textComp.color = Color.white;
        textComp.fontStyle = FontStyle.Bold;
        textComp.font = Resources.GetBuiltinResource<Font>("Arial.ttf");

        SetRectTransform(labelObj, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), size, position);

        return labelObj;
    }

    /// <summary>
    /// 創建輸入框（使用 Unity Text）
    /// </summary>
    GameObject CreateInputField(Transform parent, string name,
        Vector2 position, Vector2 size, string placeholderText)
    {
        GameObject inputObj = CreateUIElement(name, parent);

        Image bgImage = inputObj.AddComponent<Image>();
        bgImage.color = new Color(0.2f, 0.2f, 0.2f, 1f);

        InputField inputField = inputObj.AddComponent<InputField>();
        SetRectTransform(inputObj, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), size, position);

        // 創建文字組件
        GameObject textObj = CreateUIElement("Text", inputObj.transform);
        Text textComp = textObj.AddComponent<Text>();
        textComp.text = "";
        textComp.fontSize = 24;
        textComp.color = Color.white;
        textComp.alignment = TextAnchor.MiddleLeft;
        textComp.font = Resources.GetBuiltinResource<Font>("Arial.ttf");
        textComp.supportRichText = false;

        RectTransform textRect = textObj.GetComponent<RectTransform>();
        SetRectTransform(textObj, Vector2.zero, Vector2.one, Vector2.zero, Vector2.zero);
        textRect.offsetMin = new Vector2(10, 5);
        textRect.offsetMax = new Vector2(-10, -5);

        // 創建佔位符
        GameObject placeholderObj = CreateUIElement("Placeholder", inputObj.transform);
        Text placeholderComp = placeholderObj.AddComponent<Text>();
        placeholderComp.text = placeholderText;
        placeholderComp.fontSize = 24;
        placeholderComp.color = new Color(0.5f, 0.5f, 0.5f, 1f);
        placeholderComp.alignment = TextAnchor.MiddleLeft;
        placeholderComp.font = Resources.GetBuiltinResource<Font>("Arial.ttf");
        placeholderComp.fontStyle = FontStyle.Italic;

        RectTransform placeholderRect = placeholderObj.GetComponent<RectTransform>();
        SetRectTransform(placeholderObj, Vector2.zero, Vector2.one, Vector2.zero, Vector2.zero);
        placeholderRect.offsetMin = new Vector2(10, 5);
        placeholderRect.offsetMax = new Vector2(-10, -5);

        // 設置 InputField
        inputField.textComponent = textComp;
        inputField.placeholder = placeholderComp;

        return inputObj;
    }

    /// <summary>
    /// 創建按鈕（使用 Unity Text）
    /// </summary>
    Button CreateButton(Transform parent, string name, string text,
        Vector2 position, Vector2 size, UnityEngine.Events.UnityAction onClick)
    {
        GameObject buttonObj = CreateUIElement(name, parent);

        Image buttonImage = buttonObj.AddComponent<Image>();
        buttonImage.color = new Color(0.2f, 0.5f, 0.8f, 1f);

        Button button = buttonObj.AddComponent<Button>();
        button.onClick.AddListener(onClick);

        // 創建按鈕文字（使用 Unity Text）
        GameObject textObj = CreateUIElement("Text", buttonObj.transform);
        Text textComp = textObj.AddComponent<Text>();
        textComp.text = text;
        textComp.fontSize = 24;
        textComp.color = Color.white;
        textComp.alignment = TextAnchor.MiddleCenter;
        textComp.fontStyle = FontStyle.Bold;
        textComp.font = Resources.GetBuiltinResource<Font>("Arial.ttf");

        SetRectTransform(textObj, Vector2.zero, Vector2.one, Vector2.zero, Vector2.zero);
        SetRectTransform(buttonObj, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), size, position);

        return button;
    }

    /// <summary>
    /// 自動配置 XR Ray Interactor
    /// </summary>
    void ConfigureXRRayInteractors()
    {
#if UNITY_XR_INTERACTION_TOOLKIT
        try
        {
            // 使用反射來查找 XR Ray Interactor（因為類型名稱可能因版本而異）
            System.Type rayInteractorType = System.Type.GetType("UnityEngine.XR.Interaction.Toolkit.XRRayInteractor, Unity.XR.Interaction.Toolkit");
            if (rayInteractorType == null)
            {
                rayInteractorType = System.Type.GetType("UnityEngine.XR.Interaction.Toolkit.Interactables.XRRayInteractor, Unity.XR.Interaction.Toolkit");
            }
            
            if (rayInteractorType != null)
            {
                // 查找所有 XR Ray Interactor
                UnityEngine.Object[] rayInteractors = FindObjectsByType(rayInteractorType, FindObjectsSortMode.None);
                
                if (rayInteractors.Length > 0)
                {
                    // 查找或創建 XR Interaction Manager
                    System.Type managerType = System.Type.GetType("UnityEngine.XR.Interaction.Toolkit.XRInteractionManager, Unity.XR.Interaction.Toolkit");
                    if (managerType == null)
                    {
                        managerType = System.Type.GetType("UnityEngine.XR.Interaction.Toolkit.InteractionManager, Unity.XR.Interaction.Toolkit");
                    }
                    
                    UnityEngine.Component interactionManager = null;
                    if (managerType != null)
                    {
                        interactionManager = FindFirstObjectByType(managerType) as UnityEngine.Component;
                        if (interactionManager == null)
                        {
                            GameObject managerObj = new GameObject("XR Interaction Manager");
                            interactionManager = managerObj.AddComponent(managerType) as UnityEngine.Component;
                        }
                    }
                    
                    // 配置每個 Ray Interactor
                    foreach (UnityEngine.Object obj in rayInteractors)
                    {
                        if (obj == null) continue;
                        
                        // 使用反射設置屬性
                        var interactionManagerProp = rayInteractorType.GetProperty("interactionManager");
                        if (interactionManagerProp != null && interactionManager != null)
                        {
                            var currentManager = interactionManagerProp.GetValue(obj);
                            if (currentManager == null)
                            {
                                interactionManagerProp.SetValue(obj, interactionManager);
                            }
                        }
                        
                        // 設置 Ray Origin Transform
                        var rayOriginProp = rayInteractorType.GetProperty("rayOriginTransform");
                        if (rayOriginProp != null)
                        {
                            var currentOrigin = rayOriginProp.GetValue(obj) as Transform;
                            if (currentOrigin == null)
                            {
                                rayOriginProp.SetValue(obj, (obj as MonoBehaviour).transform);
                            }
                        }
                        
                        // 啟用 UI Interaction
                        var uiInteractionProp = rayInteractorType.GetProperty("enableUIInteraction");
                        if (uiInteractionProp != null)
                        {
                            uiInteractionProp.SetValue(obj, true);
                        }
                        
                        Debug.Log($"✅ 已配置 XR Ray Interactor: {(obj as MonoBehaviour).name}");
                    }
                }
            }
        }
        catch (System.Exception ex)
        {
            Debug.LogWarning($"⚠️ 配置 XR Ray Interactor 時發生錯誤: {ex.Message}");
            Debug.LogWarning($"   這可能是因為 XR Interaction Toolkit 版本不同或未安裝");
        }
#else
        Debug.Log("ℹ️ XR Interaction Toolkit 未安裝或未啟用，跳過自動配置");
#endif
    }

    /// <summary>
    /// 添加 VR 交互支持（使用 XR Interaction Toolkit 或 Unity UI 事件）
    /// </summary>
    void AddVRInteractionSupport(GameObject uiElement)
    {
        // 方法1: 嘗試添加 XR Simple Interactable（如果使用 XR Interaction Toolkit）
#if UNITY_XR_INTERACTION_TOOLKIT
        try
        {
            var interactable = uiElement.GetComponent<UnityEngine.XR.Interaction.Toolkit.Interactables.XRSimpleInteractable>();
            if (interactable == null)
            {
                interactable = uiElement.AddComponent<UnityEngine.XR.Interaction.Toolkit.Interactables.XRSimpleInteractable>();
            }
        }
        catch (System.Exception)
        {
            // XR Interaction Toolkit 不可用，使用其他方法
        }
#endif

        // 方法2: 添加 EventTrigger 支持手柄射線點擊
        EventTrigger trigger = uiElement.GetComponent<EventTrigger>();
        if (trigger == null)
        {
            trigger = uiElement.AddComponent<EventTrigger>();
        }

        // 對於按鈕，添加點擊事件
        Button btn = uiElement.GetComponent<Button>();
        if (btn != null)
        {
            // 添加 Pointer Click 事件
            EventTrigger.Entry clickEntry = new EventTrigger.Entry();
            clickEntry.eventID = EventTriggerType.PointerClick;
            clickEntry.callback.AddListener((eventData) =>
            {
                btn.onClick.Invoke();
            });
            trigger.triggers.Add(clickEntry);
        }

        // 對於輸入框，添加選擇事件
        TMP_InputField inputField = uiElement.GetComponent<TMP_InputField>();
        if (inputField != null)
        {
            // 添加 Pointer Click 事件來選擇輸入框
            EventTrigger.Entry clickEntry = new EventTrigger.Entry();
            clickEntry.eventID = EventTriggerType.PointerClick;
            clickEntry.callback.AddListener((eventData) =>
            {
                inputField.Select();
                inputField.ActivateInputField();
                ShowVirtualKeyboard(inputField);
            });
            trigger.triggers.Add(clickEntry);
        }
    }

    /// <summary>
    /// 顯示虛擬鍵盤
    /// </summary>
    void ShowVirtualKeyboard(TMP_InputField targetField)
    {
        // 如果提供了虛擬鍵盤 Prefab，實例化它
        if (virtualKeyboardPrefab != null)
        {
            if (virtualKeyboard == null || !virtualKeyboard.gameObject.activeSelf)
            {
                GameObject keyboardObj = Instantiate(virtualKeyboardPrefab, ipConfigCanvasInstance.transform);
                virtualKeyboard = keyboardObj.GetComponent<VirtualKeyboard>();
                if (virtualKeyboard == null)
                {
                    virtualKeyboard = keyboardObj.AddComponent<VirtualKeyboard>();
                }
                keyboardObj.transform.localPosition = new Vector3(0, -300, 0);

                // 修復虛擬鍵盤的字体問題
                FixVirtualKeyboardFonts(keyboardObj);
            }

            if (virtualKeyboard != null)
            {
                virtualKeyboard.Show(targetField);
            }
        }
        else
        {
            // 如果沒有 Prefab，動態創建簡單的虛擬鍵盤
            CreateSimpleVirtualKeyboard(targetField);
        }
    }

    /// <summary>
    /// 修復虛擬鍵盤的字体問題（將 TextMeshPro 轉換為 Unity Text）
    /// </summary>
    void FixVirtualKeyboardFonts(GameObject keyboardObj)
    {
        // 獲取 Unity 默認字体
        Font defaultFont = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        if (defaultFont == null)
        {
            defaultFont = Resources.GetBuiltinResource<Font>("Arial.ttf");
        }

        // 查找所有 TextMeshPro 組件並轉換為 Unity Text
        TextMeshProUGUI[] tmpComponents = keyboardObj.GetComponentsInChildren<TextMeshProUGUI>(true);
        foreach (var tmpComp in tmpComponents)
        {
            if (tmpComp == null) continue;

            // 保存文字內容和設置
            string text = tmpComp.text;
            int fontSize = (int)tmpComp.fontSize;
            Color textColor = tmpComp.color;
            TextAlignmentOptions alignment = tmpComp.alignment;

            // 獲取父對象
            GameObject parentObj = tmpComp.gameObject;
            Transform parentTransform = parentObj.transform.parent;

            // 創建新的 Unity Text 對象
            GameObject textObj = new GameObject("Text");
            textObj.transform.SetParent(parentTransform, false);

            // 複製 RectTransform 設置
            RectTransform tmpRect = tmpComp.GetComponent<RectTransform>();
            RectTransform newRect = textObj.AddComponent<RectTransform>();
            if (tmpRect != null)
            {
                newRect.anchorMin = tmpRect.anchorMin;
                newRect.anchorMax = tmpRect.anchorMax;
                newRect.sizeDelta = tmpRect.sizeDelta;
                newRect.anchoredPosition = tmpRect.anchoredPosition;
                newRect.offsetMin = tmpRect.offsetMin;
                newRect.offsetMax = tmpRect.offsetMax;
            }

            // 添加 Unity Text 組件
            Text unityText = textObj.AddComponent<Text>();
            unityText.text = text;
            unityText.fontSize = fontSize;
            unityText.color = textColor;

            // 轉換對齊方式
            switch (alignment)
            {
                case TextAlignmentOptions.Center:
                case TextAlignmentOptions.Midline:
                    unityText.alignment = TextAnchor.MiddleCenter;
                    break;
                case TextAlignmentOptions.Left:
                case TextAlignmentOptions.MidlineLeft:
                    unityText.alignment = TextAnchor.MiddleLeft;
                    break;
                case TextAlignmentOptions.Right:
                case TextAlignmentOptions.MidlineRight:
                    unityText.alignment = TextAnchor.MiddleRight;
                    break;
                default:
                    unityText.alignment = TextAnchor.MiddleCenter;
                    break;
            }

            // 設置字体
            if (defaultFont != null)
            {
                unityText.font = defaultFont;
            }

            // 刪除舊的 TextMeshPro 組件
            DestroyImmediate(tmpComp);

            Debug.Log($"✅ 已將 {parentObj.name} 的 TextMeshPro 轉換為 Unity Text");
        }
    }

    /// <summary>
    /// 創建簡單的虛擬鍵盤（如果沒有 Prefab）
    /// </summary>
    void CreateSimpleVirtualKeyboard(TMP_InputField targetField)
    {
        if (ipConfigCanvasInstance == null) return;

        // 創建鍵盤容器
        GameObject keyboardPanel = CreateUIElement("VirtualKeyboard", ipConfigCanvasInstance.transform);
        Image panelImage = keyboardPanel.AddComponent<Image>();
        panelImage.color = new Color(0.15f, 0.15f, 0.15f, 0.95f);
        SetRectTransform(keyboardPanel, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f),
            new Vector2(600, 400), new Vector2(0, -300));

        // 創建標題 - 改用 TextAnchor.MiddleCenter
        CreateTextLabel(keyboardPanel.transform, "Title", "Virtual Keyboard",
            new Vector2(0, 160), new Vector2(500, 40), 28, TextAnchor.MiddleCenter);

        // 創建數字按鈕網格 (0-9 和 .)
        float buttonSize = 80f;
        float spacing = 10f;
        float startX = -120f;
        float startY = 80f;

        // 第一行: 1, 2, 3
        for (int i = 1; i <= 3; i++)
        {
            CreateKeyboardButton(keyboardPanel.transform, $"Key{i}", i.ToString(),
                new Vector2(startX + (i - 1) * (buttonSize + spacing), startY),
                new Vector2(buttonSize, buttonSize));
        }

        // 第二行: 4, 5, 6
        for (int i = 4; i <= 6; i++)
        {
            CreateKeyboardButton(keyboardPanel.transform, $"Key{i}", i.ToString(),
                new Vector2(startX + (i - 4) * (buttonSize + spacing), startY - (buttonSize + spacing)),
                new Vector2(buttonSize, buttonSize));
        }

        // 第三行: 7, 8, 9
        for (int i = 7; i <= 9; i++)
        {
            CreateKeyboardButton(keyboardPanel.transform, $"Key{i}", i.ToString(),
                new Vector2(startX + (i - 7) * (buttonSize + spacing), startY - 2 * (buttonSize + spacing)),
                new Vector2(buttonSize, buttonSize));
        }

        // 第四行: 0, .
        CreateKeyboardButton(keyboardPanel.transform, "Key0", "0",
            new Vector2(startX, startY - 3 * (buttonSize + spacing)),
            new Vector2(buttonSize, buttonSize));
        CreateKeyboardButton(keyboardPanel.transform, "KeyDot", ".",
            new Vector2(startX + (buttonSize + spacing), startY - 3 * (buttonSize + spacing)),
            new Vector2(buttonSize, buttonSize));

        // 功能按鈕
        CreateKeyboardButton(keyboardPanel.transform, "Backspace", "Del",
            new Vector2(startX + 2 * (buttonSize + spacing), startY - 3 * (buttonSize + spacing)),
            new Vector2(buttonSize, buttonSize));
        CreateKeyboardButton(keyboardPanel.transform, "Clear", "Clear",
            new Vector2(startX + 100, startY - 4 * (buttonSize + spacing)),
            new Vector2(buttonSize * 1.5f, buttonSize));
        CreateKeyboardButton(keyboardPanel.transform, "Confirm", "OK",
            new Vector2(startX + 100 + (buttonSize * 1.5f + spacing), startY - 4 * (buttonSize + spacing)),
            new Vector2(buttonSize * 1.5f, buttonSize));

        // 添加 VirtualKeyboard 組件
        VirtualKeyboard keyboard = keyboardPanel.AddComponent<VirtualKeyboard>();
        keyboard.SetTargetInputField(targetField);
        virtualKeyboard = keyboard;

        // 重新綁定所有按鈕
        Button[] buttons = keyboardPanel.GetComponentsInChildren<Button>();
        foreach (var btn in buttons)
        {
            // 移除舊的監聽器
            btn.onClick.RemoveAllListeners();

            // 根據按鈕名稱重新綁定
            string btnName = btn.name;
            if (btnName.Contains("Key") && btnName != "KeyDot")
            {
                string numStr = btnName.Replace("Key", "");
                if (int.TryParse(numStr, out int num))
                {
                    btn.onClick.AddListener(() => keyboard.AddCharacter(num.ToString()));
                }
            }
            else if (btnName == "KeyDot")
            {
                btn.onClick.AddListener(() => keyboard.AddCharacter("."));
            }
            else if (btnName == "Backspace")
            {
                btn.onClick.AddListener(() => keyboard.Backspace());
            }
            else if (btnName == "Clear")
            {
                btn.onClick.AddListener(() => keyboard.Clear());
            }
            else if (btnName == "Confirm")
            {
                btn.onClick.AddListener(() => keyboard.Confirm());
            }

            // 添加 VR 交互支持
            AddVRInteractionSupport(btn.gameObject);
        }
    }

    /// <summary>
    /// 創建鍵盤按鈕（使用 Unity Text 避免字体問題）
    /// </summary>
    Button CreateKeyboardButton(Transform parent, string name, string text,
        Vector2 position, Vector2 size)
    {
        GameObject buttonObj = CreateUIElement(name, parent);

        Image buttonImage = buttonObj.AddComponent<Image>();
        buttonImage.color = new Color(0.3f, 0.3f, 0.3f, 1f);

        Button button = buttonObj.AddComponent<Button>();

        // 創建按鈕文字（使用 Unity Text 而不是 TextMeshPro，避免字体問題）
        GameObject textObj = CreateUIElement("Text", buttonObj.transform);
        Text textComp = textObj.AddComponent<Text>();
        textComp.text = text;
        textComp.fontSize = 32;
        textComp.color = Color.white;
        textComp.alignment = TextAnchor.MiddleCenter;

        // 使用 Unity 默認字体（Arial）
        Font defaultFont = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        if (defaultFont == null)
        {
            defaultFont = Resources.GetBuiltinResource<Font>("Arial.ttf");
        }
        if (defaultFont != null)
        {
            textComp.font = defaultFont;
        }

        SetRectTransform(textObj, Vector2.zero, Vector2.one, Vector2.zero, Vector2.zero);
        SetRectTransform(buttonObj, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), size, position);

        // 注意：按鈕綁定會在 CreateSimpleVirtualKeyboard 中統一處理
        // 這裡只創建按鈕，不綁定事件

        return button;
    }

    /// <summary>
    /// 更新 IP 配置界面顯示
    /// </summary>
    void UpdateIPConfigUI()
    {
        if (ipAddressInputField != null)
        {
            ipAddressInputField.text = tempIPAddress;
        }

        if (portInputField != null)
        {
            portInputField.text = tempPort.ToString();
        }
    }

    /// <summary>
    /// 應用 IP 配置
    /// </summary>
    void OnApplyIPConfig()
    {
        // 讀取輸入值
        if (ipAddressInputField != null)
        {
            tempIPAddress = ipAddressInputField.text;
        }

        if (portInputField != null)
        {
            if (int.TryParse(portInputField.text, out int port))
            {
                tempPort = port;
            }
        }

        // 驗證 IP 地址格式
        if (IsValidIPAddress(tempIPAddress))
        {
            rosIPAddress = tempIPAddress;
            rosPort = tempPort;

            Debug.Log($"✅ IP 配置已更新: {rosIPAddress}:{rosPort}");

            // 重新初始化連接
            if (connectionInitialized)
            {
                Debug.Log("🔄 重新初始化 ROS 連接...");
                connectionInitialized = false;
                InitializeROSConnection();
            }

            // 隱藏界面
            OnToggleIPConfigUI();
        }
        else
        {
            Debug.LogError($"❌ 無效的 IP 地址格式: {tempIPAddress}");
        }
    }

    /// <summary>
    /// 取消 IP 配置
    /// </summary>
    void OnCancelIPConfig()
    {
        // 恢復原始值
        tempIPAddress = rosIPAddress;
        tempPort = rosPort;
        UpdateIPConfigUI();

        // 隱藏界面
        OnToggleIPConfigUI();
    }

    /// <summary>
    /// 切換 IP 配置界面顯示
    /// </summary>
    void OnToggleIPConfigUI()
    {
        if (ipConfigCanvasInstance != null)
        {
            isIPConfigUIVisible = !isIPConfigUIVisible;
            ipConfigCanvasInstance.SetActive(isIPConfigUIVisible);

            if (toggleButton != null)
            {
                TextMeshProUGUI toggleText = toggleButton.GetComponentInChildren<TextMeshProUGUI>();
                if (toggleText != null)
                {
                    toggleText.text = isIPConfigUIVisible ? "Hide Config" : "Show Config";
                }
            }
        }
    }

    /// <summary>
    /// 驗證 IP 地址格式
    /// </summary>
    bool IsValidIPAddress(string ip)
    {
        if (string.IsNullOrEmpty(ip))
            return false;

        string[] parts = ip.Split('.');
        if (parts.Length != 4)
            return false;

        foreach (string part in parts)
        {
            if (!int.TryParse(part, out int num) || num < 0 || num > 255)
                return false;
        }

        return true;
    }

    #endregion

    void OnDestroy()
    {
        isHeartbeatActive = false;
        StopAllCoroutines();

        // 清理 IP 配置界面
        if (ipConfigCanvasInstance != null)
        {
            Destroy(ipConfigCanvasInstance);
        }

        Debug.Log("🔄 ROSTCPManager 已停止");
    }
}