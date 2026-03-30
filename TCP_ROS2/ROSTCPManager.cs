using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using System.Collections;
using System.Collections.Generic;      // 用於 XR Input 裝置列表
using UnityEngine.UI;
using TMPro;
using UnityEngine.EventSystems;
using UnityEngine.XR;                 // 用於讀取手把側鍵狀態 (備援)
using UnityEngine.InputSystem;        // 用於可在 Inspector 選擇的按鍵 / 手把按鈕

/// <summary>
/// 統一的 ROS TCP 連接管理器 - 修正版
/// </summary>
public class ROSTCPManager : MonoBehaviour
{
    [Header("連接設定（僅供顯示）")]
    [Tooltip("顯示用 - 實際 IP 由場景中的 ROSConnection 配置")]
    [SerializeField] private string rosIPAddress = "192.168.0.15";
    [Tooltip("顯示用 - 實際 Port 由場景中的 ROSConnection 配置")]
    [SerializeField] private int rosPort = 10000;
    public float heartbeatInterval = 1.0f;
    public float connectionTimeout = 5.0f;
    
    [Header("ROSConnection Override")]
    [Tooltip("可選：手動指定場景中的 ROSConnection")]
    [SerializeField] private ROSConnection rosOverride;

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
    public float jointStateSendInterval = 0.02f;     // 發送間隔（秒）- 50Hz
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
    public float gripperSendInterval = 0.1f;        // 發送間隔（秒）
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

    // VR IP 配置界面已移至 ROSIPConfigUI.cs

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

    // XR 手把裝置快取（用來檢查側鍵狀態）
    private static readonly List<UnityEngine.XR.InputDevice> xrControllers = new List<UnityEngine.XR.InputDevice>();

    // IP 配置界面相關變數已移至 ROSIPConfigUI.cs

    // OpenArm 關節上下限（弧度）- 區分左右手 (ROS2 openarm_bimanual_control.urdf)
    private readonly float[] leftJointMinLimits = new float[7] {
        -3.490659f,   // J1
        -3.316125f,   // J2
        -1.570796f,   // J3
        0f,           // J4
        -1.570796f,   // J5
        -0.785398f,   // J6
        -1.570796f    // J7
    };
    private readonly float[] leftJointMaxLimits = new float[7] {
        1.396263f,    // J1
        0.174533f,    // J2
        1.570796f,    // J3
        2.443461f,    // J4
        1.570796f,    // J5
        0.785398f,    // J6
        1.570796f     // J7
    };

    private readonly float[] rightJointMinLimits = new float[7] {
        -1.396263f,   // J1
        -0.174533f,   // J2
        -1.570796f,   // J3
        0f,           // J4
        -1.570796f,   // J5
        -0.785398f,   // J6
        -1.570796f    // J7
    };
    private readonly float[] rightJointMaxLimits = new float[7] {
        3.490659f,    // J1
        3.316125f,    // J2
        1.570796f,    // J3
        2.443461f,    // J4
        1.570796f,    // J5
        0.785398f,    // J6
        1.570796f     // J7
    };

    [Header("VR 手把側鍵控制")]
    [Tooltip("開啟後，只有按住對應手的側鍵時，才會發送該側手臂 / 夾爪訊息到 ROS2")]
    public bool requireSideButtonToSend = true;
    [Tooltip("左手側鍵 Input Action（例如 Left Grip / Trigger / 任意按鍵）")]
    public InputActionReference leftSideButtonAction;
    [Tooltip("右手側鍵 Input Action（例如 Right Grip / Trigger / 任意按鍵）")]
    public InputActionReference rightSideButtonAction;
    [Tooltip("任一手按住側鍵即可同時發送雙臂（ehand 模式下無法同時按兩手側鍵時使用）")]
    public bool linkedSideButton = true;

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

    void OnEnable()
    {
        // 啟用左右側鍵 Input Action（可在 Inspector 選擇）
        if (leftSideButtonAction != null && leftSideButtonAction.action != null)
        {
            leftSideButtonAction.action.Enable();
        }
        if (rightSideButtonAction != null && rightSideButtonAction.action != null)
        {
            rightSideButtonAction.action.Enable();
        }
    }

    void OnDisable()
    {
        // 停用左右側鍵 Input Action
        if (leftSideButtonAction != null && leftSideButtonAction.action != null)
        {
            leftSideButtonAction.action.Disable();
        }
        if (rightSideButtonAction != null && rightSideButtonAction.action != null)
        {
            rightSideButtonAction.action.Disable();
        }
    }

    void Start()
    {
        Debug.Log("🚀 ROSTCPManager 啟動...");

        // IP 配置界面已移至 ROSIPConfigUI.cs
        
        StartCoroutine(DelayedInitialization());
    }

    IEnumerator DelayedInitialization()
    {
        // 等待 3 秒讓 ROSConnection 連線穩定
        Debug.Log("[ROSTCPManager] 等待 3.0 秒讓 ROSConnection 連線穩定...");
        yield return new WaitForSecondsRealtime(3.0f);

        InitializeROSConnection();
    }

    void InitializeROSConnection()
    {
        try
        {
            Debug.Log($"🔧 開始初始化 ROS 連接（顯示用 IP: {rosIPAddress}:{rosPort}）");
            Debug.Log($"[ROSTCPManager] ROSConnection count = {RosConn.CountROS()}");

            // 使用 rosOverride 或從場景中找到 ROSConnection
            ros = rosOverride != null ? rosOverride : RosConn.GetSceneROS();

            if (ros == null)
            {
                Debug.LogError("❌ Scene 裡找不到 ROSConnection（請確認 Hierarchy 只有一顆，且已啟用）");
                isConnected = false;
                return;
            }

            Debug.Log($"✅ Using ROSConnection: {ros.gameObject.name} id={ros.GetInstanceID()}");

            // 等待連接建立後再註冊訂閱者
            StartCoroutine(WaitForConnectionAndRegister());
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ ROSTCPManager 初始化失敗: {ex.Message}");
            Debug.LogError($"Stack trace: {ex.StackTrace}");

            isConnected = false;

            // 延遲重試
            Invoke(nameof(InitializeROSConnection), 5.0f);
        }
    }
    
    /// <summary>
    /// 等待連接建立後再註冊訂閱者和發布者
    /// </summary>
    IEnumerator WaitForConnectionAndRegister()
    {
        // 已經在 DelayedInitialization 等待了 3 秒
        // 這裡只做額外的穩定等待
        yield return new WaitForSeconds(0.5f);
        
        Debug.Log("🔄 開始註冊訂閱者和發布者...");

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
        Debug.Log("🎉 ROSTCPManager 初始化完成");

        // 啟用自動發送關節狀態
        if (autoSendJointStates && retarget != null)
        {
            Debug.Log("✅ 啟用 OpenArmRetarget 自動發送");
        }
    }


    /// <summary>
    /// 等待並檢查連接狀態
    /// </summary>
    IEnumerator WaitAndCheckConnection()
    {
        yield return new WaitForSeconds(1.0f);

        if (ros != null)
        {
            if (ros.HasConnectionThread)
            {
                Debug.Log("🎉 ROS 連接線程已成功啟動！");
                isConnected = true;
            }
            else
            {
                Debug.LogError("❌ ROS 連接線程啟動失敗！");
                Debug.LogError("   請檢查:");
                Debug.LogError($"   1. ROS2 端是否運行: ros2 run ros_tcp_endpoint default_server_endpoint");
                Debug.LogError($"   2. IP 地址是否正確: {rosIPAddress}");
                Debug.LogError($"   3. 端口是否正確: {rosPort}");
                Debug.LogError("   4. 防火牆是否阻擋連接");
                Debug.LogError("   5. 在 Edit → Project Settings → Robotics → ROS Settings 中手動設定連接參數");
                isConnected = false;
            }
        }
    }

    /// <summary>
    /// 從 ROSConnection 讀取實際的 IP/Port（用於 UI 顯示）
    /// </summary>
    void UpdateDisplayIPFromROSConnection()
    {
        if (ros == null) return;
        
        try
        {
            var rosType = ros.GetType();
            
            // 讀取 IP
            var ipField = rosType.GetField("m_RosIPAddress",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            if (ipField != null)
            {
                rosIPAddress = ipField.GetValue(ros) as string ?? rosIPAddress;
            }
            
            // 讀取 Port
            var portField = rosType.GetField("m_RosPort",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            if (portField != null)
            {
                rosPort = (int)portField.GetValue(ros);
            }
            
            Debug.Log($"📡 ROS 連接目標: {rosIPAddress}:{rosPort}（來自 Project Settings）");
        }
        catch (System.Exception ex)
        {
            Debug.LogWarning($"無法讀取 ROSConnection 連接信息: {ex.Message}");
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
        float right = GetJawTargetMeters(rightGripper, rightGripper != null ? rightGripper.rightJaw : null);

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
        string[] testJoints = { "L_J1", "L_J2", "L_J3", "L_J4", "L_J5", "L_J6" };
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

                    // 檢查是否超出範圍 (使用左手限制)
                    bool outOfRange = false;
                    string rangeStatus = "";
                    if (i < leftJointMinLimits.Length)
                    {
                        if (angleRad < leftJointMinLimits[i] - 1e-3f)
                        {
                            outOfRange = true;
                            rangeStatus = " [低於下限]";
                        }
                        else if (angleRad > leftJointMaxLimits[i] + 1e-3f)
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

                    // 檢查是否超出範圍 (使用右手限制)
                    bool outOfRange = false;
                    string rangeStatus = "";
                    if (i < rightJointMinLimits.Length)
                    {
                        if (angleRad < rightJointMinLimits[i] - 1e-3f)
                        {
                            outOfRange = true;
                            rangeStatus = " [低於下限]";
                        }
                        else if (angleRad > rightJointMaxLimits[i] + 1e-3f)
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
        bool leftPressed = IsLeftSideButtonPressed();
        bool rightPressed = IsRightSideButtonPressed();
        bool eitherPressed = leftPressed || rightPressed;

        bool canSendLeft = !requireSideButtonToSend || (linkedSideButton ? eitherPressed : leftPressed);
        bool canSendRight = !requireSideButtonToSend || (linkedSideButton ? eitherPressed : rightPressed);

        // 自動發送關節狀態
        if (autoSendJointStates && retarget != null && isConnected && ros != null)
        {
            // 左右手分別根據各自側鍵決定是否發送
            if (Time.time - lastJointStateSendTime >= jointStateSendInterval)
            {
                if (canSendLeft)
                    SendRetargetJointsToROS2("left", retarget.left, leftJointNames, true);
                if (canSendRight)
                    SendRetargetJointsToROS2("right", retarget.right, rightJointNames, false);

                if (canSendLeft || canSendRight)
                    lastJointStateSendTime = Time.time;
            }
        }

        // 自動發送夾爪 L_EE / R_EE
        if (autoSendGripperEE && (leftGripper != null || rightGripper != null) && isConnected && ros != null)
        {
            // 夾爪：只要任一側鍵按住即可發送
            bool canSendGripper = !requireSideButtonToSend || canSendLeft || canSendRight;
            if (canSendGripper && Time.time - lastGripperSendTime >= gripperSendInterval)
            {
                PublishGripperEEJointState();
                lastGripperSendTime = Time.time;
            }
        }
    }

    /// <summary>
    /// 檢查左手側鍵是否被按下（優先使用 Input Action，可在 Inspector 選擇；若未設定則退回 XR Left Grip 偵測）
    /// </summary>
    public bool IsLeftSideButtonPressed()
    {
        // 1) 優先使用可在 Inspector 指定的 Input Action
        if (leftSideButtonAction != null && leftSideButtonAction.action != null)
        {
            try
            {
                if (leftSideButtonAction.action.IsPressed())
                    return true;
            }
            catch (System.Exception ex)
            {
                Debug.LogWarning($"⚠️ 讀取 leftSideButtonAction 狀態失敗，改用 XR Left Grip 偵測: {ex.Message}");
            }
        }

        // 2) 若沒有設定 Input Action，或上述失敗，則使用 XR Left Grip Button 作為備援
        xrControllers.Clear();

        // 取得所有左手「手持控制器」類型的 XR 裝置
        InputDevices.GetDevicesWithCharacteristics(
            InputDeviceCharacteristics.HeldInHand | InputDeviceCharacteristics.Controller | InputDeviceCharacteristics.Left,
            xrControllers
        );

        foreach (UnityEngine.XR.InputDevice device in xrControllers)
        {
            if (device.TryGetFeatureValue(UnityEngine.XR.CommonUsages.gripButton, out bool pressed) && pressed)
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// 檢查右手側鍵是否被按下（優先使用 Input Action，可在 Inspector 選擇；若未設定則退回 XR Right Grip 偵測）
    /// </summary>
    public bool IsRightSideButtonPressed()
    {
        // 1) 優先使用可在 Inspector 指定的 Input Action
        if (rightSideButtonAction != null && rightSideButtonAction.action != null)
        {
            try
            {
                if (rightSideButtonAction.action.IsPressed())
                    return true;
            }
            catch (System.Exception ex)
            {
                Debug.LogWarning($"⚠️ 讀取 rightSideButtonAction 狀態失敗，改用 XR Right Grip 偵測: {ex.Message}");
            }
        }

        // 2) 若沒有設定 Input Action，或上述失敗，則使用 XR Right Grip Button 作為備援
        xrControllers.Clear();

        // 取得所有右手「手持控制器」類型的 XR 裝置
        InputDevices.GetDevicesWithCharacteristics(
            InputDeviceCharacteristics.HeldInHand | InputDeviceCharacteristics.Controller | InputDeviceCharacteristics.Right,
            xrControllers
        );

        foreach (UnityEngine.XR.InputDevice device in xrControllers)
        {
            if (device.TryGetFeatureValue(UnityEngine.XR.CommonUsages.gripButton, out bool pressed) && pressed)
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// 從 OpenArmRetarget 讀取關節角度並發送到 ROS2
    /// </summary>
    void SendRetargetJointsToROS2(string side, OpenArmRetarget.JointMap[] joints, string[] jointNames, bool isLeft)
    {
        if (joints == null || joints.Length == 0) return;
        if (jointNames == null || jointNames.Length != joints.Length)
        {
            Debug.LogWarning($"⚠️ {side} 關節名稱數量({jointNames?.Length})與關節數量({joints.Length})不匹配");
            return;
        }

        // 🔧 只收集「真的有綁定 joint」的關節，避免把未綁定的關節硬拉到 0 rad
        List<string> validNames = new List<string>();
        List<float> validAngles = new List<float>();

        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i]?.joint == null)
            {
                // 沒綁定實體關節，跳過這個 slot
                continue;
            }

            var drive = joints[i].joint.xDrive;
            float angleDeg = drive.target;
            float angleRad = angleDeg * Mathf.Deg2Rad;  // 度 → 弧度

            // 套用上下限檢查
            angleRad = ClampJointAngle(angleRad, i, isLeft);

            validNames.Add(jointNames[i]);
            validAngles.Add(angleRad);
        }

        if (validNames.Count == 0)
        {
            Debug.LogWarning($"⚠️ {side} 沒有任何有效關節可發送");
            return;
        }

        // 發送到 ROS2
        try
        {
            PublishJointCommands(validNames.ToArray(), validAngles.ToArray());
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ 發送 {side} 關節狀態失敗: {ex.Message}");
        }
    }

    /// <summary>
    /// 限制關節角度在安全範圍內 (區分左右手)
    /// </summary>
    float ClampJointAngle(float angleRad, int jointIndex, bool isLeft)
    {
        float[] minLimits = isLeft ? leftJointMinLimits : rightJointMinLimits;
        float[] maxLimits = isLeft ? leftJointMaxLimits : rightJointMaxLimits;

        if (jointIndex < 0 || jointIndex >= minLimits.Length)
            return angleRad;

        float min = minLimits[jointIndex];
        float max = maxLimits[jointIndex];

        float clamped = Mathf.Clamp(angleRad, min, max);

        // 如果超出範圍，記錄警告 (僅當差異顯著時)
        if (Mathf.Abs(clamped - angleRad) > 0.01f)
        {
            Debug.LogWarning($"⚠️ {(isLeft ? "Left" : "Right")} Joint {jointIndex + 1} 角度超出範圍: {angleRad:F3} rad → 限制為 {clamped:F3} rad " +
                           $"(範圍: {min:F2} ~ {max:F2})");
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

        SendRetargetJointsToROS2("left", retarget.left, leftJointNames, true);
        SendRetargetJointsToROS2("right", retarget.right, rightJointNames, false);

        Debug.Log("📤 已發送當前關節狀態到 ROS2");
    }

    /// <summary>
    /// 顯示關節上下限資訊
    /// </summary>
    [ContextMenu("顯示關節上下限")]
    public void ShowJointLimits()
    {
        Debug.Log("=== OpenArm 關節上下限（弧度）===");
        Debug.Log("--- 左手 (Left) ---");
        for (int i = 0; i < leftJointMinLimits.Length; i++)
        {
            Debug.Log($"L_J{i + 1}: {leftJointMinLimits[i]:F3} ~ {leftJointMaxLimits[i]:F3} rad " +
                     $"({leftJointMinLimits[i] * Mathf.Rad2Deg:F1}° ~ {leftJointMaxLimits[i] * Mathf.Rad2Deg:F1}°)");
        }
        Debug.Log("--- 右手 (Right) ---");
        for (int i = 0; i < rightJointMinLimits.Length; i++)
        {
            Debug.Log($"R_J{i + 1}: {rightJointMinLimits[i]:F3} ~ {rightJointMaxLimits[i]:F3} rad " +
                     $"({rightJointMinLimits[i] * Mathf.Rad2Deg:F1}° ~ {rightJointMaxLimits[i] * Mathf.Rad2Deg:F1}°)");
        }
    }

    #endregion

    void OnDestroy()
    {
        isHeartbeatActive = false;
        StopAllCoroutines();

        Debug.Log("🔄 ROSTCPManager 已停止");
    }
}