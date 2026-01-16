using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using System.Collections;

/// <summary>
/// 從 VR Humanoid Animator 讀取手指骨骼角度，並發送到 ROS2
/// 
/// 功能：
/// - 讀取 RealisticCharacter 的手指骨骼
/// - 將手指彎曲角度轉換為 0~1 握合值
/// - 發送 JointState 到 /unity/ehand_commands
/// 
/// 對應 eHand-6 馬達：
///   F1 = M1 (拇指旋轉)
///   F2 = M2 (拇指伸縮)
///   F3 = M3 (食指)
///   F4 = M4 (中指)
///   F5 = M5 (無名指)
///   F6 = M6 (尾指)
/// </summary>
public class EHandFingerReader : MonoBehaviour
{
    [Header("=== ROS Connection Override ===")]
    [Tooltip("如果設定，將使用這個 ROSConnection 實例")]
    [SerializeField] private ROSConnection rosOverride;

    [Header("=== Body Animator ===")]
    [Tooltip("RealisticCharacter 的 Animator (Humanoid Avatar)")]
    public Animator bodyAnimator;

    [Header("=== Topic 設定 ===")]
    [Tooltip("手指命令 Topic")]
    public string ehandCommandsTopic = "/unity/ehand_commands";

    [Header("=== 發送設定 ===")]
    [Tooltip("發送頻率 (Hz)")]
    [Range(10, 60)]
    public float publishRate = 30f;

    [Tooltip("啟用左手")]
    public bool enableLeftHand = true;

    [Tooltip("啟用右手")]
    public bool enableRightHand = true;

    [Header("=== 手指角度映射 ===")]
    [Tooltip("完全張開時的角度 (度)")]
    public float openAngle = 0f;

    [Tooltip("完全握緊時的角度 (度)")]
    public float closeAngle = 90f;

    [Header("=== 狀態監控 ===")]
    [SerializeField] private bool rosConnected = false;
    [SerializeField] private int messagesSent = 0;
    [SerializeField] private float[] leftFingerValues = new float[6];
    [SerializeField] private float[] rightFingerValues = new float[6];

    // 手指骨骼
    private Transform leftThumbProximal, leftThumbDistal;
    private Transform leftIndexProximal, leftMiddleProximal, leftRingProximal, leftLittleProximal;
    private Transform rightThumbProximal, rightThumbDistal;
    private Transform rightIndexProximal, rightMiddleProximal, rightRingProximal, rightLittleProximal;

    // ROS
    private ROSConnection ros;
    private float lastPublishTime;

    void Start()
    {
        Debug.Log("[EHandFingerReader] === 初始化開始 ===");
        
        // 取得手指骨骼
        if (bodyAnimator != null && bodyAnimator.isHuman)
        {
            InitializeFingerBones();
        }
        else
        {
            Debug.LogError("[EHandFingerReader] bodyAnimator 未設定或不是 Humanoid Avatar！");
        }

        // 延遲訂閱，確保 ROSConnection 已經初始化完成
        StartCoroutine(DelayedInitialize());
    }

    /// <summary>
    /// 初始化手指骨骼 Transform
    /// </summary>
    private void InitializeFingerBones()
    {
        // 左手
        leftThumbProximal = bodyAnimator.GetBoneTransform(HumanBodyBones.LeftThumbProximal);
        leftThumbDistal = bodyAnimator.GetBoneTransform(HumanBodyBones.LeftThumbDistal);
        leftIndexProximal = bodyAnimator.GetBoneTransform(HumanBodyBones.LeftIndexProximal);
        leftMiddleProximal = bodyAnimator.GetBoneTransform(HumanBodyBones.LeftMiddleProximal);
        leftRingProximal = bodyAnimator.GetBoneTransform(HumanBodyBones.LeftRingProximal);
        leftLittleProximal = bodyAnimator.GetBoneTransform(HumanBodyBones.LeftLittleProximal);

        // 右手
        rightThumbProximal = bodyAnimator.GetBoneTransform(HumanBodyBones.RightThumbProximal);
        rightThumbDistal = bodyAnimator.GetBoneTransform(HumanBodyBones.RightThumbDistal);
        rightIndexProximal = bodyAnimator.GetBoneTransform(HumanBodyBones.RightIndexProximal);
        rightMiddleProximal = bodyAnimator.GetBoneTransform(HumanBodyBones.RightMiddleProximal);
        rightRingProximal = bodyAnimator.GetBoneTransform(HumanBodyBones.RightRingProximal);
        rightLittleProximal = bodyAnimator.GetBoneTransform(HumanBodyBones.RightLittleProximal);

        Debug.Log("[EHandFingerReader] 手指骨骼初始化完成");
        Debug.Log($"  Left: Thumb={leftThumbProximal != null}, Index={leftIndexProximal != null}");
        Debug.Log($"  Right: Thumb={rightThumbProximal != null}, Index={rightIndexProximal != null}");
    }

    /// <summary>
    /// 延遲初始化 - 等待 ROSConnection 連線穩定
    /// </summary>
    IEnumerator DelayedInitialize()
    {
        Debug.Log("[EHandFingerReader] 等待 2.0 秒讓 ROSConnection 連線穩定...");
        yield return new WaitForSecondsRealtime(2.0f);

        ros = rosOverride != null ? rosOverride : RosConn.GetSceneROS();
        if (ros == null)
        {
            Debug.LogError("[EHandFingerReader] Scene 裡找不到 ROSConnection！");
            rosConnected = false;
            yield break;
        }

        rosConnected = true;
        Debug.Log($"[EHandFingerReader] Using ROSConnection: {ros.gameObject.name}");

        // 註冊 Publisher
        ros.RegisterPublisher<JointStateMsg>(ehandCommandsTopic);
        Debug.Log($"[EHandFingerReader] ✓ Registered publisher: {ehandCommandsTopic}");

        Debug.Log("[EHandFingerReader] === 初始化完成 ===");
    }

    void Update()
    {
        if (!rosConnected || ros == null) return;

        // 頻率控制
        if (Time.time - lastPublishTime < 1f / publishRate) return;
        lastPublishTime = Time.time;

        // 讀取手指角度
        if (enableLeftHand) ReadLeftFingers();
        if (enableRightHand) ReadRightFingers();

        // 發送 ROS 訊息
        PublishEHandCommands();
    }

    /// <summary>
    /// 讀取左手手指角度
    /// </summary>
    private void ReadLeftFingers()
    {
        leftFingerValues[0] = GetFingerBend(leftThumbProximal);   // F1: 拇指旋轉
        leftFingerValues[1] = GetFingerBend(leftThumbDistal);    // F2: 拇指伸縮
        leftFingerValues[2] = GetFingerBend(leftIndexProximal);  // F3: 食指
        leftFingerValues[3] = GetFingerBend(leftMiddleProximal); // F4: 中指
        leftFingerValues[4] = GetFingerBend(leftRingProximal);   // F5: 無名指
        leftFingerValues[5] = GetFingerBend(leftLittleProximal); // F6: 尾指
    }

    /// <summary>
    /// 讀取右手手指角度
    /// </summary>
    private void ReadRightFingers()
    {
        rightFingerValues[0] = GetFingerBend(rightThumbProximal);   // F1: 拇指旋轉
        rightFingerValues[1] = GetFingerBend(rightThumbDistal);    // F2: 拇指伸縮
        rightFingerValues[2] = GetFingerBend(rightIndexProximal);  // F3: 食指
        rightFingerValues[3] = GetFingerBend(rightMiddleProximal); // F4: 中指
        rightFingerValues[4] = GetFingerBend(rightRingProximal);   // F5: 無名指
        rightFingerValues[5] = GetFingerBend(rightLittleProximal); // F6: 尾指
    }

    /// <summary>
    /// 計算手指彎曲程度 (0=張開, 1=握緊)
    /// </summary>
    private float GetFingerBend(Transform fingerBone)
    {
        if (fingerBone == null) return 0f;

        // 取得 local rotation 的 X 軸角度（手指彎曲方向）
        // 注意：不同骨骼可能需要調整軸向
        float angle = fingerBone.localEulerAngles.x;
        
        // 將 Unity 角度 (0~360) 轉換為 -180~180
        if (angle > 180f) angle -= 360f;

        // 映射到 0~1
        float bend = Mathf.InverseLerp(openAngle, closeAngle, angle);
        return Mathf.Clamp01(bend);
    }

    /// <summary>
    /// 發送手指命令到 ROS
    /// </summary>
    private void PublishEHandCommands()
    {
        var msg = new JointStateMsg();

        // Header
        var now = System.DateTimeOffset.UtcNow;
        msg.header = new HeaderMsg();
        msg.header.stamp = new TimeMsg
        {
            sec = (int)now.ToUnixTimeSeconds(),
            nanosec = (uint)((now.ToUnixTimeMilliseconds() % 1000) * 1000000)
        };
        msg.header.frame_id = "unity";

        // 計算關節數量
        int jointCount = 0;
        if (enableLeftHand) jointCount += 6;
        if (enableRightHand) jointCount += 6;

        msg.name = new string[jointCount];
        msg.position = new double[jointCount];
        msg.velocity = new double[jointCount];
        msg.effort = new double[jointCount];

        int idx = 0;

        // 左手
        if (enableLeftHand)
        {
            for (int i = 0; i < 6; i++)
            {
                msg.name[idx] = $"L_F{i + 1}";
                msg.position[idx] = leftFingerValues[i];
                msg.velocity[idx] = 0.0;
                msg.effort[idx] = 0.0;
                idx++;
            }
        }

        // 右手
        if (enableRightHand)
        {
            for (int i = 0; i < 6; i++)
            {
                msg.name[idx] = $"R_F{i + 1}";
                msg.position[idx] = rightFingerValues[i];
                msg.velocity[idx] = 0.0;
                msg.effort[idx] = 0.0;
                idx++;
            }
        }

        ros.Publish(ehandCommandsTopic, msg);
        messagesSent++;
    }

    /// <summary>
    /// 取得左手手指值 (供外部讀取)
    /// </summary>
    public float[] GetLeftFingerPositions()
    {
        return (float[])leftFingerValues.Clone();
    }

    /// <summary>
    /// 取得右手手指值 (供外部讀取)
    /// </summary>
    public float[] GetRightFingerPositions()
    {
        return (float[])rightFingerValues.Clone();
    }

    /// <summary>
    /// 檢查是否連線
    /// </summary>
    public bool IsConnected => rosConnected;
}
