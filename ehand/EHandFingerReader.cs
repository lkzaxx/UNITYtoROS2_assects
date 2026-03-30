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

    [Header("=== 骨骼來源（二選一）===")]
    [Tooltip("使用 Animator 自動取得骨骼（傳統方式）")]
    public Animator bodyAnimator;
    
    [Tooltip("使用直接 Transform 參考（Meta Movement SDK 專用）")]
    public bool useDirectTransforms = false;

    [Header("=== 左手手指 Transform（多關節模式）===")]
    public Transform leftHandWrist;  // 手掌根骨骼
    
    // 拇指（雙關節，使用 Y/Z 軸）
    public Transform leftThumbProximal;
    public Transform leftThumbDistal;
    
    // 食指（三關節）
    public Transform leftIndexProximal;
    public Transform leftIndexIntermediate;  // 🆕 中段關節
    public Transform leftIndexDistal;        // 🆕 末端關節
    
    // 中指（三關節）
    public Transform leftMiddleProximal;
    public Transform leftMiddleIntermediate; // 🆕 中段關節
    public Transform leftMiddleDistal;       // 🆕 末端關節
    
    // 無名指（三關節）
    public Transform leftRingProximal;
    public Transform leftRingIntermediate;   // 🆕 中段關節
    public Transform leftRingDistal;         // 🆕 末端關節
    
    // 尾指（三關節）
    public Transform leftLittleProximal;
    public Transform leftLittleIntermediate; // 🆕 中段關節
    public Transform leftLittleDistal;       // 🆕 末端關節

    [Header("=== 右手手指 Transform（多關節模式）===")]
    public Transform rightHandWrist;  // 手掌根骨骼
    
    // 拇指（雙關節，使用 Y/Z 軸）
    public Transform rightThumbProximal;
    public Transform rightThumbDistal;
    
    // 食指（三關節）
    public Transform rightIndexProximal;
    public Transform rightIndexIntermediate;  // 🆕 中段關節
    public Transform rightIndexDistal;        // 🆕 末端關節
    
    // 中指（三關節）
    public Transform rightMiddleProximal;
    public Transform rightMiddleIntermediate; // 🆕 中段關節
    public Transform rightMiddleDistal;       // 🆕 末端關節
    
    // 無名指（三關節）
    public Transform rightRingProximal;
    public Transform rightRingIntermediate;   // 🆕 中段關節
    public Transform rightRingDistal;         // 🆕 末端關節
    
    // 尾指（三關節）
    public Transform rightLittleProximal;
    public Transform rightLittleIntermediate; // 🆕 中段關節
    public Transform rightLittleDistal;       // 🆕 末端關節

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

    [Header("=== 手指角度映射（基於實測數據優化）===")]
    [Tooltip("食指張開角度 (度) - 涵蓋所有伸直姿態（張開手 ~0°, 比1 ~-30°）")]
    public float indexOpenAngle = -30f;
    
    [Tooltip("食指握緊角度 (度) - 基於實際握拳數據")]
    public float indexCloseAngle = -165f;
    
    [Tooltip("中指張開角度 (度) - 張開手時約 +12°")]
    public float middleOpenAngle = +12f;
    
    [Tooltip("中指握緊角度 (度) - 基於實際握拳數據")]
    public float middleCloseAngle = -160f;
    
    [Tooltip("無名指張開角度 (度) - 張開手時約 +15°")]
    public float ringOpenAngle = +15f;
    
    [Tooltip("無名指握緊角度 (度) - 基於實際握拳數據")]
    public float ringCloseAngle = -163f;
    
    [Tooltip("尾指張開角度 (度) - 張開手時約 +20°")]
    public float littleOpenAngle = +20f;
    
    [Tooltip("尾指握緊角度 (度) - 基於實際握拳數據")]
    public float littleCloseAngle = -180f;
    
    [Header("=== 拇指專用角度 ===")]
    [Tooltip("拇指 Y 軸（旋轉）張開角度")]
    public float thumbRotateOpen = +10f;
    
    [Tooltip("拇指 Y 軸（旋轉）握緊角度")]
    public float thumbRotateClose = +16f;
    
    [Tooltip("拇指 Z 軸（彎曲）張開角度")]
    public float thumbBendOpen = 0f;
    
    [Tooltip("拇指 Z 軸（彎曲）握緊角度")]
    public float thumbBendClose = -35f;

    [Header("=== VR 側鍵控制（與 ROSTCPManager 連動）===")]
    [Tooltip("開啟後，只有按住側鍵才會發送手指訊息到 ROS2")]
    public bool requireSideButtonToSend = true;
    [Tooltip("任一手按住側鍵即可同時發送雙手手指（與 ROSTCPManager.linkedSideButton 同理）")]
    public bool linkedSideButton = true;

    [Header("=== 狀態監控 ===")]
    [SerializeField] private bool rosConnected = false;
    [SerializeField] private int messagesSent = 0;
    [SerializeField] private float[] leftFingerValues = new float[6];
    [SerializeField] private float[] rightFingerValues = new float[6];

    // ROS
    private ROSConnection ros;
    private float lastPublishTime;

    void Start()
    {
        Debug.Log("[EHandFingerReader] === 初始化開始 ===");
        
        // 取得手指骨骼
        if (!useDirectTransforms && bodyAnimator != null && bodyAnimator.isHuman)
        {
            InitializeFromAnimator();
        }
        else if (useDirectTransforms)
        {
            Debug.Log("[EHandFingerReader] 使用直接 Transform 參考模式");
            ValidateDirectTransforms();
        }
        else
        {
            Debug.LogWarning("[EHandFingerReader] 請設定 Body Animator 或啟用 Use Direct Transforms 並拖入骨骼");
        }

        // 延遲訂閱，確保 ROSConnection 已經初始化完成
        StartCoroutine(DelayedInitialize());
    }

    /// <summary>
    /// 從 Animator 初始化手指骨骼 Transform（傳統方式）
    /// </summary>
    private void InitializeFromAnimator()
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

        Debug.Log("[EHandFingerReader] 從 Animator 初始化手指骨骼完成");
        ValidateDirectTransforms();
    }

    /// <summary>
    /// 驗證 Transform 參考
    /// </summary>
    private void ValidateDirectTransforms()
    {
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

        // 讀取手指角度（持續讀取，保持平滑插值不中斷）
        if (enableLeftHand) ReadLeftFingers();
        if (enableRightHand) ReadRightFingers();

        // 測試模式：強制輸出指定值
        if (forceTestMode)
        {
            for (int i = 0; i < 6; i++)
            {
                leftFingerValues[i] = forceTestValue;
                rightFingerValues[i] = forceTestValue;
            }
        }

        // 側鍵控制：判斷是否允許發送
        bool canSendLeft = true;
        bool canSendRight = true;

        if (requireSideButtonToSend && ROSTCPManager.Instance != null)
        {
            bool leftPressed = ROSTCPManager.Instance.IsLeftSideButtonPressed();
            bool rightPressed = ROSTCPManager.Instance.IsRightSideButtonPressed();
            bool eitherPressed = leftPressed || rightPressed;

            canSendLeft = linkedSideButton ? eitherPressed : leftPressed;
            canSendRight = linkedSideButton ? eitherPressed : rightPressed;
        }

        // 發送 ROS 訊息（只發送有權限的手）
        if (canSendLeft || canSendRight)
            PublishEHandCommands(canSendLeft, canSendRight);
    }
    
    [Header("=== 強制測試模式 ===")]
    [Tooltip("啟用後強制輸出指定值，用於測試靈巧手")]
    public bool forceTestMode = false;
    
    [Tooltip("強制輸出的值 (0~1)")]
    [Range(0f, 1f)]
    public float forceTestValue = 1.0f;

    [Header("=== 平滑設定 ===")]
    [Tooltip("平滑係數 (0~1)，越小越平滑但延遲越高")]
    public float smoothness = 0.5f;
    
    private float[] targetLeftFingerValues = new float[6];
    private float[] targetRightFingerValues = new float[6];

    /// <summary>
    /// 讀取左手手指角度
    /// </summary>
    private void ReadLeftFingers()
    {
        // 讀取目標值（拇指使用專屬函數，四指使用三關節累加）
        targetLeftFingerValues[0] = GetThumbBend(leftThumbProximal, 1, thumbRotateOpen, thumbRotateClose);
        targetLeftFingerValues[1] = GetThumbBend(leftThumbProximal, 2, thumbBendOpen, thumbBendClose);
        
        // 四指（三關節累加）
        targetLeftFingerValues[2] = GetFingerBend(
            leftIndexProximal, leftIndexIntermediate, leftIndexDistal,
            indexOpenAngle, indexCloseAngle);
        targetLeftFingerValues[3] = GetFingerBend(
            leftMiddleProximal, leftMiddleIntermediate, leftMiddleDistal,
            middleOpenAngle, middleCloseAngle);
        targetLeftFingerValues[4] = GetFingerBend(
            leftRingProximal, leftRingIntermediate, leftRingDistal,
            ringOpenAngle, ringCloseAngle);
        targetLeftFingerValues[5] = GetFingerBend(
            leftLittleProximal, leftLittleIntermediate, leftLittleDistal,
            littleOpenAngle, littleCloseAngle);
        
        // 平滑插值
        for (int i = 0; i < 6; i++)
        {
            leftFingerValues[i] = Mathf.Lerp(leftFingerValues[i], targetLeftFingerValues[i], 1f - smoothness);
        }
    }

    /// <summary>
    /// 讀取右手手指角度
    /// </summary>
    private void ReadRightFingers()
    {
        // 讀取目標值（拇指使用專屬函數，四指使用三關節累加）
        targetRightFingerValues[0] = GetThumbBend(rightThumbProximal, 1, thumbRotateOpen, thumbRotateClose);
        targetRightFingerValues[1] = GetThumbBend(rightThumbProximal, 2, thumbBendOpen, thumbBendClose);
        
        // 四指（三關節累加）
        targetRightFingerValues[2] = GetFingerBend(
            rightIndexProximal, rightIndexIntermediate, rightIndexDistal,
            indexOpenAngle, indexCloseAngle);
        targetRightFingerValues[3] = GetFingerBend(
            rightMiddleProximal, rightMiddleIntermediate, rightMiddleDistal,
            middleOpenAngle, middleCloseAngle);
        targetRightFingerValues[4] = GetFingerBend(
            rightRingProximal, rightRingIntermediate, rightRingDistal,
            ringOpenAngle, ringCloseAngle);
        targetRightFingerValues[5] = GetFingerBend(
            rightLittleProximal, rightLittleIntermediate, rightLittleDistal,
            littleOpenAngle, littleCloseAngle);
        
        // 平滑插值
        for (int i = 0; i < 6; i++)
        {
            rightFingerValues[i] = Mathf.Lerp(rightFingerValues[i], targetRightFingerValues[i], 1f - smoothness);
        }
    }
    
    
    /// <summary>
    /// 計算拇指彎曲程度（使用專用角度設定）
    /// </summary>
    private float GetThumbBend(Transform fingerBone, int axis, float open, float close)
    {
        if (fingerBone == null) return 0f;

        Vector3 euler = fingerBone.localEulerAngles;
        float angle = axis == 0 ? euler.x : (axis == 1 ? euler.y : euler.z);
        
        if (angle > 180f) angle -= 360f;

        float bend = Mathf.InverseLerp(open, close, angle);
        return Mathf.Clamp01(bend);
    }

    [Header("=== 除錯 ===")]
    [Tooltip("顯示原始角度值")]
    public bool debugMode = false;
    
    [Tooltip("使用的旋轉軸 (0=X, 1=Y, 2=Z)")]
    [Range(0, 2)]
    public int bendAxis = 2;  // Z 軸（適用於 Meta Movement SDK）
    
    private float debugTimer = 0f;

    /// <summary>
    /// 計算手指彎曲程度 (0=張開, 1=握緊)
    /// 使用三關節累加（Proximal + Intermediate + Distal）
    /// 符合 Unity XR Hands 標準做法
    /// 
    /// 特殊處理：Distal 正值抑制
    /// - 手指伸直時，Distal 會產生正值（過度伸展）
    /// - 這會抵消 P+I 的負值，導致總角度不準確
    /// - 將 Distal 正值視為 0，只累加負值部分
    /// </summary>
    private float GetFingerBend(
        Transform proximal,
        Transform intermediate,
        Transform distal,
        float openAngle,
        float closeAngle)
    {
        if (proximal == null || intermediate == null || distal == null)
            return 0f;

        // 取得各關節角度
        float pAngle = GetJointAngle(proximal);
        float iAngle = GetJointAngle(intermediate);
        float dAngle = GetJointAngle(distal);
        
        // Distal 正值抑制：忽略過度伸展（只計算彎曲部分）
        if (dAngle > 0f)
        {
            dAngle = 0f;
        }

        // 累加三個關節的 Z 軸角度
        float totalAngle = pAngle + iAngle + dAngle;

        // 映射到 0~1
        float bend = Mathf.InverseLerp(openAngle, closeAngle, totalAngle);
        return Mathf.Clamp01(bend);
    }

    /// <summary>
    /// 取得單一關節的 Z 軸角度（-180~180）
    /// </summary>
    private float GetJointAngle(Transform joint)
    {
        if (joint == null) return 0f;
        
        float angle = joint.localRotation.eulerAngles.z;
        if (angle > 180f) angle -= 360f;
        return angle;
    }
    
    void LateUpdate()
    {
        // 除錯：每秒輸出一次角度資訊
        if (debugMode)
        {
            debugTimer += Time.deltaTime;
            if (debugTimer >= 1.0f)
            {
                debugTimer = 0f;
                
                Debug.Log("=== 左手手指角度 [localRotation Z軸] ===");
                
                // 拇指近端 Debug (F1 - 旋轉)
                if (leftThumbProximal != null && leftHandWrist != null)
                {
                    Vector3 thumbEuler = leftThumbProximal.localEulerAngles;
                    float thumbY = thumbEuler.y > 180f ? thumbEuler.y - 360f : thumbEuler.y;
                    float thumbZ = thumbEuler.z > 180f ? thumbEuler.z - 360f : thumbEuler.z;
                    Debug.Log($"[F1 拇指旋轉] Y={thumbY:F1}° (localY), Z={thumbZ:F1}° (localZ) → output={leftFingerValues[0]:F2}");
                }
                
                // 拇指末端 Debug (F2 - 伸縮)
                if (leftThumbProximal != null && leftHandWrist != null)
                {
                    Vector3 thumbEuler = leftThumbProximal.localEulerAngles;
                    float thumbY = thumbEuler.y > 180f ? thumbEuler.y - 360f : thumbEuler.y;
                    float thumbZ = thumbEuler.z > 180f ? thumbEuler.z - 360f : thumbEuler.z;
                    Debug.Log($"[F2 拇指伸縮] Y={thumbY:F1}° (localY), Z={thumbZ:F1}° (localZ) → output={leftFingerValues[1]:F2}");
                }
                
                // 食指 Debug (F3) - 三關節累加
                if (leftIndexProximal != null)
                {
                    float p = GetJointAngle(leftIndexProximal);
                    float i = GetJointAngle(leftIndexIntermediate);
                    float d = GetJointAngle(leftIndexDistal);
                    float total = p + i + d;
                    Debug.Log($"[F3 食指] 總角度={total:F1}° (P:{p:F1}° I:{i:F1}° D:{d:F1}°) → output={leftFingerValues[2]:F2}");
                }
                
                // 中指 Debug (F4) - 三關節累加
                if (leftMiddleProximal != null)
                {
                    float p = GetJointAngle(leftMiddleProximal);
                    float i = GetJointAngle(leftMiddleIntermediate);
                    float d = GetJointAngle(leftMiddleDistal);
                    float total = p + i + d;
                    Debug.Log($"[F4 中指] 總角度={total:F1}° (P:{p:F1}° I:{i:F1}° D:{d:F1}°) → output={leftFingerValues[3]:F2}");
                }
                
                // 無名指 Debug (F5) - 三關節累加
                if (leftRingProximal != null)
                {
                    float p = GetJointAngle(leftRingProximal);
                    float i = GetJointAngle(leftRingIntermediate);
                    float d = GetJointAngle(leftRingDistal);
                    float total = p + i + d;
                    Debug.Log($"[F5 無名指] 總角度={total:F1}° (P:{p:F1}° I:{i:F1}° D:{d:F1}°) → output={leftFingerValues[4]:F2}");
                }
                
                // 尾指 Debug (F6) - 三關節累加
                if (leftLittleProximal != null)
                {
                    float p = GetJointAngle(leftLittleProximal);
                    float i = GetJointAngle(leftLittleIntermediate);
                    float d = GetJointAngle(leftLittleDistal);
                    float total = p + i + d;
                    Debug.Log($"[F6 尾指] 總角度={total:F1}° (P:{p:F1}° I:{i:F1}° D:{d:F1}°) → output={leftFingerValues[5]:F2}");
                }
                
                Debug.Log("==================");
            }
        }
    }

    /// <summary>
    /// 發送手指命令到 ROS（根據側鍵狀態決定發送哪些手）
    /// </summary>
    private void PublishEHandCommands(bool sendLeft = true, bool sendRight = true)
    {
        bool actualLeft = enableLeftHand && sendLeft;
        bool actualRight = enableRightHand && sendRight;

        if (!actualLeft && !actualRight) return;

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
        if (actualLeft) jointCount += 6;
        if (actualRight) jointCount += 6;

        msg.name = new string[jointCount];
        msg.position = new double[jointCount];
        msg.velocity = new double[jointCount];
        msg.effort = new double[jointCount];

        int idx = 0;

        // 左手
        if (actualLeft)
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
        if (actualRight)
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

    // ========================================
    // 動態校準功能
    // ========================================

    [Header("=== 動態校準 ===")]
    [Tooltip("顯示當前手指的原始角度")]
    public bool showCurrentAngles = false;

    /// <summary>
    /// 校準當前手勢為「張開」極值
    /// 在 Unity Inspector 中右鍵點擊組件 → "校準當前手勢為張開"
    /// </summary>
    [ContextMenu("校準當前手勢為張開")]
    public void CalibrateOpen()
    {
        if (leftIndexProximal == null)
        {
            Debug.LogError("[Calibrate] 找不到手指骨骼！請先設定 Transform 參考。");
            return;
        }

        Debug.Log("=== 開始校準「張開」極值 ===");

        // 讀取當前角度（三關節累加）
        float indexZ = GetCurrentAngle(leftIndexProximal, leftIndexIntermediate, leftIndexDistal);
        float middleZ = GetCurrentAngle(leftMiddleProximal, leftMiddleIntermediate, leftMiddleDistal);
        float ringZ = GetCurrentAngle(leftRingProximal, leftRingIntermediate, leftRingDistal);
        float littleZ = GetCurrentAngle(leftLittleProximal, leftLittleIntermediate, leftLittleDistal);
        
        Vector3 thumbEuler = leftThumbProximal.localEulerAngles;
        float thumbY = thumbEuler.y > 180f ? thumbEuler.y - 360f : thumbEuler.y;
        float thumbZ = thumbEuler.z > 180f ? thumbEuler.z - 360f : thumbEuler.z;

        // 更新參數
        indexOpenAngle = indexZ;
        middleOpenAngle = middleZ;
        ringOpenAngle = ringZ;
        littleOpenAngle = littleZ;
        thumbRotateOpen = thumbY;
        thumbBendOpen = thumbZ;

        // 輸出結果
        Debug.Log($"[食指] openAngle = {indexZ:F1}°");
        Debug.Log($"[中指] openAngle = {middleZ:F1}°");
        Debug.Log($"[無名指] openAngle = {ringZ:F1}°");
        Debug.Log($"[尾指] openAngle = {littleZ:F1}°");
        Debug.Log($"[拇指Y] openAngle = {thumbY:F1}°");
        Debug.Log($"[拇指Z] openAngle = {thumbZ:F1}°");
        Debug.Log("=== 校準完成！===");
    }

    /// <summary>
    /// 校準當前手勢為「握緊」極值
    /// 在 Unity Inspector 中右鍵點擊組件 → "校準當前手勢為握緊"
    /// </summary>
    [ContextMenu("校準當前手勢為握緊")]
    public void CalibrateClosed()
    {
        if (leftIndexProximal == null)
        {
            Debug.LogError("[Calibrate] 找不到手指骨骼！請先設定 Transform 參考。");
            return;
        }

        Debug.Log("=== 開始校準「握緊」極值 ===");

        // 讀取當前角度（三關節累加）
        float indexZ = GetCurrentAngle(leftIndexProximal, leftIndexIntermediate, leftIndexDistal);
        float middleZ = GetCurrentAngle(leftMiddleProximal, leftMiddleIntermediate, leftMiddleDistal);
        float ringZ = GetCurrentAngle(leftRingProximal, leftRingIntermediate, leftRingDistal);
        float littleZ = GetCurrentAngle(leftLittleProximal, leftLittleIntermediate, leftLittleDistal);
        
        Vector3 thumbEuler = leftThumbProximal.localEulerAngles;
        float thumbY = thumbEuler.y > 180f ? thumbEuler.y - 360f : thumbEuler.y;
        float thumbZ = thumbEuler.z > 180f ? thumbEuler.z - 360f : thumbEuler.z;

        // 更新參數
        indexCloseAngle = indexZ;
        middleCloseAngle = middleZ;
        ringCloseAngle = ringZ;
        littleCloseAngle = littleZ;
        thumbRotateClose = thumbY;
        thumbBendClose = thumbZ;

        // 輸出結果
        Debug.Log($"[食指] closeAngle = {indexZ:F1}°");
        Debug.Log($"[中指] closeAngle = {middleZ:F1}°");
        Debug.Log($"[無名指] closeAngle = {ringZ:F1}°");
        Debug.Log($"[尾指] closeAngle = {littleZ:F1}°");
        Debug.Log($"[拇指Y] closeAngle = {thumbY:F1}°");
        Debug.Log($"[拇指Z] closeAngle = {thumbZ:F1}°");
        Debug.Log("=== 校準完成！===");
    }

    /// <summary>
    /// 顯示當前所有手指的原始角度（用於調試）
    /// </summary>
    [ContextMenu("顯示當前手指角度")]
    public void ShowCurrentAngles()
    {
        if (leftIndexProximal == null)
        {
            Debug.LogError("[ShowAngles] 找不到手指骨骼！");
            return;
        }

        Debug.Log("=== 當前手指角度（三關節累加）===");
        Debug.Log($"[食指] 總角度 = {GetCurrentAngle(leftIndexProximal, leftIndexIntermediate, leftIndexDistal):F1}°");
        Debug.Log($"[中指] 總角度 = {GetCurrentAngle(leftMiddleProximal, leftMiddleIntermediate, leftMiddleDistal):F1}°");
        Debug.Log($"[無名指] 總角度 = {GetCurrentAngle(leftRingProximal, leftRingIntermediate, leftRingDistal):F1}°");
        Debug.Log($"[尾指] 總角度 = {GetCurrentAngle(leftLittleProximal, leftLittleIntermediate, leftLittleDistal):F1}°");
        
        Vector3 thumbEuler = leftThumbProximal.localEulerAngles;
        float thumbY = thumbEuler.y > 180f ? thumbEuler.y - 360f : thumbEuler.y;
        float thumbZ = thumbEuler.z > 180f ? thumbEuler.z - 360f : thumbEuler.z;
        Debug.Log($"[拇指] Y = {thumbY:F1}°, Z = {thumbZ:F1}°");
        Debug.Log("==================");
    }

    /// <summary>
    /// 取得指定手指當前的總角度（三關節累加，-180~180）
    /// </summary>
    private float GetCurrentAngle(Transform proximal, Transform intermediate, Transform distal)
    {
        if (proximal == null || intermediate == null || distal == null)
            return 0f;
        
        return GetJointAngle(proximal) + GetJointAngle(intermediate) + GetJointAngle(distal);
    }

    /// <summary>
    /// 輸出當前校準參數（用於複製到其他場景）
    /// </summary>
    [ContextMenu("輸出校準參數")]
    public void PrintCalibrationData()
    {
        Debug.Log("=== 當前校準參數 ===");
        Debug.Log($"indexOpenAngle = {indexOpenAngle:F1}f;");
        Debug.Log($"indexCloseAngle = {indexCloseAngle:F1}f;");
        Debug.Log($"middleOpenAngle = {middleOpenAngle:F1}f;");
        Debug.Log($"middleCloseAngle = {middleCloseAngle:F1}f;");
        Debug.Log($"ringOpenAngle = {ringOpenAngle:F1}f;");
        Debug.Log($"ringCloseAngle = {ringCloseAngle:F1}f;");
        Debug.Log($"littleOpenAngle = {littleOpenAngle:F1}f;");
        Debug.Log($"littleCloseAngle = {littleCloseAngle:F1}f;");
        Debug.Log($"thumbRotateOpen = {thumbRotateOpen:F1}f;");
        Debug.Log($"thumbRotateClose = {thumbRotateClose:F1}f;");
        Debug.Log($"thumbBendOpen = {thumbBendOpen:F1}f;");
        Debug.Log($"thumbBendClose = {thumbBendClose:F1}f;");
        Debug.Log("==================");
    }
}
