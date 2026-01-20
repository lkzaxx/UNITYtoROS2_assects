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

    [Header("=== 左手手指 Transform（直接參考模式）===")]
    public Transform leftThumbProximal;
    public Transform leftThumbDistal;
    public Transform leftIndexProximal;
    public Transform leftMiddleProximal;
    public Transform leftRingProximal;
    public Transform leftLittleProximal;

    [Header("=== 右手手指 Transform（直接參考模式）===")]
    public Transform rightThumbProximal;
    public Transform rightThumbDistal;
    public Transform rightIndexProximal;
    public Transform rightMiddleProximal;
    public Transform rightRingProximal;
    public Transform rightLittleProximal;

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

    [Header("=== 手指角度映射（每根手指獨立設定）===")]
    [Tooltip("食指張開角度 (度)")]
    public float indexOpenAngle = -7f;
    
    [Tooltip("食指握緊角度 (度)")]
    public float indexCloseAngle = -58f;
    
    [Tooltip("中指張開角度 (度)")]
    public float middleOpenAngle = 0f;
    
    [Tooltip("中指握緊角度 (度)")]
    public float middleCloseAngle = -60f;
    
    [Tooltip("無名指張開角度 (度)")]
    public float ringOpenAngle = 10f;
    
    [Tooltip("無名指握緊角度 (度)")]
    public float ringCloseAngle = -67f;
    
    [Tooltip("尾指張開角度 (度)")]
    public float littleOpenAngle = 16f;
    
    [Tooltip("尾指握緊角度 (度)")]
    public float littleCloseAngle = -78f;
    
    [Header("=== 拇指專用角度 ===")]
    [Tooltip("拇指 Y 軸（旋轉）張開角度")]
    public float thumbRotateOpen = 9f;
    
    [Tooltip("拇指 Y 軸（旋轉）握緊角度")]
    public float thumbRotateClose = 17f;
    
    [Tooltip("拇指 Z 軸（彎曲）張開角度")]
    public float thumbBendOpen = 0f;
    
    [Tooltip("拇指 Z 軸（彎曲）握緊角度")]
    public float thumbBendClose = -40f;

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

        // 讀取手指角度
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

        // 發送 ROS 訊息
        PublishEHandCommands();
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
        // 讀取目標值（每根手指使用專屬角度範圍）
        targetLeftFingerValues[0] = GetThumbBend(leftThumbProximal, 1, thumbRotateOpen, thumbRotateClose);
        targetLeftFingerValues[1] = GetThumbBend(leftThumbProximal, 2, thumbBendOpen, thumbBendClose);
        targetLeftFingerValues[2] = GetFingerBend(leftIndexProximal, indexOpenAngle, indexCloseAngle);
        targetLeftFingerValues[3] = GetFingerBend(leftMiddleProximal, middleOpenAngle, middleCloseAngle);
        targetLeftFingerValues[4] = GetFingerBend(leftRingProximal, ringOpenAngle, ringCloseAngle);
        targetLeftFingerValues[5] = GetFingerBend(leftLittleProximal, littleOpenAngle, littleCloseAngle);
        
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
        // 讀取目標值（每根手指使用專屬角度範圍）
        targetRightFingerValues[0] = GetThumbBend(rightThumbProximal, 1, thumbRotateOpen, thumbRotateClose);
        targetRightFingerValues[1] = GetThumbBend(rightThumbProximal, 2, thumbBendOpen, thumbBendClose);
        targetRightFingerValues[2] = GetFingerBend(rightIndexProximal, indexOpenAngle, indexCloseAngle);
        targetRightFingerValues[3] = GetFingerBend(rightMiddleProximal, middleOpenAngle, middleCloseAngle);
        targetRightFingerValues[4] = GetFingerBend(rightRingProximal, ringOpenAngle, ringCloseAngle);
        targetRightFingerValues[5] = GetFingerBend(rightLittleProximal, littleOpenAngle, littleCloseAngle);
        
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
    /// </summary>
    private float GetFingerBend(Transform fingerBone, float openAngle, float closeAngle)
    {
        if (fingerBone == null) return 0f;

        // 取得 local rotation 的角度（根據設定的軸向）
        Vector3 euler = fingerBone.localEulerAngles;
        float angle = bendAxis == 0 ? euler.x : (bendAxis == 1 ? euler.y : euler.z);
        
        // 將 Unity 角度 (0~360) 轉換為 -180~180
        if (angle > 180f) angle -= 360f;

        // 映射到 0~1
        float bend = Mathf.InverseLerp(openAngle, closeAngle, angle);
        return Mathf.Clamp01(bend);
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
                
                string axisName = bendAxis == 0 ? "X" : (bendAxis == 1 ? "Y" : "Z");
                Debug.Log($"=== 左手手指角度 [計算軸: {axisName}] ===");
                
                // 拇指近端 Debug (F1 - 旋轉)
                if (leftThumbProximal != null)
                {
                    Vector3 euler = leftThumbProximal.localEulerAngles;
                    float x = euler.x > 180f ? euler.x - 360f : euler.x;
                    float y = euler.y > 180f ? euler.y - 360f : euler.y;
                    float z = euler.z > 180f ? euler.z - 360f : euler.z;
                    Debug.Log($"[F1 拇指旋轉] X={x:F1}, Y={y:F1}, Z={z:F1} → output={leftFingerValues[0]:F2}");
                }
                
                // 拇指末端 Debug (F2 - 伸縮)
                if (leftThumbProximal != null)
                {
                    Vector3 euler = leftThumbProximal.localEulerAngles;
                    float x = euler.x > 180f ? euler.x - 360f : euler.x;
                    float y = euler.y > 180f ? euler.y - 360f : euler.y;
                    float z = euler.z > 180f ? euler.z - 360f : euler.z;
                    Debug.Log($"[F2 拇指伸縮] X={x:F1}, Y={y:F1}, Z={z:F1} → output={leftFingerValues[1]:F2}");
                }
                
                // 食指 Debug (F3)
                if (leftIndexProximal != null)
                {
                    Vector3 euler = leftIndexProximal.localEulerAngles;
                    float x = euler.x > 180f ? euler.x - 360f : euler.x;
                    float y = euler.y > 180f ? euler.y - 360f : euler.y;
                    float z = euler.z > 180f ? euler.z - 360f : euler.z;
                    Debug.Log($"[F3 食指] X={x:F1}, Y={y:F1}, Z={z:F1} → output={leftFingerValues[2]:F2}");
                }
                
                // 中指 Debug (F4)
                if (leftMiddleProximal != null)
                {
                    Vector3 euler = leftMiddleProximal.localEulerAngles;
                    float x = euler.x > 180f ? euler.x - 360f : euler.x;
                    float y = euler.y > 180f ? euler.y - 360f : euler.y;
                    float z = euler.z > 180f ? euler.z - 360f : euler.z;
                    Debug.Log($"[F4 中指] X={x:F1}, Y={y:F1}, Z={z:F1} → output={leftFingerValues[3]:F2}");
                }
                
                // 無名指 Debug (F5)
                if (leftRingProximal != null)
                {
                    Vector3 euler = leftRingProximal.localEulerAngles;
                    float x = euler.x > 180f ? euler.x - 360f : euler.x;
                    float y = euler.y > 180f ? euler.y - 360f : euler.y;
                    float z = euler.z > 180f ? euler.z - 360f : euler.z;
                    Debug.Log($"[F5 無名指] X={x:F1}, Y={y:F1}, Z={z:F1} → output={leftFingerValues[4]:F2}");
                }
                
                // 尾指 Debug (F6)
                if (leftLittleProximal != null)
                {
                    Vector3 euler = leftLittleProximal.localEulerAngles;
                    float x = euler.x > 180f ? euler.x - 360f : euler.x;
                    float y = euler.y > 180f ? euler.y - 360f : euler.y;
                    float z = euler.z > 180f ? euler.z - 360f : euler.z;
                    Debug.Log($"[F6 尾指] X={x:F1}, Y={y:F1}, Z={z:F1} → output={leftFingerValues[5]:F2}");
                }
                
                Debug.Log("==================");
            }
        }
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

        // 讀取當前角度
        float indexZ = GetCurrentAngle(leftIndexProximal);
        float middleZ = GetCurrentAngle(leftMiddleProximal);
        float ringZ = GetCurrentAngle(leftRingProximal);
        float littleZ = GetCurrentAngle(leftLittleProximal);
        
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

        // 讀取當前角度
        float indexZ = GetCurrentAngle(leftIndexProximal);
        float middleZ = GetCurrentAngle(leftMiddleProximal);
        float ringZ = GetCurrentAngle(leftRingProximal);
        float littleZ = GetCurrentAngle(leftLittleProximal);
        
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

        Debug.Log("=== 當前手指角度 ===");
        Debug.Log($"[食指] Z = {GetCurrentAngle(leftIndexProximal):F1}°");
        Debug.Log($"[中指] Z = {GetCurrentAngle(leftMiddleProximal):F1}°");
        Debug.Log($"[無名指] Z = {GetCurrentAngle(leftRingProximal):F1}°");
        Debug.Log($"[尾指] Z = {GetCurrentAngle(leftLittleProximal):F1}°");
        
        Vector3 thumbEuler = leftThumbProximal.localEulerAngles;
        float thumbY = thumbEuler.y > 180f ? thumbEuler.y - 360f : thumbEuler.y;
        float thumbZ = thumbEuler.z > 180f ? thumbEuler.z - 360f : thumbEuler.z;
        Debug.Log($"[拇指] Y = {thumbY:F1}°, Z = {thumbZ:F1}°");
        Debug.Log("==================");
    }

    /// <summary>
    /// 取得指定骨骼當前的 Z 軸角度（-180~180）
    /// </summary>
    private float GetCurrentAngle(Transform bone)
    {
        if (bone == null) return 0f;
        
        Vector3 euler = bone.localEulerAngles;
        float angle = bendAxis == 0 ? euler.x : (bendAxis == 1 ? euler.y : euler.z);
        
        // 轉換為 -180~180
        if (angle > 180f) angle -= 360f;
        
        return angle;
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
