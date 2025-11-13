using System;
using UnityEngine;

/// <summary>
/// OpenArm Retarget with IK support - 完全修復版 v3
/// ✅ 修正座標系轉換問題
/// ✅ 修正編輯器模式下的初始化
/// ✅ 加強調試輸出
/// </summary>
public class OpenArmRetargetIK : MonoBehaviour
{
    public enum ControlMode
    {
        SingleJoint,
        IK,
        Hybrid
    }

    public enum Axis { X, Y, Z }

    [Serializable]
    public class JointMap
    {
        [Header("Target Joint")]
        public string nameHint;
        public ArticulationBody joint;

        [Header("Source (Humanoid bone)")]
        public Transform source;
        public Axis sourceAxis = Axis.X;
        public bool useNeutralCalibration = true;
        public Vector3 neutralEulerLocal;

        [Header("Mapping")]
        public float scale = 1f;
        public float offsetDeg = 0f;
        public float minDeg = -180f;
        public float maxDeg = 180f;

        [Header("Stability")]
        public float deadZone = 2f;
        public float hysteresis = 1.5f;
        public float smoothAlpha = 0.25f;
        public float rateLimitDegPerSec = 180f;
        public float softLimitMargin = 8f;

        [Header("Drive")]
        public float stiffness = 4000f;
        public float damping = 300f;
        public float forceLimit = 10000f;

        // 內部狀態
        float _filteredDeg;
        float _lastCmdDeg;
        bool _inDeadHold;
        float _deadCenter;

        public bool isLocked = false;
        public float lockedTarget = 0f;

        public void CalibrateNeutral()
        {
            if (source == null) return;
            neutralEulerLocal = source.localEulerAngles;
        }

        public float ReadSourceAngleDegRaw()
        {
            if (source == null) return 0f;
            var e = source.localEulerAngles;

            float sx = Mathf.DeltaAngle(0f, e.x);
            float sy = Mathf.DeltaAngle(0f, e.y);
            float sz = Mathf.DeltaAngle(0f, e.z);

            float raw = 0f;
            switch (sourceAxis)
            {
                case Axis.X: raw = sx; break;
                case Axis.Y: raw = sy; break;
                default: raw = sz; break;
            }

            if (useNeutralCalibration)
            {
                var ne = neutralEulerLocal;
                float nx = Mathf.DeltaAngle(0f, ne.x);
                float ny = Mathf.DeltaAngle(0f, ne.y);
                float nz = Mathf.DeltaAngle(0f, ne.z);
                float nAxis = sourceAxis == Axis.X ? nx : (sourceAxis == Axis.Y ? ny : nz);
                raw = Mathf.DeltaAngle(nAxis, raw);
            }

            return raw;
        }

        public void Apply(float deltaTime)
        {
            if (joint == null) return;

            var drive = joint.xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;

            if (isLocked)
            {
                drive.target = lockedTarget;
                joint.xDrive = drive;
                _lastCmdDeg = lockedTarget;
                return;
            }

            float src = ReadSourceAngleDegRaw();
            float mapped = offsetDeg + scale * src;

            if (_inDeadHold)
            {
                if (Mathf.Abs(mapped - _deadCenter) > (deadZone + hysteresis))
                    _inDeadHold = false;
                else
                    mapped = _deadCenter;
            }
            else
            {
                if (Mathf.Abs(mapped - _deadCenter) < deadZone)
                {
                    _inDeadHold = true;
                    mapped = _deadCenter;
                }
            }

            _filteredDeg = Mathf.Lerp(_filteredDeg, mapped, Mathf.Clamp01(smoothAlpha));

            float lowerSoft = minDeg + softLimitMargin;
            float upperSoft = maxDeg - softLimitMargin;
            float targetDeg = Mathf.Clamp(_filteredDeg, minDeg, maxDeg);

            if (targetDeg > upperSoft && targetDeg < maxDeg)
            {
                float t = Mathf.InverseLerp(upperSoft, maxDeg, targetDeg);
                targetDeg = Mathf.Lerp(targetDeg, upperSoft, t);
            }
            else if (targetDeg < lowerSoft && targetDeg > minDeg)
            {
                float t = Mathf.InverseLerp(lowerSoft, minDeg, targetDeg);
                targetDeg = Mathf.Lerp(targetDeg, lowerSoft, t);
            }

            if (rateLimitDegPerSec > 0f && deltaTime > 0f)
            {
                float maxStep = rateLimitDegPerSec * deltaTime;
                float step = Mathf.Clamp(targetDeg - _lastCmdDeg, -maxStep, +maxStep);
                targetDeg = _lastCmdDeg + step;
            }

            drive.target = targetDeg;
            joint.xDrive = drive;

            _lastCmdDeg = targetDeg;
        }

        public void SetTargetDirect(float angleDeg)
        {
            if (joint == null) return;

            var drive = joint.xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;
            drive.target = Mathf.Clamp(angleDeg, minDeg, maxDeg);
            joint.xDrive = drive;

            _lastCmdDeg = angleDeg;
        }
    }

    [Serializable]
    public class ArmIKConfig
    {
        [Header("IK 追蹤目標")]
        public Transform shoulderReference;
        public Transform wristTarget;
        public Transform elbowHint;

        [Header("⚠️ 重要：機械臂基座參考點")]
        [Tooltip("機械臂的實際基座Transform（通常是最外層的父物件，如 'OpenArm_Base' 或 'Robot'）")]
        public Transform robotBaseOverride;

        [Header("末端執行器偏移（局部座標）")]
        [Tooltip("從手腕關節到實際抓取點的偏移")]
        public Vector3 endEffectorOffset = Vector3.zero;

        [Header("✅ 縮放設定 - 請先執行校準！")]
        [Tooltip("統一縮放因子（由 OpenArmIKAutoScaler 自動設定）")]
        public float uniformScale = 1.0f;

        [Header("IK 平滑")]
        [Range(0f, 1f)]
        [Tooltip("值越大追蹤越快，0=完全不動，1=立即追蹤")]
        public float positionSmooth = 0.3f;

        [Range(0f, 1f)]
        public float rotationSmooth = 0.3f;

        [Header("IK 約束（機械臂局部座標系）")]
        public bool usePositionConstraint = true;
        [Tooltip("相對於機械臂基座的最小位置（局部座標）")]
        public Vector3 constraintMin = new Vector3(-0.5f, -0.3f, 0.1f);
        [Tooltip("相對於機械臂基座的最大位置（局部座標）")]
        public Vector3 constraintMax = new Vector3(0.5f, 0.5f, 0.8f);

        // 內部平滑狀態
        [HideInInspector] public Vector3 smoothedPosition;
        [HideInInspector] public Quaternion smoothedRotation = Quaternion.identity;
        [HideInInspector] public bool isInitialized = false;

        // ✅ 診斷資訊
        [HideInInspector] public Vector3 lastHumanArmVectorWorld;
        [HideInInspector] public Vector3 lastHumanArmVectorLocal;
        [HideInInspector] public Vector3 lastScaledArmVectorLocal;
        [HideInInspector] public Vector3 lastConstrainedLocal;
    }

    [Header("Control Mode")]
    public ControlMode controlMode = ControlMode.SingleJoint;

    [Header("Left arm")]
    public JointMap[] leftJoints = new JointMap[7];
    public ArmIKConfig leftIK = new ArmIKConfig();

    [Header("Right arm")]
    public JointMap[] rightJoints = new JointMap[7];
    public ArmIKConfig rightIK = new ArmIKConfig();

    [Header("IK Solver")]
    public OpenArmIK leftIKSolver;
    public OpenArmIK rightIKSolver;

    [Header("Global")]
    public bool autoCalibrateOnStart = true;
    public KeyCode switchModeKey = KeyCode.Tab;

    [Header("⚠️ 校準提醒")]
    [Tooltip("如果為true，會在Play模式開始時檢查是否已校準")]
    public bool warnIfNotCalibrated = true;

    [Header("Debug")]
    public bool showDebugInfo = false;
    public bool drawDebugGizmos = true;
    [Tooltip("顯示詳細的座標轉換過程")]
    public bool showDetailedDebug = false;

    // ✅ 新增：追蹤是否已校準
    private bool _hasBeenCalibrated = false;

    void Start()
    {
        // ✅ 先確保IK求解器已初始化連桿偏移
        if (leftIKSolver != null)
        {
            leftIKSolver.SendMessage("InitializeLinkOffsets", SendMessageOptions.DontRequireReceiver);
        }
        if (rightIKSolver != null)
        {
            rightIKSolver.SendMessage("InitializeLinkOffsets", SendMessageOptions.DontRequireReceiver);
        }

        if (autoCalibrateOnStart)
        {
            CalibrateAll();
        }

        // ✅ 檢查是否需要校準縮放
        CheckCalibrationStatus();

        InitializeIKConfig(leftIK, leftIKSolver);
        InitializeIKConfig(rightIK, rightIKSolver);

        Debug.Log($"🤖 OpenArmRetargetIK 啟動 | 模式: {controlMode}");
    }

    void Update()
    {
        if (Input.GetKeyDown(switchModeKey))
        {
            SwitchMode();
        }
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        switch (controlMode)
        {
            case ControlMode.SingleJoint:
                ApplySingleJointMode(dt);
                break;

            case ControlMode.IK:
                ApplyIKMode(dt);
                break;

            case ControlMode.Hybrid:
                ApplyHybridMode(dt);
                break;
        }
    }

    #region 校準檢查

    void CheckCalibrationStatus()
    {
        if (!warnIfNotCalibrated) return;

        bool needsCalibration = false;
        string warnings = "⚠️ OpenArmRetargetIK 校準檢查:\n";

        // 檢查左手
        if (leftIK.wristTarget != null && leftIKSolver != null)
        {
            if (Mathf.Approximately(leftIK.uniformScale, 1.0f))
            {
                warnings += "❌ 左手 uniformScale = 1.0 (可能未校準)\n";
                needsCalibration = true;
            }
            if (leftIK.robotBaseOverride == null)
            {
                warnings += "⚠️ 左手 robotBaseOverride 未設定（將自動使用 joints[0].parent）\n";
            }
        }

        // 檢查右手
        if (rightIK.wristTarget != null && rightIKSolver != null)
        {
            if (Mathf.Approximately(rightIK.uniformScale, 1.0f))
            {
                warnings += "❌ 右手 uniformScale = 1.0 (可能未校準)\n";
                needsCalibration = true;
            }
            if (rightIK.robotBaseOverride == null)
            {
                warnings += "⚠️ 右手 robotBaseOverride 未設定（將自動使用 joints[0].parent）\n";
            }
        }

        if (needsCalibration)
        {
            warnings += "\n💡 建議：請在場景中添加 OpenArmIKAutoScaler 並執行 'Calibrate Now'";
            Debug.LogWarning(warnings);
        }
        else
        {
            Debug.Log("✅ OpenArmRetargetIK: 校準狀態正常");
        }
    }

    #endregion

    #region 控制模式實作

    void ApplySingleJointMode(float deltaTime)
    {
        if (leftJoints != null)
        {
            foreach (var j in leftJoints)
                j?.Apply(deltaTime);
        }

        if (rightJoints != null)
        {
            foreach (var j in rightJoints)
                j?.Apply(deltaTime);
        }
    }

    void ApplyIKMode(float deltaTime)
    {
        if (leftIKSolver != null && leftIK.wristTarget != null)
        {
            Vector3 targetPos = GetSmoothedIKTarget(leftIK, leftIKSolver, deltaTime);

            if (leftIKSolver.SolveIK(targetPos, out float[] angles))
            {
                ApplyIKAngles(leftJoints, angles);

                if (showDebugInfo && Time.frameCount % 60 == 0)
                {
                    float error = Vector3.Distance(leftIKSolver.GetEndEffectorPosition(), targetPos);
                    Debug.Log($"✅ 左手 IK | 誤差: {error * 1000f:F1}mm");
                }
            }
        }

        if (rightIKSolver != null && rightIK.wristTarget != null)
        {
            Vector3 targetPos = GetSmoothedIKTarget(rightIK, rightIKSolver, deltaTime);

            if (rightIKSolver.SolveIK(targetPos, out float[] angles))
            {
                ApplyIKAngles(rightJoints, angles);

                if (showDebugInfo && Time.frameCount % 60 == 0)
                {
                    float error = Vector3.Distance(rightIKSolver.GetEndEffectorPosition(), targetPos);
                    Debug.Log($"✅ 右手 IK | 誤差: {error * 1000f:F1}mm");
                }
            }
        }
    }

    void ApplyHybridMode(float deltaTime)
    {
        if (leftIKSolver != null && leftIK.wristTarget != null)
        {
            Vector3 targetPos = GetSmoothedIKTarget(leftIK, leftIKSolver, deltaTime);

            if (leftIKSolver.SolveIK(targetPos, out float[] angles))
            {
                for (int i = 0; i < 4 && i < leftJoints.Length; i++)
                {
                    leftJoints[i]?.SetTargetDirect(angles[i]);
                }

                for (int i = 4; i < leftJoints.Length; i++)
                {
                    leftJoints[i]?.Apply(deltaTime);
                }
            }
        }

        if (rightIKSolver != null && rightIK.wristTarget != null)
        {
            Vector3 targetPos = GetSmoothedIKTarget(rightIK, rightIKSolver, deltaTime);

            if (rightIKSolver.SolveIK(targetPos, out float[] angles))
            {
                for (int i = 0; i < 4 && i < rightJoints.Length; i++)
                {
                    rightJoints[i]?.SetTargetDirect(angles[i]);
                }

                for (int i = 4; i < rightJoints.Length; i++)
                {
                    rightJoints[i]?.Apply(deltaTime);
                }
            }
        }
    }

    #endregion

    #region IK 輔助方法（✅ 完全修復版）

    void InitializeIKConfig(ArmIKConfig config, OpenArmIK ikSolver)
    {
        if (config.wristTarget != null && ikSolver != null)
        {
            // ✅ 確保在Play模式下才初始化位置
            if (Application.isPlaying)
            {
                config.smoothedPosition = GetSmoothedIKTarget(config, ikSolver, 0f);
                config.isInitialized = true;
            }
        }
    }

    /// <summary>
    /// ✅ 獲取機械臂基座Transform（優先使用override，否則自動查找）
    /// </summary>
    Transform GetRobotBase(ArmIKConfig config, OpenArmIK ikSolver)
    {
        // 1. 優先使用手動設定的基座
        if (config.robotBaseOverride != null)
            return config.robotBaseOverride;

        // 2. 自動查找：使用第一個關節的父物件
        if (ikSolver.joints != null && ikSolver.joints.Length > 0 && ikSolver.joints[0].joint != null)
        {
            Transform parent = ikSolver.joints[0].joint.transform.parent;
            if (parent != null)
                return parent;

            // 如果沒有父物件，使用關節本身
            return ikSolver.joints[0].joint.transform;
        }

        Debug.LogWarning("⚠️ 無法找到機械臂基座！請設定 robotBaseOverride");
        return null;
    }

    /// <summary>
    /// ✅ 獲取平滑後的 IK 目標位置（完全修復版 v3）
    /// </summary>
    Vector3 GetSmoothedIKTarget(ArmIKConfig config, OpenArmIK ikSolver, float deltaTime)
    {
        if (config.wristTarget == null || ikSolver == null)
            return config.smoothedPosition;

        // 1. 獲取機械臂基座
        Transform robotBase = GetRobotBase(config, ikSolver);
        if (robotBase == null)
            return config.smoothedPosition;

        // 2. 獲取人體肩膀和手腕位置（世界座標）
        Vector3 humanShoulderWorld = config.shoulderReference != null
            ? config.shoulderReference.position
            : robotBase.position; // 如果沒有肩膀參考，使用機械臂基座

        Vector3 humanWristWorld = config.wristTarget.position;

        // 3. 計算人體手臂向量（世界座標）
        Vector3 humanArmVectorWorld = humanWristWorld - humanShoulderWorld;
        config.lastHumanArmVectorWorld = humanArmVectorWorld;

        // 4. ✅ 關鍵修正：將人體手臂向量轉換到機械臂基座的局部座標系
        Vector3 humanArmVectorLocal = robotBase.InverseTransformDirection(humanArmVectorWorld);
        config.lastHumanArmVectorLocal = humanArmVectorLocal;

        // 5. 套用統一縮放（在局部座標系中）
        Vector3 scaledArmVectorLocal = humanArmVectorLocal * config.uniformScale;
        config.lastScaledArmVectorLocal = scaledArmVectorLocal;

        // 6. 套用末端執行器偏移（在局部座標系中）
        Vector3 localTarget = scaledArmVectorLocal + config.endEffectorOffset;

        // 7. 約束檢查（在局部座標系中）
        if (config.usePositionConstraint)
        {
            localTarget.x = Mathf.Clamp(localTarget.x, config.constraintMin.x, config.constraintMax.x);
            localTarget.y = Mathf.Clamp(localTarget.y, config.constraintMin.y, config.constraintMax.y);
            localTarget.z = Mathf.Clamp(localTarget.z, config.constraintMin.z, config.constraintMax.z);
        }
        config.lastConstrainedLocal = localTarget;

        // 8. ✅ 轉回世界座標（相對於機械臂基座）
        Vector3 finalTargetWorldPos = robotBase.TransformPoint(localTarget);

        // 9. 平滑處理
        if (!config.isInitialized || deltaTime <= 0f || !Application.isPlaying)
        {
            config.smoothedPosition = finalTargetWorldPos;
            config.isInitialized = true;
        }
        else
        {
            float smoothFactor = Mathf.Clamp01(config.positionSmooth);
            config.smoothedPosition = Vector3.Lerp(
                config.smoothedPosition,
                finalTargetWorldPos,
                smoothFactor
            );
        }

        // 🔍 詳細調試輸出
        if (showDetailedDebug && (Time.frameCount % 60 == 0 || !Application.isPlaying))
        {
            Debug.Log($"=== IK 目標計算詳細資訊 ===\n" +
                     $"機械臂基座: {robotBase.name} @ {robotBase.position}\n" +
                     $"人體肩膀(世界): {humanShoulderWorld}\n" +
                     $"人體手腕(世界): {humanWristWorld}\n" +
                     $"人體臂向量(世界): {humanArmVectorWorld} (長度: {humanArmVectorWorld.magnitude:F3}m)\n" +
                     $"人體臂向量(局部): {humanArmVectorLocal}\n" +
                     $"uniformScale: {config.uniformScale:F3}\n" +
                     $"縮放後(局部): {scaledArmVectorLocal} (長度: {scaledArmVectorLocal.magnitude:F3}m)\n" +
                     $"約束後(局部): {localTarget}\n" +
                     $"最終目標(世界): {finalTargetWorldPos}");
        }

        return config.smoothedPosition;
    }

    void ApplyIKAngles(JointMap[] joints, float[] angles)
    {
        if (joints == null || angles == null) return;

        int count = Mathf.Min(joints.Length, angles.Length);
        for (int i = 0; i < count; i++)
        {
            joints[i]?.SetTargetDirect(angles[i]);
        }
    }

    #endregion

    #region 公共方法

    [ContextMenu("校準所有關節")]
    public void CalibrateAll()
    {
        if (leftJoints != null)
            foreach (var j in leftJoints)
                j?.CalibrateNeutral();

        if (rightJoints != null)
            foreach (var j in rightJoints)
                j?.CalibrateNeutral();

        _hasBeenCalibrated = true;
        Debug.Log("✅ OpenArmRetargetIK: 關節校準完成");
    }

    [ContextMenu("切換控制模式")]
    public void SwitchMode()
    {
        controlMode = (ControlMode)(((int)controlMode + 1) % 3);
        Debug.Log($"🔄 切換到模式: {controlMode}");
    }

    public void SetControlMode(ControlMode mode)
    {
        controlMode = mode;
        Debug.Log($"🔄 設定模式: {controlMode}");
    }

    [ContextMenu("診斷 IK 設定")]
    public void DiagnoseIKSettings()
    {
        Debug.Log("=== OpenArmRetargetIK 診斷報告 ===");

        // 左手診斷
        if (leftIK.wristTarget != null && leftIKSolver != null)
        {
            Transform robotBase = GetRobotBase(leftIK, leftIKSolver);
            Vector3 testTarget = GetSmoothedIKTarget(leftIK, leftIKSolver, 0f);
            float currentError = leftIKSolver.GetEndEffectorPosition() != null
                ? Vector3.Distance(leftIKSolver.GetEndEffectorPosition(), testTarget)
                : 0f;

            Debug.Log($"【左手】\n" +
                     $"  機械臂基座: {(robotBase != null ? robotBase.name : "未找到")}\n" +
                     $"  uniformScale: {leftIK.uniformScale:F3}\n" +
                     $"  人體臂長: {leftIK.lastHumanArmVectorWorld.magnitude:F3}m\n" +
                     $"  縮放後臂長: {leftIK.lastScaledArmVectorLocal.magnitude:F3}m\n" +
                     $"  當前誤差: {currentError * 1000f:F1}mm\n" +
                     $"  約束範圍: {leftIK.constraintMin} ~ {leftIK.constraintMax}");
        }

        // 右手診斷
        if (rightIK.wristTarget != null && rightIKSolver != null)
        {
            Transform robotBase = GetRobotBase(rightIK, rightIKSolver);
            Vector3 testTarget = GetSmoothedIKTarget(rightIK, rightIKSolver, 0f);
            float currentError = rightIKSolver.GetEndEffectorPosition() != null
                ? Vector3.Distance(rightIKSolver.GetEndEffectorPosition(), testTarget)
                : 0f;

            Debug.Log($"【右手】\n" +
                     $"  機械臂基座: {(robotBase != null ? robotBase.name : "未找到")}\n" +
                     $"  uniformScale: {rightIK.uniformScale:F3}\n" +
                     $"  人體臂長: {rightIK.lastHumanArmVectorWorld.magnitude:F3}m\n" +
                     $"  縮放後臂長: {rightIK.lastScaledArmVectorLocal.magnitude:F3}m\n" +
                     $"  當前誤差: {currentError * 1000f:F1}mm\n" +
                     $"  約束範圍: {rightIK.constraintMin} ~ {rightIK.constraintMax}");
        }
    }

    #endregion

    #region 調試視覺化

    void OnDrawGizmos()
    {
        if (!drawDebugGizmos) return;

        if (leftIK.wristTarget != null && leftIKSolver != null)
        {
            DrawIKDebug(leftIK, leftIKSolver, Color.blue, "L");
        }

        if (rightIK.wristTarget != null && rightIKSolver != null)
        {
            DrawIKDebug(rightIK, rightIKSolver, Color.red, "R");
        }
    }

    void DrawIKDebug(ArmIKConfig config, OpenArmIK ikSolver, Color color, string label)
    {
        // 繪製人體手腕目標（原始）
        Gizmos.color = color;
        Gizmos.DrawWireSphere(config.wristTarget.position, 0.03f);

        // 繪製平滑後的IK目標
        Vector3 smoothedTarget = GetSmoothedIKTarget(config, ikSolver, 0f);
        Gizmos.color = Color.Lerp(color, Color.white, 0.5f);
        Gizmos.DrawWireSphere(smoothedTarget, 0.025f);

        // 繪製從原始到平滑的連線
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(config.wristTarget.position, smoothedTarget);

        // 繪製機械臂末端執行器
        Vector3 endPos = ikSolver.GetEndEffectorPosition();
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(endPos, 0.02f);

        // 繪製從IK目標到末端執行器的誤差
        Gizmos.color = Color.magenta;
        Gizmos.DrawLine(smoothedTarget, endPos);

        // 繪製機械臂基座
        Transform robotBase = GetRobotBase(config, ikSolver);
        if (robotBase != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireCube(robotBase.position, Vector3.one * 0.05f);

            // 繪製基座座標軸
            Gizmos.color = Color.red;
            Gizmos.DrawRay(robotBase.position, robotBase.right * 0.1f);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(robotBase.position, robotBase.up * 0.1f);
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(robotBase.position, robotBase.forward * 0.1f);
        }

#if UNITY_EDITOR
        float distance = Vector3.Distance(smoothedTarget, endPos);
        UnityEditor.Handles.Label(
            (smoothedTarget + endPos) * 0.5f,
            $"[{label}] IK誤差: {distance * 1000f:F1}mm\nScale:{config.uniformScale:F2}"
        );
        
        // 顯示基座名稱
        if (robotBase != null)
        {
            UnityEditor.Handles.Label(
                robotBase.position + Vector3.up * 0.1f,
                $"基座: {robotBase.name}"
            );
        }
#endif
    }

    void OnGUI()
    {
        if (!showDebugInfo) return;

        GUILayout.BeginArea(new Rect(10, 10, 400, 300));
        GUILayout.Label("OpenArm Retarget IK (修復版 v3)", GUI.skin.box);

        GUILayout.Label($"控制模式: {controlMode}");
        GUILayout.Label($"切換鍵: {switchModeKey}");

        if (GUILayout.Button($"切換模式 (當前: {controlMode})"))
        {
            SwitchMode();
        }

        if (GUILayout.Button("校準關節"))
        {
            CalibrateAll();
        }

        if (GUILayout.Button("診斷 IK 設定"))
        {
            DiagnoseIKSettings();
        }

        GUILayout.Space(10);

        if (controlMode == ControlMode.IK || controlMode == ControlMode.Hybrid)
        {
            if (leftIKSolver != null && leftIK.wristTarget != null)
            {
                Vector3 target = GetSmoothedIKTarget(leftIK, leftIKSolver, 0f);
                float error = Vector3.Distance(leftIKSolver.GetEndEffectorPosition(), target) * 1000f;

                GUILayout.Label($"【左手】");
                GUILayout.Label($"  誤差: {error:F1}mm");
                GUILayout.Label($"  Scale: {leftIK.uniformScale:F3}");
                GUILayout.Label($"  臂長: {leftIK.lastHumanArmVectorWorld.magnitude:F3}m");
            }

            GUILayout.Space(5);

            if (rightIKSolver != null && rightIK.wristTarget != null)
            {
                Vector3 target = GetSmoothedIKTarget(rightIK, rightIKSolver, 0f);
                float error = Vector3.Distance(rightIKSolver.GetEndEffectorPosition(), target) * 1000f;

                GUILayout.Label($"【右手】");
                GUILayout.Label($"  誤差: {error:F1}mm");
                GUILayout.Label($"  Scale: {rightIK.uniformScale:F3}");
                GUILayout.Label($"  臂長: {rightIK.lastHumanArmVectorWorld.magnitude:F3}m");
            }
        }

        GUILayout.EndArea();
    }

    #endregion
}