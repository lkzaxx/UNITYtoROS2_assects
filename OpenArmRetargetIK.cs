using System;
using UnityEngine;

/// <summary>
/// OpenArm Retarget with IK support - 整合逆運動學的手臂重定向
/// 支援兩種模式：
/// 1. 單關節映射（原始模式）- 每個關節獨立映射
/// 2. IK 模式 - 從末端位置計算整體關節角度
/// </summary>
public class OpenArmRetargetIK : MonoBehaviour
{
    public enum ControlMode
    {
        SingleJoint,    // 單關節映射（原始模式）
        IK,             // 逆運動學模式
        Hybrid          // 混合模式（IK + 單關節微調）
    }

    public enum Axis { X, Y, Z }

    [Serializable]
    public class JointMap
    {
        [Header("Target Joint")]
        public string nameHint;
        public ArticulationBody joint;

        [Header("Source (Humanoid bone)")]
        public Transform source;                // 來源骨骼（上臂/前臂/手腕）
        public Axis sourceAxis = Axis.X;        // 取該骨骼的哪一個 local Euler 軸
        public bool useNeutralCalibration = true;
        public Vector3 neutralEulerLocal;       // 校準時紀錄的 localEulerAngles

        [Header("Mapping")]
        public float scale = 1f;                // 角度比例（可用 -1 反向）
        public float offsetDeg = 0f;            // 角度偏移（度）
        public float minDeg = -180f;            // 目標下限
        public float maxDeg = 180f;             // 目標上限

        [Header("Stability")]
        public float deadZone = 2f;             // 死區：|角度| < deadZone 視為 0
        public float hysteresis = 1.5f;         // 遷就帶：一旦進入死區，要超過此值才解除
        public float smoothAlpha = 0.25f;       // 低通濾波（0~1，越大越跟手）
        public float rateLimitDegPerSec = 180f; // 角速度上限（deg/s）
        public float softLimitMargin = 8f;      // 靠近上下限時提前降速的緩衝（度）

        [Header("Drive")]
        public float stiffness = 4000f;
        public float damping = 300f;
        public float forceLimit = 10000f;

        // 內部狀態
        float _filteredDeg;        // 濾波後角度
        float _lastCmdDeg;         // 上一幀送給驅動器的角度
        bool  _inDeadHold;         // 是否位於死區並被「鎖住」
        float _deadCenter;         // 死區中心（通常為 0）
        
        // 校準鎖定狀態
        public bool isLocked = false;      // 是否被鎖定在目標角度
        public float lockedTarget = 0f;    // 鎖定的目標角度

        public void CalibrateNeutral()
        {
            if (source == null) return;
            neutralEulerLocal = source.localEulerAngles;
        }

        public float ReadSourceAngleDegRaw()
        {
            if (source == null) return 0f;
            var e = source.localEulerAngles;

            // 轉成 -180..180，避免 0/360 跳變
            float sx = Mathf.DeltaAngle(0f, e.x);
            float sy = Mathf.DeltaAngle(0f, e.y);
            float sz = Mathf.DeltaAngle(0f, e.z);

            float raw = 0f;
            switch (sourceAxis)
            {
                case Axis.X: raw = sx; break;
                case Axis.Y: raw = sy; break;
                default:     raw = sz; break;
            }

            if (useNeutralCalibration)
            {
                var ne = neutralEulerLocal;
                float nx = Mathf.DeltaAngle(0f, ne.x);
                float ny = Mathf.DeltaAngle(0f, ne.y);
                float nz = Mathf.DeltaAngle(0f, ne.z);
                float nAxis = sourceAxis == Axis.X ? nx : (sourceAxis == Axis.Y ? ny : nz);
                raw = Mathf.DeltaAngle(nAxis, raw); // 以校準姿勢為 0 度
            }

            return raw;
        }

        public void Apply(float deltaTime)
        {
            if (joint == null) return;

            // 驅動器參數
            var drive = joint.xDrive;
            drive.stiffness  = stiffness;
            drive.damping    = damping;
            drive.forceLimit = forceLimit;

            // 如果被鎖定，直接使用鎖定值並跳過所有計算
            if (isLocked)
            {
                drive.target = lockedTarget;
                joint.xDrive = drive;
                _lastCmdDeg = lockedTarget;
                return;
            }

            // 1) 讀取角度 → 映射
            float src = ReadSourceAngleDegRaw();
            float mapped = offsetDeg + scale * src;

            // 2) 死區 + 遷就帶（防飄 & 手停就停）
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

            // 3) 低通濾波（EMA）
            _filteredDeg = Mathf.Lerp(_filteredDeg, mapped, Mathf.Clamp01(smoothAlpha));

            // 4) 軟上限（接近邊界時提前降速）
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

            // 5) 限速（deg/s）
            if (rateLimitDegPerSec > 0f && deltaTime > 0f)
            {
                float maxStep = rateLimitDegPerSec * deltaTime;
                float step = Mathf.Clamp(targetDeg - _lastCmdDeg, -maxStep, +maxStep);
                targetDeg = _lastCmdDeg + step;
            }

            // 6) 寫入目標
            drive.target = targetDeg;
            joint.xDrive = drive;

            _lastCmdDeg = targetDeg;
        }

        /// <summary>
        /// 直接設定關節目標角度（用於 IK 模式）
        /// </summary>
        public void SetTargetDirect(float angleDeg)
        {
            if (joint == null) return;

            var drive = joint.xDrive;
            drive.stiffness  = stiffness;
            drive.damping    = damping;
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
        public Transform shoulderReference;     // 肩膀參考點（用於相對座標計算）
        public Transform wristTarget;           // 手腕目標位置（來自人體）
        public Transform elbowHint;             // 手肘提示（可選，用於控制手肘方向）

        [Header("末端執行器偏移")]
        public Vector3 endEffectorOffset = Vector3.zero;  // 末端執行器相對手腕的偏移
        public Vector3 positionScale = Vector3.one;       // 位置縮放（用於調整人體與機械臂的尺寸差異）

        [Header("IK 平滑")]
        public float positionSmooth = 0.3f;     // 位置平滑（0~1）
        public float rotationSmooth = 0.3f;     // 旋轉平滑（0~1）

        [Header("IK 約束 (相對於機械臂基座)")]
        public bool usePositionConstraint = true;
        public Vector3 constraintMin = new Vector3(-0.5f, -0.3f, 0.1f);
        public Vector3 constraintMax = new Vector3(0.5f, 0.5f, 0.8f);

        // 內部平滑狀態
        [HideInInspector] public Vector3 smoothedPosition;
        [HideInInspector] public Quaternion smoothedRotation = Quaternion.identity;
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

    [Header("Debug")]
    public bool showDebugInfo = false;

    void Start()
    {
        if (autoCalibrateOnStart)
        {
            CalibrateAll();
        }

        // 初始化平滑狀態（使用相對座標計算）
        if (leftIK.wristTarget != null && leftIKSolver != null)
        {
            leftIK.smoothedPosition = GetSmoothedIKTarget(leftIK, leftIKSolver, 0f);
        }
        if (rightIK.wristTarget != null && rightIKSolver != null)
        {
            rightIK.smoothedPosition = GetSmoothedIKTarget(rightIK, rightIKSolver, 0f);
        }

        Debug.Log($"🤖 OpenArmRetargetIK 啟動 | 模式: {controlMode}");
    }

    void Update()
    {
        // 切換控制模式
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

    #region 控制模式實作

    /// <summary>
    /// 單關節映射模式（原始模式）
    /// </summary>
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

    /// <summary>
    /// IK 模式 - 從手腕位置計算所有關節角度
    /// </summary>
    void ApplyIKMode(float deltaTime)
    {
        // 左手
        if (leftIKSolver != null && leftIK.wristTarget != null)
        {
            Vector3 targetPos = GetSmoothedIKTarget(leftIK, leftIKSolver, deltaTime);
            
            if (leftIKSolver.SolveIK(targetPos, out float[] angles))
            {
                ApplyIKAngles(leftJoints, angles);
                
                if (showDebugInfo && Time.frameCount % 30 == 0)
                    Debug.Log($"✅ 左手 IK 成功 | 目標: {targetPos}");
            }
        }

        // 右手
        if (rightIKSolver != null && rightIK.wristTarget != null)
        {
            Vector3 targetPos = GetSmoothedIKTarget(rightIK, rightIKSolver, deltaTime);
            
            if (rightIKSolver.SolveIK(targetPos, out float[] angles))
            {
                ApplyIKAngles(rightJoints, angles);
                
                if (showDebugInfo && Time.frameCount % 30 == 0)
                    Debug.Log($"✅ 右手 IK 成功 | 目標: {targetPos}");
            }
        }
    }

    /// <summary>
    /// 混合模式 - IK 處理主要關節，單關節映射處理末端關節
    /// </summary>
    void ApplyHybridMode(float deltaTime)
    {
        // 左手：前 4 個關節用 IK，後 3 個用單關節映射
        if (leftIKSolver != null && leftIK.wristTarget != null)
        {
            Vector3 targetPos = GetSmoothedIKTarget(leftIK, leftIKSolver, deltaTime);
            
            if (leftIKSolver.SolveIK(targetPos, out float[] angles))
            {
                // 前 4 個關節用 IK
                for (int i = 0; i < 4 && i < leftJoints.Length; i++)
                {
                    leftJoints[i]?.SetTargetDirect(angles[i]);
                }
                
                // 後 3 個用單關節映射
                for (int i = 4; i < leftJoints.Length; i++)
                {
                    leftJoints[i]?.Apply(deltaTime);
                }
            }
        }

        // 右手：同樣邏輯
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

    #region IK 輔助方法

    /// <summary>
    /// 獲取平滑後的 IK 目標位置（使用相對座標）
    /// </summary>
    Vector3 GetSmoothedIKTarget(ArmIKConfig config, OpenArmIK ikSolver, float deltaTime)
    {
        if (config.wristTarget == null)
            return config.smoothedPosition;

        // 1. 計算人體手腕相對於肩膀的相對位置
        Vector3 humanShoulderPos = config.shoulderReference != null 
            ? config.shoulderReference.position 
            : Vector3.zero;
        Vector3 humanWristPos = config.wristTarget.position;
        Vector3 relativeToShoulder = humanWristPos - humanShoulderPos;
        
        // 2. 套用縮放（處理尺寸差異）
        relativeToShoulder = Vector3.Scale(relativeToShoulder, config.positionScale);
        
        // 3. 套用偏移
        relativeToShoulder += config.wristTarget.TransformDirection(config.endEffectorOffset);
        
        // 4. 轉換到機械臂基座的座標系統
        Vector3 robotBasePos = Vector3.zero;
        if (ikSolver != null && ikSolver.joints != null && ikSolver.joints.Length > 0)
        {
            if (ikSolver.joints[0].joint != null)
            {
                robotBasePos = ikSolver.joints[0].joint.transform.position;
            }
        }
        Vector3 robotTargetPos = robotBasePos + relativeToShoulder;

        // 5. 約束檢查（相對於機械臂基座）
        if (config.usePositionConstraint)
        {
            Vector3 relativePos = robotTargetPos - robotBasePos;
            relativePos.x = Mathf.Clamp(relativePos.x, config.constraintMin.x, config.constraintMax.x);
            relativePos.y = Mathf.Clamp(relativePos.y, config.constraintMin.y, config.constraintMax.y);
            relativePos.z = Mathf.Clamp(relativePos.z, config.constraintMin.z, config.constraintMax.z);
            robotTargetPos = robotBasePos + relativePos;
        }

        // 6. 平滑
        config.smoothedPosition = Vector3.Lerp(
            config.smoothedPosition, 
            robotTargetPos, 
            Mathf.Clamp01(config.positionSmooth)
        );

        return config.smoothedPosition;
    }

    /// <summary>
    /// 套用 IK 計算出的角度到關節
    /// </summary>
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

    /// <summary>
    /// 校準所有關節的中性姿勢
    /// </summary>
    [ContextMenu("校準所有關節")]
    public void CalibrateAll()
    {
        if (leftJoints != null)
            foreach (var j in leftJoints)
                j?.CalibrateNeutral();

        if (rightJoints != null)
            foreach (var j in rightJoints)
                j?.CalibrateNeutral();

        Debug.Log("✅ OpenArmRetargetIK: 校準完成");
    }

    /// <summary>
    /// 切換控制模式
    /// </summary>
    [ContextMenu("切換控制模式")]
    public void SwitchMode()
    {
        controlMode = (ControlMode)(((int)controlMode + 1) % 3);
        Debug.Log($"🔄 切換到模式: {controlMode}");
    }

    /// <summary>
    /// 設定控制模式
    /// </summary>
    public void SetControlMode(ControlMode mode)
    {
        controlMode = mode;
        Debug.Log($"🔄 設定模式: {controlMode}");
    }

    #endregion

    #region GUI 顯示

    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 250, 150));
        GUILayout.Label("OpenArm Retarget IK", GUI.skin.box);
        
        GUILayout.Label($"控制模式: {controlMode}");
        GUILayout.Label($"切換鍵: {switchModeKey}");

        if (GUILayout.Button($"切換模式 (當前: {controlMode})"))
        {
            SwitchMode();
        }

        if (GUILayout.Button("校準"))
        {
            CalibrateAll();
        }

        GUILayout.EndArea();
    }

    #endregion
}

