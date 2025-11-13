using System;
using UnityEngine;

/// <summary>
/// OpenArm Retarget with IK support - 修復版 v2
/// 修正了座標系轉換問題
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

        [Header("末端執行器偏移（局部座標）")]
        public Vector3 endEffectorOffset = Vector3.zero;

        [Header("縮放設定")]
        [Tooltip("統一縮放因子（人體臂長 vs 機械臂長度）")]
        public float uniformScale = 1.0f;

        [Header("IK 平滑")]
        [Range(0f, 1f)]
        [Tooltip("值越大追蹤越快，0=完全不動，1=立即追蹤")]
        public float positionSmooth = 0.3f;

        [Range(0f, 1f)]
        public float rotationSmooth = 0.3f;

        [Header("IK 約束（機械臂局部座標系）")]
        public bool usePositionConstraint = true;
        public Vector3 constraintMin = new Vector3(-0.5f, -0.3f, 0.1f);
        public Vector3 constraintMax = new Vector3(0.5f, 0.5f, 0.8f);

        // 內部平滑狀態
        [HideInInspector] public Vector3 smoothedPosition;
        [HideInInspector] public Quaternion smoothedRotation = Quaternion.identity;
        [HideInInspector] public bool isInitialized = false;
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
    public bool drawDebugGizmos = true;

    void Start()
    {
        if (autoCalibrateOnStart)
        {
            CalibrateAll();
        }

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

    #region IK 輔助方法（✅ 修復版）

    void InitializeIKConfig(ArmIKConfig config, OpenArmIK ikSolver)
    {
        if (config.wristTarget != null && ikSolver != null)
        {
            config.smoothedPosition = GetSmoothedIKTarget(config, ikSolver, 0f);
            config.isInitialized = true;
        }
    }

    /// <summary>
    /// 獲取平滑後的 IK 目標位置（✅ 完全修復版）
    /// </summary>
    Vector3 GetSmoothedIKTarget(ArmIKConfig config, OpenArmIK ikSolver, float deltaTime)
    {
        if (config.wristTarget == null || ikSolver == null)
            return config.smoothedPosition;

        // 1. 獲取機械臂基座 Transform
        Transform robotBase = null;
        if (ikSolver.joints != null && ikSolver.joints.Length > 0 && ikSolver.joints[0].joint != null)
        {
            robotBase = ikSolver.joints[0].joint.transform.parent;
            if (robotBase == null)
                robotBase = ikSolver.joints[0].joint.transform;
        }

        if (robotBase == null)
        {
            Debug.LogWarning("⚠️ 無法找到機械臂基座");
            return config.smoothedPosition;
        }

        // 2. 計算人體手臂向量（世界座標）
        Vector3 humanShoulderPos = config.shoulderReference != null
            ? config.shoulderReference.position
            : config.wristTarget.position;
        Vector3 humanWristPos = config.wristTarget.position;
        Vector3 humanArmVectorWorld = humanWristPos - humanShoulderPos;

        // ✅ 修正：將人體手臂向量轉換到機械臂的局部座標系
        Vector3 humanArmVectorLocal = robotBase.InverseTransformDirection(humanArmVectorWorld);

        // 3. 套用統一縮放（在局部座標系中）
        Vector3 scaledArmVectorLocal = humanArmVectorLocal * config.uniformScale;

        // 4. 套用末端執行器偏移（在局部座標系中）
        Vector3 localTarget = scaledArmVectorLocal + config.endEffectorOffset;

        // 5. 約束檢查（在局部座標系中）
        if (config.usePositionConstraint)
        {
            localTarget.x = Mathf.Clamp(localTarget.x, config.constraintMin.x, config.constraintMax.x);
            localTarget.y = Mathf.Clamp(localTarget.y, config.constraintMin.y, config.constraintMax.y);
            localTarget.z = Mathf.Clamp(localTarget.z, config.constraintMin.z, config.constraintMax.z);
        }

        // 6. 轉回世界座標
        Vector3 finalTargetWorldPos = robotBase.TransformPoint(localTarget);

        // 7. 平滑處理
        if (!config.isInitialized || deltaTime <= 0f)
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

        // 🔍 調試輸出
        if (showDebugInfo && Time.frameCount % 60 == 0)
        {
            Debug.Log($"=== IK 目標計算 ===");
            Debug.Log($"人體臂向量(世界): {humanArmVectorWorld}");
            Debug.Log($"人體臂向量(局部): {humanArmVectorLocal}");
            Debug.Log($"縮放後(局部): {scaledArmVectorLocal}");
            Debug.Log($"約束後(局部): {localTarget}");
            Debug.Log($"最終目標(世界): {finalTargetWorldPos}");
            Debug.Log($"機械臂基座: {robotBase.position}");
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

        Debug.Log("✅ OpenArmRetargetIK: 校準完成");
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

    #endregion

    #region 調試視覺化

    void OnDrawGizmos()
    {
        if (!drawDebugGizmos) return;

        if (leftIK.wristTarget != null && leftIKSolver != null)
        {
            DrawIKDebug(leftIK, leftIKSolver, Color.blue);
        }

        if (rightIK.wristTarget != null && rightIKSolver != null)
        {
            DrawIKDebug(rightIK, rightIKSolver, Color.red);
        }
    }

    void DrawIKDebug(ArmIKConfig config, OpenArmIK ikSolver, Color color)
    {
        Gizmos.color = color;
        Gizmos.DrawWireSphere(config.wristTarget.position, 0.03f);

        Gizmos.color = Color.Lerp(color, Color.white, 0.5f);
        Gizmos.DrawWireSphere(config.smoothedPosition, 0.025f);

        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(config.wristTarget.position, config.smoothedPosition);

        Vector3 endPos = ikSolver.GetEndEffectorPosition();
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(endPos, 0.02f);

        Gizmos.color = Color.magenta;
        Gizmos.DrawLine(config.smoothedPosition, endPos);

#if UNITY_EDITOR
        float distance = Vector3.Distance(config.smoothedPosition, endPos);
        UnityEditor.Handles.Label(
            (config.smoothedPosition + endPos) * 0.5f,
            $"IK誤差: {distance * 1000f:F1}mm"
        );
#endif
    }

    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 300, 200));
        GUILayout.Label("OpenArm Retarget IK (修復版 v2)", GUI.skin.box);

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

        if (controlMode == ControlMode.IK || controlMode == ControlMode.Hybrid)
        {
            if (leftIKSolver != null && leftIK.wristTarget != null)
            {
                float error = Vector3.Distance(
                    leftIKSolver.GetEndEffectorPosition(),
                    leftIK.smoothedPosition
                ) * 1000f;
                GUILayout.Label($"左手誤差: {error:F1}mm");
            }

            if (rightIKSolver != null && rightIK.wristTarget != null)
            {
                float error = Vector3.Distance(
                    rightIKSolver.GetEndEffectorPosition(),
                    rightIK.smoothedPosition
                ) * 1000f;
                GUILayout.Label($"右手誤差: {error:F1}mm");
            }
        }

        GUILayout.EndArea();
    }

    #endregion
}