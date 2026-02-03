using System;
using UnityEngine;

public class OpenArmRetarget : MonoBehaviour
{
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
        
        [Tooltip("使用 Swing-Twist 四元數分解來讀取角度，可避免歐拉角奇異點問題")]
        public bool useSwingTwistDecomposition = false;  // 四元數分解開關
        Quaternion _neutralRotation;            // 校準時紀錄的四元數（用於 SwingTwist 模式）

        [Header("Mapping")]
        public float scale = 1f;                // 角度比例（可用 -1 反向）
        public float offsetDeg = 0f;            // 角度偏移（度）
        public float minDeg = -180f;            // 目標下限
        public float maxDeg = 180f;             // 目標上限

        [Header("Stability")]
        public float deadZone = 2f;             // 死區：|角度變化| < deadZone 視為靜止
        public float hysteresis = 1.5f;         // 遷就帶：一旦進入死區，要超過此值才解除
        public float smoothTime = 0.08f;        // 平滑時間（秒），越大越平滑
        public float rateLimitDegPerSec = 180f; // 角速度上限（deg/s）
        public float softLimitMargin = 8f;      // 靠近上下限時提前降速的緩衝（度）

        [Header("Anti-Jitter（防抖）")]
        public float jitterThreshold = 1.0f;    // 抖動閾值：變化小於此值視為噪音
        public float holdTime = 0.15f;          // 靜止判定時間（秒）

        [Header("Drive")]
        public float stiffness = 4000f;
        public float damping = 300f;
        public float forceLimit = 10000f;

        // 內部狀態
        float _filteredDeg;        // 濾波後角度
        float _lastCmdDeg;         // 上一幀送給驅動器的角度
        bool  _inDeadHold;         // 是否位於死區並被「鎖住」
        float _deadCenter;         // 死區中心（動態更新）

        // 防抖狀態
        float _lastRawDeg;         // 上一幀的原始角度
        float _stillTimer;         // 靜止計時器
        bool  _isHolding;          // 是否正在保持靜止
        
        // 校準鎖定狀態
        public bool isLocked = false;      // 是否被鎖定在目標角度
        public float lockedTarget = 0f;    // 鎖定的目標角度

        public void CalibrateNeutral()
        {
            if (source == null) return;
            neutralEulerLocal = source.localEulerAngles;
            _neutralRotation = source.localRotation;  // 同時記錄四元數
        }

        public float ReadSourceAngleDegRaw()
        {
            if (source == null) return 0f;

            float raw = 0f;

            if (useSwingTwistDecomposition)
            {
                // 使用 Swing-Twist 四元數分解（避免歐拉角奇異點）
                Quaternion currentRot = source.localRotation;
                
                // 如果有中性校準，計算相對旋轉
                if (useNeutralCalibration)
                {
                    currentRot = Quaternion.Inverse(_neutralRotation) * currentRot;
                }
                
                raw = GetTwistAngle(currentRot, sourceAxis);
            }
            else
            {
                // 原本的歐拉角方式
                var e = source.localEulerAngles;

                // 轉成 -180..180，避免 0/360 跳變
                float sx = Mathf.DeltaAngle(0f, e.x);
                float sy = Mathf.DeltaAngle(0f, e.y);
                float sz = Mathf.DeltaAngle(0f, e.z);

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
            }

            return raw;
        }

        /// <summary>
        /// 使用 Swing-Twist 分解，從四元數中提取繞指定軸的旋轉角度
        /// 這個方法不會受到歐拉角奇異點（Gimbal Lock）的影響
        /// </summary>
        float GetTwistAngle(Quaternion rotation, Axis axis)
        {
            // 根據軸向決定扭轉軸
            Vector3 twistAxis = axis == Axis.X ? Vector3.right :
                                axis == Axis.Y ? Vector3.up : Vector3.forward;

            // 從四元數中提取旋轉軸向量
            Vector3 rotationAxis = new Vector3(rotation.x, rotation.y, rotation.z);
            
            // 將旋轉軸投影到扭轉軸上
            Vector3 projected = Vector3.Project(rotationAxis, twistAxis);
            
            // 重建只包含扭轉分量的四元數
            Quaternion twist = new Quaternion(projected.x, projected.y, projected.z, rotation.w);
            
            // 處理接近零的情況
            float magnitude = Mathf.Sqrt(twist.x * twist.x + twist.y * twist.y + 
                                        twist.z * twist.z + twist.w * twist.w);
            if (magnitude < 0.0001f)
            {
                return 0f;
            }
            
            // 正規化
            twist.x /= magnitude;
            twist.y /= magnitude;
            twist.z /= magnitude;
            twist.w /= magnitude;
            
            // 確保 w 為正（取最短路徑）
            if (twist.w < 0)
            {
                twist.x = -twist.x;
                twist.y = -twist.y;
                twist.z = -twist.z;
                twist.w = -twist.w;
            }

            // 轉成角度（四元數角度 = 2 * acos(w)）
            float angle = 2f * Mathf.Acos(Mathf.Clamp(twist.w, -1f, 1f)) * Mathf.Rad2Deg;
            
            // 判斷旋轉方向
            if (Vector3.Dot(projected, twistAxis) < 0)
            {
                angle = -angle;
            }

            // 轉成 -180 ~ 180 範圍
            return Mathf.DeltaAngle(0f, angle);
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

            // 2) 防抖動檢測：如果角度變化很小，判定為靜止
            float rawDelta = Mathf.Abs(mapped - _lastRawDeg);
            _lastRawDeg = mapped;

            if (rawDelta < jitterThreshold)
            {
                // 變化小於閾值，累加靜止時間
                _stillTimer += deltaTime;
                if (_stillTimer >= holdTime && !_isHolding)
                {
                    // 達到靜止時間，鎖定在當前濾波後的位置
                    _isHolding = true;
                    _deadCenter = _filteredDeg;  // 動態更新死區中心
                }
            }
            else
            {
                // 有明顯移動，重置靜止狀態
                _stillTimer = 0f;
                _isHolding = false;
            }

            // 3) 動態死區：靜止時鎖定在當前位置
            if (_isHolding)
            {
                // 只有超過 deadZone + hysteresis 才解除鎖定
                if (Mathf.Abs(mapped - _deadCenter) > (deadZone + hysteresis))
                {
                    _isHolding = false;
                    _stillTimer = 0f;
                }
                else
                {
                    mapped = _deadCenter;  // 保持在鎖定位置
                }
            }

            // 4) 時間獨立的低通濾波（指數平滑）
            float alpha = 1f - Mathf.Exp(-deltaTime / Mathf.Max(smoothTime, 0.001f));
            _filteredDeg = Mathf.Lerp(_filteredDeg, mapped, alpha);

            // 5) 軟上限（接近邊界時提前降速，避免敲打）
            float lowerSoft = minDeg + softLimitMargin;
            float upperSoft = maxDeg - softLimitMargin;
            float targetDeg = Mathf.Clamp(_filteredDeg, minDeg, maxDeg);

            if (targetDeg > upperSoft && targetDeg < maxDeg)
            {
                // 線性縮小靠近上限的增量
                float t = Mathf.InverseLerp(upperSoft, maxDeg, targetDeg);
                targetDeg = Mathf.Lerp(targetDeg, upperSoft, t);
            }
            else if (targetDeg < lowerSoft && targetDeg > minDeg)
            {
                float t = Mathf.InverseLerp(lowerSoft, minDeg, targetDeg);
                targetDeg = Mathf.Lerp(targetDeg, lowerSoft, t);
            }

            // 6) 限速（deg/s）
            if (rateLimitDegPerSec > 0f && deltaTime > 0f)
            {
                float maxStep = rateLimitDegPerSec * deltaTime;
                float step = Mathf.Clamp(targetDeg - _lastCmdDeg, -maxStep, +maxStep);
                targetDeg = _lastCmdDeg + step;
            }

            // 7) 寫入目標
            drive.target = targetDeg; // ArticulationDrive.target 單位為「度」
            joint.xDrive = drive;

            _lastCmdDeg = targetDeg;
        }
    }

    [Header("Left arm joints (1..7)")]
    public JointMap[] left = new JointMap[7];

    [Header("Right arm joints (1..7)")]
    public JointMap[] right = new JointMap[7];

    [Header("Global")]
    public bool autoCalibrateOnStart = true;

    [Header("APK 初始化延遲")]
    [Tooltip("啟動時延遲初始化的時間（秒），等待 XR 系統準備好")]
    public float initDelay = 1.0f;

    bool _initialized = false;

    void Start()
    {
        // ===== APK 物理震盪修復 =====
        // 1. 全域設定（影響新建物體）
        Physics.defaultSolverIterations = 25;
        Physics.defaultSolverVelocityIterations = 15;

        // 2. 針對每個 ArticulationBody 單獨設定（影響已存在的物體）
        SetArticulationSolverIterations(left, 25, 15);
        SetArticulationSolverIterations(right, 25, 15);
        // ===== 修復結束 =====

        // 延遲初始化，等待 XR 系統準備好
        StartCoroutine(DelayedInitialize());
    }

    System.Collections.IEnumerator DelayedInitialize()
    {
        // 等待指定時間，讓 XR 系統完成初始化
        yield return new WaitForSeconds(initDelay);

        // 等待一幀，確保所有 Transform 已更新
        yield return null;

        Initialize();
    }

    void Initialize()
    {
        if (_initialized) return;

        if (autoCalibrateOnStart)
        {
            if (left != null)  foreach (var j in left)  j?.CalibrateNeutral();
            if (right != null) foreach (var j in right) j?.CalibrateNeutral();
        }

        _initialized = true;
        Debug.Log("[OpenArmRetarget] 初始化完成");
    }

    /// <summary>
    /// 當 Quest 被放下/戴回時觸發，重新校準
    /// </summary>
    void OnApplicationPause(bool pauseStatus)
    {
        if (!pauseStatus && _initialized)
        {
            // 從暫停恢復時，重新校準
            Debug.Log("[OpenArmRetarget] 從暫停恢復，重新校準");
            StartCoroutine(RecalibrateAfterResume());
        }
    }

    System.Collections.IEnumerator RecalibrateAfterResume()
    {
        // 等待一小段時間讓 XR 系統恢復
        yield return new WaitForSeconds(0.5f);
        yield return null;

        if (left != null)  foreach (var j in left)  j?.CalibrateNeutral();
        if (right != null) foreach (var j in right) j?.CalibrateNeutral();

        Debug.Log("[OpenArmRetarget] 重新校準完成");
    }

    /// <summary>
    /// 設定 ArticulationBody 的求解器迭代次數（方案3）
    /// </summary>
    void SetArticulationSolverIterations(JointMap[] joints, int posIterations, int velIterations)
    {
        if (joints == null) return;

        foreach (var j in joints)
        {
            if (j?.joint != null)
            {
                j.joint.solverIterations = posIterations;
                j.joint.solverVelocityIterations = velIterations;

                // 額外增加關節摩擦和角阻尼來抑制震盪
                j.joint.jointFriction = 5f;
                j.joint.angularDamping = 50f;
            }
        }
    }

    void FixedUpdate()
    {
        // 只有初始化完成後才執行
        if (!_initialized) return;

        float dt = Time.fixedDeltaTime;
        if (left != null)  foreach (var j in left)  j?.Apply(dt);
        if (right != null) foreach (var j in right) j?.Apply(dt);
    }
}
