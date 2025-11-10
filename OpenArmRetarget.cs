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
            // 進入死區就「鎖住」在 deadCenter（通常 0）
            if (_inDeadHold)
            {
                // 只有超過 deadZone + hysteresis 才解除
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

            // 3) 低通濾波（EM A）
            _filteredDeg = Mathf.Lerp(_filteredDeg, mapped, Mathf.Clamp01(smoothAlpha));

            // 4) 軟上限（接近邊界時提前降速，避免敲打）
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

            // 5) 限速（deg/s）
            if (rateLimitDegPerSec > 0f && deltaTime > 0f)
            {
                float maxStep = rateLimitDegPerSec * deltaTime;
                float step = Mathf.Clamp(targetDeg - _lastCmdDeg, -maxStep, +maxStep);
                targetDeg = _lastCmdDeg + step;
            }

            // 6) 寫入目標
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

    void Start()
    {
        if (autoCalibrateOnStart)
        {
            if (left != null)  foreach (var j in left)  j?.CalibrateNeutral();
            if (right != null) foreach (var j in right) j?.CalibrateNeutral();
        }
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        if (left != null)  foreach (var j in left)  j?.Apply(dt);
        if (right != null) foreach (var j in right) j?.Apply(dt);
    }
}
