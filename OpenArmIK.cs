using UnityEngine;

/// <summary>
/// OpenArm 7-DOF 機械臂的正向/逆向運動學求解器
/// 使用 CCD (Cyclic Coordinate Descent) 算法
/// </summary>
public class OpenArmIK : MonoBehaviour
{
    [System.Serializable]
    public class JointInfo
    {
        public string name;
        public ArticulationBody joint;
        public Vector3 axis = Vector3.right;  // 旋轉軸（local space）
        public float minDeg = -180f;
        public float maxDeg = 180f;
        
        [HideInInspector] public float currentAngle;  // 當前角度（度）
        [HideInInspector] public Vector3 position;    // 關節位置（world space）
    }

    [Header("OpenArm 關節鏈（從基座到末端）")]
    public JointInfo[] joints = new JointInfo[7];

    [Header("末端執行器")]
    public Transform endEffector;  // 末端執行器 Transform

    [Header("IK 設定")]
    [Range(1, 50)]
    public int maxIterations = 20;       // CCD 最大迭代次數
    
    [Range(0.001f, 0.1f)]
    public float tolerance = 0.01f;      // 容許誤差（公尺）
    
    [Range(0.1f, 1.0f)]
    public float learningRate = 0.5f;    // 學習率（每次迭代的角度變化比例）

    [Header("調試")]
    public bool showDebugInfo = true;
    public bool drawGizmos = true;
    public Color gizmoColor = Color.cyan;

    // 內部狀態
    private Vector3 _lastTargetPosition;
    private bool _ikSolved = false;

    void Start()
    {
        if (endEffector == null && joints.Length > 0)
        {
            Debug.LogWarning("⚠️ OpenArmIK: 未設定末端執行器，使用最後一個關節");
        }
    }

    /// <summary>
    /// 正向運動學：從關節角度計算末端位置
    /// </summary>
    public Vector3 ForwardKinematics(float[] angles)
    {
        if (angles == null || angles.Length != joints.Length)
        {
            Debug.LogWarning($"⚠️ OpenArmIK FK: 角度數量不匹配 (需要 {joints.Length} 個)");
            return Vector3.zero;
        }

        // 更新關節角度並計算位置
        UpdateJointPositions(angles);

        // 返回末端執行器位置
        if (endEffector != null)
            return endEffector.position;
        else if (joints.Length > 0 && joints[joints.Length - 1].joint != null)
            return joints[joints.Length - 1].joint.transform.position;
        
        return Vector3.zero;
    }

    /// <summary>
    /// 逆向運動學：從目標位置計算關節角度
    /// 使用 CCD (Cyclic Coordinate Descent) 算法
    /// </summary>
    public bool SolveIK(Vector3 targetPosition, out float[] resultAngles)
    {
        resultAngles = new float[joints.Length];
        
        // 初始化：讀取當前關節角度
        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i].joint != null)
            {
                var drive = joints[i].joint.xDrive;
                resultAngles[i] = drive.target;
            }
        }

        _lastTargetPosition = targetPosition;
        Vector3 endPos = GetEndEffectorPosition();
        float initialDistance = Vector3.Distance(endPos, targetPosition);

        if (showDebugInfo)
            Debug.Log($"🎯 OpenArmIK: 開始求解 IK | 目標: {targetPosition} | 初始距離: {initialDistance:F3}m");

        // CCD 迭代
        for (int iter = 0; iter < maxIterations; iter++)
        {
            bool improved = false;

            // 從末端往基座方向遍歷每個關節
            for (int i = joints.Length - 1; i >= 0; i--)
            {
                if (joints[i].joint == null) continue;

                // 更新末端位置
                endPos = GetEndEffectorPosition();
                float currentDistance = Vector3.Distance(endPos, targetPosition);

                // 檢查是否已達到容許誤差
                if (currentDistance < tolerance)
                {
                    _ikSolved = true;
                    if (showDebugInfo)
                        Debug.Log($"✅ OpenArmIK: 求解成功 | 迭代: {iter} | 誤差: {currentDistance:F4}m");
                    return true;
                }

                // 計算向量
                Vector3 jointPos = joints[i].joint.transform.position;
                Vector3 toEnd = endPos - jointPos;
                Vector3 toTarget = targetPosition - jointPos;

                // 避免除以零
                if (toEnd.sqrMagnitude < 0.0001f) continue;

                // 計算旋轉軸（world space）
                Vector3 rotationAxis = joints[i].joint.transform.TransformDirection(joints[i].axis);

                // 計算需要旋轉的角度
                Vector3 projEnd = Vector3.ProjectOnPlane(toEnd, rotationAxis);
                Vector3 projTarget = Vector3.ProjectOnPlane(toTarget, rotationAxis);

                if (projEnd.sqrMagnitude < 0.0001f || projTarget.sqrMagnitude < 0.0001f)
                    continue;

                float angle = Vector3.SignedAngle(projEnd, projTarget, rotationAxis);
                
                // 套用學習率
                angle *= learningRate;

                // 更新角度
                float newAngle = resultAngles[i] + angle;
                newAngle = Mathf.Clamp(newAngle, joints[i].minDeg, joints[i].maxDeg);
                
                resultAngles[i] = newAngle;
                
                // 套用到關節（用於測試）
                var drive = joints[i].joint.xDrive;
                drive.target = newAngle;
                joints[i].joint.xDrive = drive;

                improved = true;
            }

            // 如果沒有改善，提前結束
            if (!improved)
            {
                if (showDebugInfo)
                    Debug.LogWarning($"⚠️ OpenArmIK: 迭代 {iter} 無改善，提前結束");
                break;
            }
        }

        // 未達到容許誤差
        endPos = GetEndEffectorPosition();
        float finalDistance = Vector3.Distance(endPos, targetPosition);
        _ikSolved = finalDistance < tolerance * 2f; // 放寬一點

        if (showDebugInfo)
        {
            if (_ikSolved)
                Debug.Log($"✅ OpenArmIK: 部分成功 | 最終誤差: {finalDistance:F4}m");
            else
                Debug.LogWarning($"⚠️ OpenArmIK: 未能收斂 | 最終誤差: {finalDistance:F4}m");
        }

        return _ikSolved;
    }

    /// <summary>
    /// 簡化版 IK：只使用前 3 個關節（肩關節）
    /// 用於快速定位
    /// </summary>
    public bool SolveIKSimple(Vector3 targetPosition, out float[] resultAngles)
    {
        resultAngles = new float[joints.Length];
        
        if (joints.Length < 3)
        {
            Debug.LogError("❌ OpenArmIK: 關節數量不足，無法執行簡化 IK");
            return false;
        }

        // 只處理前 3 個關節
        Vector3 basePos = joints[0].joint.transform.position;
        Vector3 toTarget = targetPosition - basePos;

        // 計算方位角（Azimuth）- Joint 1
        float azimuth = Mathf.Atan2(toTarget.x, toTarget.z) * Mathf.Rad2Deg;
        resultAngles[0] = Mathf.Clamp(azimuth, joints[0].minDeg, joints[0].maxDeg);

        // 計算俯仰角（Elevation）- Joint 2
        float distance = new Vector2(toTarget.x, toTarget.z).magnitude;
        float elevation = Mathf.Atan2(toTarget.y, distance) * Mathf.Rad2Deg;
        resultAngles[1] = Mathf.Clamp(elevation, joints[1].minDeg, joints[1].maxDeg);

        // Joint 3 保持相對角度
        resultAngles[2] = 0f;

        // 其餘關節保持當前角度
        for (int i = 3; i < joints.Length; i++)
        {
            if (joints[i].joint != null)
            {
                var drive = joints[i].joint.xDrive;
                resultAngles[i] = drive.target;
            }
        }

        if (showDebugInfo)
            Debug.Log($"🎯 OpenArmIK Simple: 方位角={azimuth:F1}° 俯仰角={elevation:F1}°");

        return true;
    }

    /// <summary>
    /// 套用關節角度到 ArticulationBody
    /// </summary>
    public void ApplyJointAngles(float[] angles)
    {
        if (angles == null || angles.Length != joints.Length)
        {
            Debug.LogWarning($"⚠️ OpenArmIK: 角度數量不匹配");
            return;
        }

        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i].joint != null)
            {
                var drive = joints[i].joint.xDrive;
                drive.target = Mathf.Clamp(angles[i], joints[i].minDeg, joints[i].maxDeg);
                joints[i].joint.xDrive = drive;
                joints[i].currentAngle = angles[i];
            }
        }
    }

    /// <summary>
    /// 獲取當前末端執行器位置
    /// </summary>
    public Vector3 GetEndEffectorPosition()
    {
        if (endEffector != null)
            return endEffector.position;
        else if (joints.Length > 0 && joints[joints.Length - 1].joint != null)
            return joints[joints.Length - 1].joint.transform.position;
        
        return Vector3.zero;
    }

    /// <summary>
    /// 更新關節位置（內部使用）
    /// </summary>
    private void UpdateJointPositions(float[] angles)
    {
        for (int i = 0; i < joints.Length && i < angles.Length; i++)
        {
            if (joints[i].joint != null)
            {
                joints[i].currentAngle = angles[i];
                joints[i].position = joints[i].joint.transform.position;
            }
        }
    }

    /// <summary>
    /// 獲取當前所有關節角度
    /// </summary>
    public float[] GetCurrentAngles()
    {
        float[] angles = new float[joints.Length];
        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i].joint != null)
            {
                var drive = joints[i].joint.xDrive;
                angles[i] = drive.target;
            }
        }
        return angles;
    }

    #region 調試與視覺化

    void OnDrawGizmos()
    {
        if (!drawGizmos || joints == null || joints.Length == 0) return;

        Gizmos.color = gizmoColor;

        // 繪製關節鏈
        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i].joint == null) continue;

            Vector3 pos = joints[i].joint.transform.position;
            Gizmos.DrawWireSphere(pos, 0.02f);

            // 繪製到下一個關節的連線
            if (i < joints.Length - 1 && joints[i + 1].joint != null)
            {
                Vector3 nextPos = joints[i + 1].joint.transform.position;
                Gizmos.DrawLine(pos, nextPos);
            }
        }

        // 繪製末端執行器
        Vector3 endPos = GetEndEffectorPosition();
        Gizmos.color = _ikSolved ? Color.green : Color.yellow;
        Gizmos.DrawWireSphere(endPos, 0.03f);

        // 繪製目標位置
        if (_lastTargetPosition != Vector3.zero)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(_lastTargetPosition, 0.025f);
            Gizmos.DrawLine(endPos, _lastTargetPosition);
        }

        // 繪製旋轉軸
        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i].joint == null) continue;

            Vector3 pos = joints[i].joint.transform.position;
            Vector3 axis = joints[i].joint.transform.TransformDirection(joints[i].axis);
            
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(pos, axis * 0.05f);
        }
    }

    [ContextMenu("測試 IK - 向前 0.3m")]
    void TestIKForward()
    {
        Vector3 basePos = joints[0].joint.transform.position;
        Vector3 target = basePos + transform.forward * 0.3f + Vector3.up * 0.2f;
        
        if (SolveIK(target, out float[] angles))
        {
            Debug.Log($"✅ 測試成功: {string.Join(", ", angles)}");
        }
    }

    [ContextMenu("測試簡化 IK")]
    void TestIKSimple()
    {
        Vector3 basePos = joints[0].joint.transform.position;
        Vector3 target = basePos + transform.forward * 0.3f + Vector3.up * 0.2f;
        
        if (SolveIKSimple(target, out float[] angles))
        {
            ApplyJointAngles(angles);
            Debug.Log($"✅ 簡化 IK 成功");
        }
    }

    #endregion
}

