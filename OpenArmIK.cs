using UnityEngine;

/// <summary>
/// OpenArm 7-DOF 機械臂的正向/逆向運動學求解器
/// 使用 CCD (Cyclic Coordinate Descent) 算法 - 完全修復版
/// ✅ 針對 OpenArm 實際結構優化
/// </summary>
public class OpenArmIK : MonoBehaviour
{
    [System.Serializable]
    public class JointInfo
    {
        public string name;
        public ArticulationBody joint;
        public Vector3 axis = Vector3.right;
        public float minDeg = -180f;
        public float maxDeg = 180f;

        [HideInInspector] public float currentAngle;
        [HideInInspector] public Vector3 position;
        
        // ✅ 新增：快取連桿偏移量（局部座標）
        [HideInInspector] public Vector3 linkOffsetLocal;
        [HideInInspector] public float linkLength; // 連桿長度（用於調試）
    }

    [Header("OpenArm 關節鏈（從基座到末端）")]
    [Tooltip("Joint1:Shoulder Pitch, J2:Shoulder Roll, J3:Shoulder Yaw, J4:Elbow Pitch, J5:Wrist Roll, J6:Wrist Yaw, J7:Wrist Pitch")]
    public JointInfo[] joints = new JointInfo[7];

    [Header("末端執行器")]
    public Transform endEffector;

    [Header("IK 設定")]
    [Range(1, 100)]
    public int maxIterations = 30;

    [Range(0.001f, 0.1f)]
    public float tolerance = 0.01f;

    [Range(0.1f, 1.0f)]
    public float learningRate = 0.5f;

    [Header("調試")]
    public bool showDebugInfo = true;
    public bool drawGizmos = true;
    public Color gizmoColor = Color.cyan;
    public bool debugFK = false; // 調試 FK 計算

    // 內部狀態
    private Vector3 _lastTargetPosition;
    private bool _ikSolved = false;
    private bool _linkOffsetsInitialized = false;

    void Start()
    {
        if (endEffector == null && joints.Length > 0)
        {
            Debug.LogWarning("⚠️ OpenArmIK: 未設定末端執行器，使用最後一個關節");
        }

        // ✅ 初始化：快取所有連桿偏移量
        InitializeLinkOffsets();
    }

    /// <summary>
    /// ✅ 初始化：預先計算並快取所有連桿的局部偏移量
    /// 這樣在 FK 計算時就不需要每次都查詢 Transform
    /// </summary>
    void InitializeLinkOffsets()
    {
        if (_linkOffsetsInitialized) return;

        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i].joint == null)
            {
                Debug.LogWarning($"⚠️ Joint {i} ({joints[i].name}) 未設定 ArticulationBody");
                continue;
            }

            // 計算到下一個關節或末端執行器的局部偏移
            if (i < joints.Length - 1 && joints[i + 1].joint != null)
            {
                // 到下一個關節的局部偏移
                joints[i].linkOffsetLocal = joints[i].joint.transform.InverseTransformPoint(
                    joints[i + 1].joint.transform.position
                );
                joints[i].linkLength = joints[i].linkOffsetLocal.magnitude;
            }
            else if (i == joints.Length - 1)
            {
                // 最後一個關節到末端執行器的偏移
                if (endEffector != null)
                {
                    joints[i].linkOffsetLocal = joints[i].joint.transform.InverseTransformPoint(
                        endEffector.position
                    );
                    joints[i].linkLength = joints[i].linkOffsetLocal.magnitude;
                }
                else
                {
                    // 如果沒有末端執行器，偏移為零
                    joints[i].linkOffsetLocal = Vector3.zero;
                    joints[i].linkLength = 0f;
                }
            }

            if (showDebugInfo)
            {
                Debug.Log($"📏 Joint {i} ({joints[i].name}): " +
                         $"LinkOffset={joints[i].linkOffsetLocal}, Length={joints[i].linkLength:F3}m");
            }
        }

        _linkOffsetsInitialized = true;
        Debug.Log("✅ OpenArmIK: 連桿偏移量初始化完成");
    }

    /// <summary>
    /// ✅ 正向運動學：根據關節角度計算末端位置
    /// 不依賴 Transform 的實際位置，使用數學計算
    /// </summary>
    private Vector3 ComputeEndEffectorPosition(float[] angles)
    {
        if (!_linkOffsetsInitialized)
        {
            Debug.LogWarning("⚠️ 連桿偏移量未初始化，使用實際位置");
            return GetEndEffectorPosition();
        }

        if (angles == null || angles.Length != joints.Length)
        {
            Debug.LogWarning($"⚠️ 角度數量不匹配: 需要 {joints.Length}, 得到 {angles?.Length ?? 0}");
            return GetEndEffectorPosition();
        }

        if (joints[0].joint == null)
        {
            Debug.LogWarning("⚠️ 基座關節未設定");
            return Vector3.zero;
        }

        // 從基座開始累積變換
        // 注意：使用基座的父物件作為參考座標系（通常是機械臂的 base）
        Transform baseParent = joints[0].joint.transform.parent;
        Vector3 position = joints[0].joint.transform.position;
        Quaternion rotation = baseParent != null ? baseParent.rotation : Quaternion.identity;

        // ✅ 針對 OpenArm 7-DOF 結構的 FK 計算
        // Joint1-3: 肩膀 (Pitch-Roll-Yaw)
        // Joint4: 肘部 (Pitch)
        // Joint5-7: 手腕 (Roll-Yaw-Pitch)

        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i].joint == null) continue;

            // 將局部旋轉軸轉換到世界座標
            Vector3 worldAxis = rotation * joints[i].axis;

            // 套用當前關節的旋轉（使用計算的角度，不是實際角度）
            rotation = Quaternion.AngleAxis(angles[i], worldAxis) * rotation;

            // 移動到下一個關節位置
            // 將局部偏移量轉換到世界座標後加到當前位置
            position += rotation * joints[i].linkOffsetLocal;
        }

        return position;
    }

    /// <summary>
    /// 正向運動學：從關節角度計算末端位置（公開介面）
    /// </summary>
    public Vector3 ForwardKinematics(float[] angles)
    {
        if (angles == null || angles.Length != joints.Length)
        {
            Debug.LogWarning($"⚠️ OpenArmIK FK: 角度數量不匹配 (需要 {joints.Length} 個)");
            return Vector3.zero;
        }

        if (!_linkOffsetsInitialized)
        {
            InitializeLinkOffsets();
        }

        return ComputeEndEffectorPosition(angles);
    }

    /// <summary>
    /// 逆向運動學：從目標位置計算關節角度
    /// 使用 CCD (Cyclic Coordinate Descent) 算法
    /// ✅ 完全修復版：使用正確的 FK 計算
    /// </summary>
    public bool SolveIK(Vector3 targetPosition, out float[] resultAngles)
    {
        if (!_linkOffsetsInitialized)
        {
            InitializeLinkOffsets();
        }

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
        
        // ✅ 使用正確的 FK 計算初始距離
        Vector3 endPos = ComputeEndEffectorPosition(resultAngles);
        float initialDistance = Vector3.Distance(endPos, targetPosition);

        if (showDebugInfo)
            Debug.Log($"🎯 OpenArmIK: 開始求解 IK | 目標: {targetPosition} | 初始距離: {initialDistance:F4}m");

        float bestDistance = initialDistance;
        float[] bestAngles = (float[])resultAngles.Clone();

        // CCD 迭代
        for (int iter = 0; iter < maxIterations; iter++)
        {
            bool improved = false;

            // 從末端往基座方向遍歷每個關節
            for (int i = joints.Length - 1; i >= 0; i--)
            {
                if (joints[i].joint == null) continue;

                // ✅ 使用 FK 計算當前末端位置
                endPos = ComputeEndEffectorPosition(resultAngles);
                float currentDistance = Vector3.Distance(endPos, targetPosition);

                // 檢查是否已達到容許誤差
                if (currentDistance < tolerance)
                {
                    _ikSolved = true;
                    if (showDebugInfo)
                        Debug.Log($"✅ OpenArmIK: 求解成功 | 迭代: {iter} | 誤差: {currentDistance:F4}m ({currentDistance * 1000f:F1}mm)");
                    return true;
                }

                // 計算當前關節在世界座標中的位置
                // 需要用 FK 計算到這個關節為止的位置
                Vector3 jointPos = ComputeJointPosition(resultAngles, i);
                
                Vector3 toEnd = endPos - jointPos;
                Vector3 toTarget = targetPosition - jointPos;

                // 避免除以零
                if (toEnd.sqrMagnitude < 0.0001f) continue;

                // 計算旋轉軸（world space）
                // 需要計算到當前關節為止的累積旋轉
                Quaternion jointRotation = ComputeJointRotation(resultAngles, i);
                Vector3 rotationAxis = jointRotation * joints[i].axis;

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

                // ✅ 檢查是否改善（使用 FK 計算）
                float newDistance = Vector3.Distance(ComputeEndEffectorPosition(resultAngles), targetPosition);
                if (newDistance < bestDistance)
                {
                    bestDistance = newDistance;
                    bestAngles = (float[])resultAngles.Clone();
                    improved = true;
                }
            }

            // 如果沒有改善，提前結束
            if (!improved && iter > 5) // 至少迭代 5 次
            {
                if (showDebugInfo)
                    Debug.Log($"⚠️ OpenArmIK: 迭代 {iter} 無改善，提前結束");
                break;
            }
        }

        // 使用最佳結果
        resultAngles = bestAngles;

        // 未達到容許誤差
        endPos = ComputeEndEffectorPosition(resultAngles);
        float finalDistance = Vector3.Distance(endPos, targetPosition);
        _ikSolved = finalDistance < tolerance * 2f;

        if (showDebugInfo)
        {
            if (_ikSolved)
                Debug.Log($"✅ OpenArmIK: 部分成功 | 最終誤差: {finalDistance:F4}m ({finalDistance * 1000f:F1}mm)");
            else
                Debug.LogWarning($"⚠️ OpenArmIK: 未能收斂 | 最終誤差: {finalDistance:F4}m ({finalDistance * 1000f:F1}mm)");
        }

        return _ikSolved;
    }

    /// <summary>
    /// ✅ 計算指定關節在世界座標中的位置（FK 的部分計算）
    /// </summary>
    private Vector3 ComputeJointPosition(float[] angles, int jointIndex)
    {
        if (jointIndex < 0 || jointIndex >= joints.Length) 
            return Vector3.zero;

        Transform baseParent = joints[0].joint.transform.parent;
        Vector3 position = joints[0].joint.transform.position;
        Quaternion rotation = baseParent != null ? baseParent.rotation : Quaternion.identity;

        // 只計算到指定關節為止
        for (int i = 0; i < jointIndex; i++)
        {
            if (joints[i].joint == null) continue;

            Vector3 worldAxis = rotation * joints[i].axis;
            rotation = Quaternion.AngleAxis(angles[i], worldAxis) * rotation;
            position += rotation * joints[i].linkOffsetLocal;
        }

        return position;
    }

    /// <summary>
    /// ✅ 計算到指定關節為止的累積旋轉
    /// </summary>
    private Quaternion ComputeJointRotation(float[] angles, int jointIndex)
    {
        if (jointIndex < 0 || jointIndex >= joints.Length) 
            return Quaternion.identity;

        Transform baseParent = joints[0].joint.transform.parent;
        Quaternion rotation = baseParent != null ? baseParent.rotation : Quaternion.identity;

        // 計算到指定關節為止的累積旋轉
        for (int i = 0; i < jointIndex; i++)
        {
            if (joints[i].joint == null) continue;

            Vector3 worldAxis = rotation * joints[i].axis;
            rotation = Quaternion.AngleAxis(angles[i], worldAxis) * rotation;
        }

        return rotation;
    }

    /// <summary>
    /// 簡化版 IK：只使用前 4 個關節（肩膀 + 肘部）
    /// 用於快速定位
    /// </summary>
    public bool SolveIKSimple(Vector3 targetPosition, out float[] resultAngles)
    {
        resultAngles = new float[joints.Length];

        if (joints.Length < 4)
        {
            Debug.LogError("❌ OpenArmIK: 關節數量不足，無法執行簡化 IK");
            return false;
        }

        // 獲取基座位置
        Vector3 basePos = joints[0].joint.transform.position;
        Vector3 toTarget = targetPosition - basePos;

        // Joint1: Shoulder Pitch（前後俯仰）
        float distance2D = new Vector2(toTarget.x, toTarget.z).magnitude;
        float pitchAngle = Mathf.Atan2(toTarget.y, distance2D) * Mathf.Rad2Deg;
        resultAngles[0] = Mathf.Clamp(pitchAngle, joints[0].minDeg, joints[0].maxDeg);

        // Joint2: Shoulder Roll（左右擺動）- 簡化為 0
        resultAngles[1] = 0f;

        // Joint3: Shoulder Yaw（水平旋轉）
        float yawAngle = Mathf.Atan2(toTarget.x, toTarget.z) * Mathf.Rad2Deg;
        resultAngles[2] = Mathf.Clamp(yawAngle, joints[2].minDeg, joints[2].maxDeg);

        // Joint4: Elbow Pitch - 保持伸直
        resultAngles[3] = 0f;

        // 其餘關節保持當前角度
        for (int i = 4; i < joints.Length; i++)
        {
            if (joints[i].joint != null)
            {
                var drive = joints[i].joint.xDrive;
                resultAngles[i] = drive.target;
            }
        }

        if (showDebugInfo)
            Debug.Log($"🎯 OpenArmIK Simple: Pitch={pitchAngle:F1}° Yaw={yawAngle:F1}°");

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
    /// 獲取當前末端執行器位置（實際物理位置）
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
            else if (i == joints.Length - 1 && endEffector != null)
            {
                Gizmos.DrawLine(pos, endEffector.position);
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
            
            // 顯示誤差距離
            float distance = Vector3.Distance(endPos, _lastTargetPosition);
            #if UNITY_EDITOR
            UnityEditor.Handles.Label(
                (endPos + _lastTargetPosition) * 0.5f,
                $"誤差: {distance * 1000f:F1}mm"
            );
            #endif
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

        // ✅ 調試 FK：繪製計算的位置 vs 實際位置
        if (debugFK && _linkOffsetsInitialized)
        {
            float[] currentAngles = GetCurrentAngles();
            Vector3 computedPos = ComputeEndEffectorPosition(currentAngles);
            
            Gizmos.color = Color.magenta;
            Gizmos.DrawWireSphere(computedPos, 0.035f);
            
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(endPos, computedPos);
            
            float fkError = Vector3.Distance(endPos, computedPos);
            #if UNITY_EDITOR
            UnityEditor.Handles.Label(
                computedPos,
                $"FK誤差: {fkError * 1000f:F2}mm\n(紫=計算 黃=實際)"
            );
            #endif
        }
    }

    [ContextMenu("測試 FK 準確度")]
    void TestFKAccuracy()
    {
        if (!_linkOffsetsInitialized)
            InitializeLinkOffsets();

        float[] currentAngles = GetCurrentAngles();
        Vector3 actualPos = GetEndEffectorPosition();
        Vector3 computedPos = ComputeEndEffectorPosition(currentAngles);
        float error = Vector3.Distance(actualPos, computedPos);

        Debug.Log($"📊 FK 準確度測試:\n" +
                 $"  實際位置: {actualPos}\n" +
                 $"  計算位置: {computedPos}\n" +
                 $"  誤差: {error * 1000f:F2}mm\n" +
                 $"  {(error < 0.001f ? "✅ 非常準確" : error < 0.01f ? "✅ 準確" : "⚠️ 需要檢查")}");
    }

    [ContextMenu("測試 IK - 向前 0.3m")]
    void TestIKForward()
    {
        Vector3 basePos = joints[0].joint.transform.position;
        Vector3 target = basePos + transform.forward * 0.3f + Vector3.up * 0.2f;

        if (SolveIK(target, out float[] angles))
        {
            Debug.Log($"✅ 測試成功!\n角度: {string.Join(", ", System.Array.ConvertAll(angles, x => $"{x:F1}°"))}");
            ApplyJointAngles(angles);
        }
        else
        {
            Debug.LogWarning("⚠️ IK 求解未達最佳結果");
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

    [ContextMenu("重新初始化連桿偏移")]
    void ReinitializeLinkOffsets()
    {
        _linkOffsetsInitialized = false;
        InitializeLinkOffsets();
    }

    #endregion
}
```

## 🎯 **使用說明**

### **1. 設定關節**

在 Inspector 中按照 OpenArm 結構設定：
```
joints[0]: openarm_right_link1 (Shoulder Pitch)
joints[1]: openarm_right_link2 (Shoulder Roll)
joints[2]: openarm_right_link3 (Shoulder Yaw)
joints[3]: openarm_right_link4 (Elbow Pitch)
joints[4]: openarm_right_link5 (Wrist Roll)
joints[5]: openarm_right_link6 (Wrist Yaw)
joints[6]: openarm_right_link7 (Wrist Pitch)
```

### **2. 檢查 FK 準確度**

右鍵點擊組件 → "測試 FK 準確度"

- ✅ 誤差 < 1mm：非常好
- ✅ 誤差 < 10mm：可接受
- ⚠️ 誤差 > 10mm：需要檢查 `axis` 設定

### **3. 啟用 FK 調試視覺化**

勾選 `Debug FK`，場景中會顯示：
- 🟡 黃色球：實際物理位置
- 🟣 紫色球：FK 計算位置
- 藍線：兩者的差異

### **4. 調整參數**
```
maxIterations: 30-50 (增加以提高精度)
tolerance: 0.005-0.01 (5-10mm)
learningRate: 0.3-0.7 (降低以提高穩定性)