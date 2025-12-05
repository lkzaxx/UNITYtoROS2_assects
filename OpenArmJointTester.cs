using System;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// OpenArm 關節方向測試工具
/// 用於測試每個關節在給定正值時的實際運動方向
/// 配合 JOINT_LIMITS_FIX_PROPOSAL.md 驗證左右手是否需要鏡像
/// </summary>
public class OpenArmJointTester : MonoBehaviour
{
    [Serializable]
    public class ArmJoints
    {
        public string armName = "Left";
        public ArticulationBody[] joints = new ArticulationBody[7]; // Joint 1-7
    }

    [Header("手臂關節配置")]
    public ArmJoints leftArm = new ArmJoints { armName = "Left" };
    public ArmJoints rightArm = new ArmJoints { armName = "Right" };

    [Header("測試參數")]
    [Tooltip("測試用的正值角度（度）")]
    public float testAngleDeg = 45f;
    
    [Tooltip("移動到測試角度的時間（秒）")]
    public float moveTime = 2f;

    [Header("驅動參數")]
    public float stiffness = 4000f;
    public float damping = 300f;
    public float forceLimit = 10000f;

    [Header("UI 顯示（可選）")]
    public Text statusText;

    // 當前測試狀態
    private int currentJointIndex = -1; // -1 表示回零位
    private ArmJoints currentArm = null;
    private string lastAction = "Ready";

    void Start()
    {
        // 初始化所有關節驅動器參數
        InitializeJointDrives(leftArm);
        InitializeJointDrives(rightArm);

        UpdateStatusText();
        
        Debug.Log("=== OpenArm 關節測試工具 ===");
        Debug.Log("按鍵說明：");
        Debug.Log("數字鍵 1-7: 測試對應關節的正值");
        Debug.Log("0: 所有關節回零");
        Debug.Log("L: 切換到左手測試");
        Debug.Log("R: 切換到右手測試");
        Debug.Log("Space: 當前關節回零");
        Debug.Log("========================");
    }

    void Update()
    {
        // 數字鍵 1-7：測試對應關節
        for (int i = 1; i <= 7; i++)
        {
            if (Input.GetKeyDown(KeyCode.Alpha0 + i))
            {
                TestJoint(currentArm ?? leftArm, i - 1);
            }
        }

        // 0：回零所有關節
        if (Input.GetKeyDown(KeyCode.Alpha0) || Input.GetKeyDown(KeyCode.Keypad0))
        {
            ResetAllJoints(currentArm ?? leftArm);
        }

        // L：選擇左手
        if (Input.GetKeyDown(KeyCode.L))
        {
            currentArm = leftArm;
            lastAction = $"切換到左手測試";
            UpdateStatusText();
        }

        // R：選擇右手
        if (Input.GetKeyDown(KeyCode.R))
        {
            currentArm = rightArm;
            lastAction = $"切換到右手測試";
            UpdateStatusText();
        }

        // Space：當前關節回零
        if (Input.GetKeyDown(KeyCode.Space))
        {
            if (currentJointIndex >= 0 && currentArm != null)
            {
                SetJointAngle(currentArm, currentJointIndex, 0f);
                lastAction = $"{currentArm.armName} Joint {currentJointIndex + 1} 回零";
                UpdateStatusText();
            }
        }

        // Q：測試所有關節正值（演示模式）
        if (Input.GetKeyDown(KeyCode.Q))
        {
            TestAllJointsPositive(currentArm ?? leftArm);
        }

        // ESC：停止所有測試，回零
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            ResetAllJoints(leftArm);
            ResetAllJoints(rightArm);
            lastAction = "所有關節回零";
            UpdateStatusText();
        }
    }

    /// <summary>
    /// 初始化關節驅動器
    /// </summary>
    void InitializeJointDrives(ArmJoints arm)
    {
        if (arm == null || arm.joints == null) return;

        foreach (var joint in arm.joints)
        {
            if (joint == null) continue;

            var drive = joint.xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;
            joint.xDrive = drive;
        }
    }

    /// <summary>
    /// 測試單一關節
    /// </summary>
    public void TestJoint(ArmJoints arm, int jointIndex)
    {
        if (arm == null || arm.joints == null || jointIndex < 0 || jointIndex >= 7)
        {
            Debug.LogError($"無效的關節索引: {jointIndex}");
            return;
        }

        // 先將所有關節回零
        ResetAllJoints(arm);

        // 設置目標關節到測試角度
        SetJointAngle(arm, jointIndex, testAngleDeg);

        currentJointIndex = jointIndex;
        currentArm = arm;
        lastAction = $"{arm.armName} Joint {jointIndex + 1} → +{testAngleDeg}°";
        
        Debug.Log($"=== 測試 {arm.armName} Joint {jointIndex + 1} ===");
        Debug.Log($"設定角度: +{testAngleDeg}°");
        Debug.Log($"請觀察運動方向並記錄");
        
        UpdateStatusText();
    }

    /// <summary>
    /// 測試所有關節正值（依序執行）
    /// </summary>
    public void TestAllJointsPositive(ArmJoints arm)
    {
        if (arm == null) return;
        
        StartCoroutine(TestAllJointsSequence(arm));
    }

    System.Collections.IEnumerator TestAllJointsSequence(ArmJoints arm)
    {
        Debug.Log($"=== 開始測試 {arm.armName} 所有關節 ===");
        
        for (int i = 0; i < 7; i++)
        {
            // 回零
            ResetAllJoints(arm);
            yield return new WaitForSeconds(moveTime);

            // 測試關節
            TestJoint(arm, i);
            Debug.Log($"Joint {i + 1} 運動方向: __________（請記錄）");
            yield return new WaitForSeconds(moveTime * 2);
        }

        // 最後回零
        ResetAllJoints(arm);
        Debug.Log($"=== {arm.armName} 測試完成 ===");
    }

    /// <summary>
    /// 設置關節角度
    /// </summary>
    void SetJointAngle(ArmJoints arm, int jointIndex, float angleDeg)
    {
        if (arm == null || arm.joints == null || jointIndex < 0 || jointIndex >= 7)
            return;

        var joint = arm.joints[jointIndex];
        if (joint == null) return;

        var drive = joint.xDrive;
        drive.target = angleDeg;
        joint.xDrive = drive;

        Debug.Log($"{arm.armName} Joint {jointIndex + 1} 設定為 {angleDeg}°");
    }

    /// <summary>
    /// 重置所有關節到零位
    /// </summary>
    public void ResetAllJoints(ArmJoints arm)
    {
        if (arm == null || arm.joints == null) return;

        for (int i = 0; i < arm.joints.Length; i++)
        {
            SetJointAngle(arm, i, 0f);
        }

        Debug.Log($"{arm.armName} 所有關節回零");
        lastAction = $"{arm.armName} 所有關節回零";
        UpdateStatusText();
    }

    /// <summary>
    /// 更新 UI 狀態文字
    /// </summary>
    void UpdateStatusText()
    {
        if (statusText == null) return;

        string armName = currentArm != null ? currentArm.armName : "未選擇";
        string jointInfo = currentJointIndex >= 0 ? $"Joint {currentJointIndex + 1}" : "無";
        
        statusText.text = $"當前手臂: {armName}\n" +
                         $"當前關節: {jointInfo}\n" +
                         $"測試角度: +{testAngleDeg}°\n" +
                         $"最後操作: {lastAction}\n\n" +
                         $"按鍵:\n" +
                         $"1-7: 測試關節\n" +
                         $"0: 回零\n" +
                         $"L/R: 切換左右手\n" +
                         $"Q: 自動測試所有關節\n" +
                         $"Space: 當前關節回零\n" +
                         $"ESC: 停止並回零";
    }

    // === UI 按鈕方法（可選，用於 Unity UI Button） ===

    public void UI_TestLeftJoint(int jointIndex)
    {
        TestJoint(leftArm, jointIndex);
    }

    public void UI_TestRightJoint(int jointIndex)
    {
        TestJoint(rightArm, jointIndex);
    }

    public void UI_ResetLeftArm()
    {
        ResetAllJoints(leftArm);
    }

    public void UI_ResetRightArm()
    {
        ResetAllJoints(rightArm);
    }

    public void UI_ResetBothArms()
    {
        ResetAllJoints(leftArm);
        ResetAllJoints(rightArm);
    }

    public void UI_TestAllLeft()
    {
        TestAllJointsPositive(leftArm);
    }

    public void UI_TestAllRight()
    {
        TestAllJointsPositive(rightArm);
    }

    public void UI_SetTestAngle(float angle)
    {
        testAngleDeg = angle;
        UpdateStatusText();
    }

    // === 檢視器輔助 ===

    void OnGUI()
    {
        if (statusText != null) return; // 如果有 UI Text 就不顯示 OnGUI

        GUILayout.BeginArea(new Rect(10, 10, 400, 400));
        GUILayout.BeginVertical("box");

        GUILayout.Label("=== OpenArm 關節測試工具 ===", new GUIStyle(GUI.skin.label) { fontSize = 16, fontStyle = FontStyle.Bold });
        
        GUILayout.Space(10);
        
        GUILayout.Label($"當前手臂: {(currentArm != null ? currentArm.armName : "未選擇")}");
        GUILayout.Label($"測試角度: +{testAngleDeg}°");
        GUILayout.Label($"最後操作: {lastAction}");

        GUILayout.Space(10);

        // 左手按鈕
        GUILayout.Label("左手:", new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold });
        GUILayout.BeginHorizontal();
        for (int i = 0; i < 7; i++)
        {
            if (GUILayout.Button($"J{i + 1}", GUILayout.Width(45)))
            {
                TestJoint(leftArm, i);
            }
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(5);

        // 右手按鈕
        GUILayout.Label("右手:", new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold });
        GUILayout.BeginHorizontal();
        for (int i = 0; i < 7; i++)
        {
            if (GUILayout.Button($"J{i + 1}", GUILayout.Width(45)))
            {
                TestJoint(rightArm, i);
            }
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(10);

        // 控制按鈕
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("左手回零"))
        {
            ResetAllJoints(leftArm);
        }
        if (GUILayout.Button("右手回零"))
        {
            ResetAllJoints(rightArm);
        }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("測試左手全部"))
        {
            TestAllJointsPositive(leftArm);
        }
        if (GUILayout.Button("測試右手全部"))
        {
            TestAllJointsPositive(rightArm);
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(5);

        // 角度調整
        GUILayout.BeginHorizontal();
        GUILayout.Label($"測試角度: {testAngleDeg:F1}°", GUILayout.Width(120));
        testAngleDeg = GUILayout.HorizontalSlider(testAngleDeg, 0f, 90f, GUILayout.Width(200));
        GUILayout.EndHorizontal();

        GUILayout.EndVertical();
        GUILayout.EndArea();
    }
}

