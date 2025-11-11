// OpenArmIKAutoScaler.cs
using UnityEngine;
using UnityEngine.InputSystem;

public class OpenArmIKAutoScaler : MonoBehaviour
{
    [Header("Targets")]
    public OpenArmRetargetIK retargetIK;          // 指到場景中的 OpenArmRetargetIK
    public OpenArmIK leftIKSolver;                // 左臂 OpenArmIK
    public OpenArmIK rightIKSolver;               // 右臂 OpenArmIK

    [Header("Optional Input")]
    public InputActionReference calibrateAction;  // 例如 Grip+Trigger

    [Header("Calibrate Options")]
    [Tooltip("是否同時校準左右手臂（若其中一側未設定會自動跳過）")]
    public bool calibrateBothArms = true;

    void OnEnable()
    {
        if (calibrateAction != null)
        {
            calibrateAction.action.Enable();
            calibrateAction.action.performed += OnCalibrate;
        }
    }

    void OnDisable()
    {
        if (calibrateAction != null)
        {
            calibrateAction.action.performed -= OnCalibrate;
            calibrateAction.action.Disable();
        }
    }

    [ContextMenu("Calibrate Now")]
    public void CalibrateNow()
    {
        OnCalibrate(default);
    }

    void OnCalibrate(InputAction.CallbackContext _)
    {
        if (retargetIK == null)
        {
            Debug.LogWarning("OpenArmIKAutoScaler: retargetIK 未設定");
            return;
        }

        if (calibrateBothArms)
        {
            TryCalibrateSide("Left", retargetIK.leftIK, leftIKSolver);
            TryCalibrateSide("Right", retargetIK.rightIK, rightIKSolver);
        }
        else
        {
            // 只校準有填齊的一側（右優先）
            if (!TryCalibrateSide("Right", retargetIK.rightIK, rightIKSolver))
                TryCalibrateSide("Left", retargetIK.leftIK, leftIKSolver);
        }
    }

    bool TryCalibrateSide(string tag, OpenArmRetargetIK.ArmIKConfig arm, OpenArmIK solver)
    {
        if (arm == null || solver == null)
        {
            Debug.LogWarning($"OpenArmIKAutoScaler[{tag}]: Arm 或 Solver 未設定，略過");
            return false;
        }
        if (arm.shoulderReference == null || arm.wristTarget == null)
        {
            Debug.LogWarning($"OpenArmIKAutoScaler[{tag}]: 請設定 shoulderReference 與 wristTarget");
            return false;
        }
        if (solver.joints == null || solver.joints.Length == 0 || solver.joints[0].joint == null)
        {
            Debug.LogWarning($"OpenArmIKAutoScaler[{tag}]: IK joints 未設定或不完整");
            return false;
        }

        // 1) 人體「肩->腕」直線長度（請在伸直手臂狀態按下校準）
        float humanLen = Vector3.Distance(arm.shoulderReference.position, arm.wristTarget.position);

        // 2) 機械臂關節鏈總長（關節序列相鄰 Transform 的距離總和 + 最後一節到 endEffector）
        float robotLen = ComputeRobotChainLength(solver);

        if (humanLen <= 1e-4f || robotLen <= 1e-4f)
        {
            Debug.LogWarning($"OpenArmIKAutoScaler[{tag}]: humanLen 或 robotLen 無效 human={humanLen:F3} robot={robotLen:F3}");
            return false;
        }

        // 3) 等比縮放係數（各軸同值）
        float scale = robotLen / humanLen;
        arm.positionScale = new Vector3(scale, scale, scale);

        Debug.Log($"✅ OpenArmIKAutoScaler[{tag}] 校準完成 | HumanLen={humanLen:F3}m, RobotLen={robotLen:F3}m, Scale={scale:F3}");
        return true;
    }

    float ComputeRobotChainLength(OpenArmIK solver)
    {
        float total = 0f;

        // joints[0]..joints[n-1] 的相鄰距離總和
        for (int i = 0; i < solver.joints.Length - 1; i++)
        {
            var a = solver.joints[i]?.joint?.transform;
            var b = solver.joints[i + 1]?.joint?.transform;
            if (a != null && b != null)
                total += Vector3.Distance(a.position, b.position);
        }

        // 最後一關節到 endEffector 的距離（若 endEffector 有設定）
        var last = solver.joints.Length > 0 ? solver.joints[solver.joints.Length - 1]?.joint?.transform : null;
        if (last != null && solver.endEffector != null)
            total += Vector3.Distance(last.position, solver.endEffector.position);

        return total;
    }
}