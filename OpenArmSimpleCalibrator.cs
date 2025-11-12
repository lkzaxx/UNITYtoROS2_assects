using System.Collections;
using UnityEngine;
using UnityEngine.InputSystem;

public class OpenArmSimpleCalibrator : MonoBehaviour
{
    [Header("References")]
    public OpenArmRetarget retarget;                 // 指到場景裡的 OpenArmRetarget
    public InputActionReference calibrateActionRight; // 指到 Input Actions 的右手校準按鍵
    public InputActionReference calibrateActionLeft;  // 指到 Input Actions 的左手校準按鍵

    [Header("Left Arm Targets (deg)")]
    public float[] desiredTargetsLeft = new float[7] { 90f, 0f, 0f, 0f, 0f, 0f, 0f };

    [Header("Right Arm Targets (deg)")]
    // 你的需求：openarm_right_link1 = +90，其餘 = 0
    public float[] desiredTargetsRight = new float[7] { 90f, 0f, 0f, 0f, 0f, 0f, 0f };

    [Header("Snap Settings")]
    [Tooltip("校準後，鎖定在目標角度的秒數（防止 Apply() 覆寫）。")]
    public float lockHoldSeconds = 0.5f;             // 鎖定時間，建議 0.3~1.0 秒

    Coroutine _lockCoLeft;
    Coroutine _lockCoRight;

    void OnEnable()
    {
        if (calibrateActionRight != null)
        {
            calibrateActionRight.action.Enable();
            calibrateActionRight.action.performed += OnCalibrateRightPerformed;
        }
        if (calibrateActionLeft != null)
        {
            calibrateActionLeft.action.Enable();
            calibrateActionLeft.action.performed += OnCalibrateLeftPerformed;
        }
    }

    void OnDisable()
    {
        if (calibrateActionRight != null)
        {
            calibrateActionRight.action.performed -= OnCalibrateRightPerformed;
            calibrateActionRight.action.Disable();
        }
        if (calibrateActionLeft != null)
        {
            calibrateActionLeft.action.performed -= OnCalibrateLeftPerformed;
            calibrateActionLeft.action.Disable();
        }
    }

    void OnCalibrateRightPerformed(InputAction.CallbackContext _)
    {
        CalibrateArm("right", retarget?.right, desiredTargetsRight, ref _lockCoRight);
    }

    void OnCalibrateLeftPerformed(InputAction.CallbackContext _)
    {
        CalibrateArm("left", retarget?.left, desiredTargetsLeft, ref _lockCoLeft);
    }

    void CalibrateArm(string side, OpenArmRetarget.JointMap[] joints, float[] desiredTargets, ref Coroutine lockCo)
    {
        if (retarget == null)
        {
            Debug.LogWarning($"[OpenArm] Calibrate {side}: retarget 未設定。");
            return;
        }

        if (joints == null || joints.Length < 7)
        {
            Debug.LogWarning($"[OpenArm] Calibrate {side}: 關節陣列未設定或長度不足。");
            return;
        }

        // 1) 先重置中性姿勢（讓當前姿勢成為新的 0 度參考點）
        for (int i = 0; i < 7; i++)
        {
            var j = joints[i];
            if (j == null) continue;
            j.CalibrateNeutral();
        }

        // 2) 計算每軸 offset 並鎖定關節
        for (int i = 0; i < 7; i++)
        {
            var j = joints[i];
            if (j == null) continue;

            float srcDeg = j.ReadSourceAngleDegRaw();
            float desiredDeg = (i < desiredTargets.Length) ? desiredTargets[i] : 0f;

            // offset = desired - scale*src ；此時 src ≈ 0，offset ≈ desired
            j.offsetDeg = desiredDeg - j.scale * srcDeg;

            // 立即鎖定在目標角度（防止 Apply() 覆寫）
            j.isLocked = true;
            j.lockedTarget = desiredDeg;

            // 立刻把驅動器目標也設為鎖定角度，避免本幀下落
            if (j.joint != null)
            {
                var drive = j.joint.xDrive;
                drive.target = desiredDeg;
                j.joint.xDrive = drive;
            }
        }

        // 3) 延遲一段時間後解鎖
        if (lockCo != null) StopCoroutine(lockCo);
        lockCo = StartCoroutine(UnlockAfterDelay(side, joints));

        Debug.Log($"[OpenArm] {side} 臂校準完成（已鎖定 " + lockHoldSeconds + " 秒）。");
    }

    IEnumerator UnlockAfterDelay(string side, OpenArmRetarget.JointMap[] joints)
    {
        yield return new WaitForSeconds(lockHoldSeconds);
        
        // 解鎖指定手臂的關節，讓 Apply() 恢復正常運作
        if (joints != null)
        {
            foreach (var j in joints)
            {
                if (j != null)
                {
                    j.isLocked = false;
                }
            }
        }
        
        Debug.Log($"[OpenArm] {side} 臂已解鎖，恢復正常控制。");
    }
}
