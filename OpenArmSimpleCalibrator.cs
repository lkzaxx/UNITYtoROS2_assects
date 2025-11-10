using System.Collections;
using UnityEngine;
using UnityEngine.InputSystem;

public class OpenArmSimpleCalibrator : MonoBehaviour
{
    [Header("References")]
    public OpenArmRetarget retarget;                 // 指到場景裡的 OpenArmRetarget
    public InputActionReference calibrateAction;     // 指到 Input Actions 的 Calibrate（Grip+Hold）

    [Header("Right Arm Targets (deg)")]
    // 你的需求：openarm_right_link1 = +90，其餘 = 0
    public float[] desiredTargetsRight = new float[7] { 90f, 0f, 0f, 0f, 0f, 0f, 0f };

    [Header("Snap Settings")]
    [Tooltip("校準後，鎖定在目標角度的秒數（防止 Apply() 覆寫）。")]
    public float lockHoldSeconds = 0.5f;             // 鎖定時間，建議 0.3~1.0 秒

    Coroutine _lockCo;

    void OnEnable()
    {
        if (calibrateAction != null)
        {
            calibrateAction.action.Enable();
            calibrateAction.action.performed += OnCalibratePerformed;
        }
    }

    void OnDisable()
    {
        if (calibrateAction != null)
        {
            calibrateAction.action.performed -= OnCalibratePerformed;
            calibrateAction.action.Disable();
        }
    }

    void OnCalibratePerformed(InputAction.CallbackContext _)
    {
        if (retarget == null || retarget.right == null || retarget.right.Length < 7)
        {
            Debug.LogWarning("[OpenArm] Calibrate: retarget/right 未設定或長度不足。");
            return;
        }

        // 1) 先重置中性姿勢（讓當前姿勢成為新的 0 度參考點）
        for (int i = 0; i < 7; i++)
        {
            var j = retarget.right[i];
            if (j == null) continue;
            
            // 記錄當前姿勢為中性姿勢
            j.CalibrateNeutral();
        }

        // 2) 計算每軸 offset 並鎖定關節
        for (int i = 0; i < 7; i++)
        {
            var j = retarget.right[i];
            if (j == null) continue;

            // 因為剛重置了 neutral，srcDeg 現在會是 0 或接近 0
            // 使用 JointMap 的方法讀取（會考慮 useNeutralCalibration）
            float srcDeg = j.ReadSourceAngleDegRaw();
            float desiredDeg = (i < desiredTargetsRight.Length) ? desiredTargetsRight[i] : 0f;

            // mapped = offset + scale*src  =>  offset = desired - scale*src
            // 由於 srcDeg ≈ 0，所以 offset ≈ desired
            j.offsetDeg = desiredDeg - j.scale * srcDeg;
            
            // 立即鎖定在目標角度（防止 Apply() 覆寫）
            j.isLocked = true;
            j.lockedTarget = desiredDeg;
        }

        // 2) 延遲一段時間後解鎖
        if (_lockCo != null) StopCoroutine(_lockCo);
        _lockCo = StartCoroutine(UnlockAfterDelay());

        Debug.Log("[OpenArm] 右臂校準完成（已鎖定 " + lockHoldSeconds + " 秒）。");
    }

    IEnumerator UnlockAfterDelay()
    {
        yield return new WaitForSeconds(lockHoldSeconds);
        
        // 解鎖所有關節，讓 Apply() 恢復正常運作
        if (retarget != null && retarget.right != null)
        {
            foreach (var j in retarget.right)
            {
                if (j != null)
                {
                    j.isLocked = false;
                }
            }
        }
        
        Debug.Log("[OpenArm] 右臂已解鎖，恢復正常控制。");
    }
}
