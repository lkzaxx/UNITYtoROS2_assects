using UnityEngine;
using UnityEngine.InputSystem;

public class GripperHoldToOpenPrismatic : MonoBehaviour
{
    public enum Axis { X, Y, Z }

    [Header("Input (Button)")]
    [Tooltip("綁你在 Input Actions 建好的 triggerPressed (OpenXR) / Button / Interactions: Press")]
    public InputActionReference gripperButton;

    [Header("Jaws (ArticulationBody)")]
    public ArticulationBody leftJaw;
    public ArticulationBody rightJaw;

    [Header("Prismatic Settings (meters)")]
    [Tooltip("關閉時的位置（通常對應 Lower limit）")]
    public float closePos = 0f;
    [Tooltip("打開時的位置（通常對應 Upper limit）")]
    public float openPos  = 0.044f; // 44 mm
    [Tooltip("夾爪滑動軸向")]
    public Axis axis = Axis.X;
    [Tooltip("右爪方向相反時勾選")]
    public bool invertRight = false;

    [Header("Drive")]
    public float stiffness = 6000f;
    public float damping   = 300f;
    public float forceLimit = 200f;

    [Header("動作手感")]
    [Tooltip("每秒移動距離（m/s）。0 = 立即到位")]
    public float slewMetersPerSec = 0.2f;

    float currentTarget; // 以「左爪」的目標作為基準（m）

    void OnEnable()
    {
        if (gripperButton) gripperButton.action.Enable();
        currentTarget = closePos;
    }

    void OnDisable()
    {
        if (gripperButton) gripperButton.action.Disable();
    }

    void Update()
    {
        bool pressed = gripperButton && gripperButton.action.IsPressed();

        float desired = pressed ? openPos : closePos;
        desired = Mathf.Clamp(desired, Mathf.Min(closePos, openPos), Mathf.Max(closePos, openPos));

        if (slewMetersPerSec > 0f)
        {
            float step = slewMetersPerSec * Time.deltaTime;
            currentTarget = Mathf.MoveTowards(currentTarget, desired, step);
        }
        else
        {
            currentTarget = desired;
        }

        // 左右爪各自套目標（右爪可反向）
        if (leftJaw)  ApplyLinearTarget(leftJaw, currentTarget);
        if (rightJaw)
        {
            float t01 = Mathf.InverseLerp(closePos, openPos, currentTarget);
            float rightTarget = invertRight
                ? Mathf.Lerp(openPos, closePos, t01)   // 反向
                : Mathf.Lerp(closePos, openPos, t01);  // 同向
            ApplyLinearTarget(rightJaw, rightTarget);
        }
    }

    void ApplyLinearTarget(ArticulationBody ab, float targetMeters)
    {
        // 依軸取對應 Drive，並套參數與 target（單位：m）
        switch (axis)
        {
            case Axis.X:
                var dx = ab.xDrive;
                dx.stiffness = stiffness;
                dx.damping = damping;
                dx.forceLimit = forceLimit;
                dx.target = targetMeters;
                ab.xDrive = dx;
                break;

            case Axis.Y:
                var dy = ab.yDrive;
                dy.stiffness = stiffness;
                dy.damping = damping;
                dy.forceLimit = forceLimit;
                dy.target = targetMeters;
                ab.yDrive = dy;
                break;

            case Axis.Z:
                var dz = ab.zDrive;
                dz.stiffness = stiffness;
                dz.damping = damping;
                dz.forceLimit = forceLimit;
                dz.target = targetMeters;
                ab.zDrive = dz;
                break;
        }
    }
}
