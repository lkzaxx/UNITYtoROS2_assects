using System;
using UnityEngine;

public class OpenArmRetarget : MonoBehaviour
{
    public enum Axis { X, Y, Z }

    [Serializable]
    public class JointMap
    {
        public string nameHint;
        public ArticulationBody joint;
        [Header("Source (Humanoid bone)")]
        public Transform source;          // 來源骨骼（上臂/前臂/手腕）
        public Axis sourceAxis = Axis.X;  // 取該骨骼的哪一個 local Euler 軸
        [Header("Mapping")]
        public float scale = 1f;          // 角度比例（可用 -1 反向）
        public float offsetDeg = 0f;      // 角度偏移（度）
        public float minDeg = -180f;
        public float maxDeg = 180f;

        [Header("Drive")]
        public float stiffness = 1500f;
        public float damping = 100f;
        public float forceLimit = 1000f;

        public float ReadSourceAngleDeg()
        {
            if (source == null) return 0f;
            var e = source.localEulerAngles;
            // 轉成 -180..180
            float sx = Mathf.DeltaAngle(0, e.x);
            float sy = Mathf.DeltaAngle(0, e.y);
            float sz = Mathf.DeltaAngle(0, e.z);
            switch (sourceAxis)
            {
                case Axis.X: return sx;
                case Axis.Y: return sy;
                default:     return sz;
            }
        }

        public void Apply()
        {
            if (joint == null) return;

            var drive = joint.xDrive; // 對於 Revolute，使用 xDrive
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;

            float src = ReadSourceAngleDeg();
            float targetDeg = Mathf.Clamp(offsetDeg + scale * src, minDeg, maxDeg);

            drive.target = targetDeg; // ArticulationDrive.target 使用「度」
            joint.xDrive = drive;
        }
    }

    [Header("Left arm joints (1..7)")]
    public JointMap[] left = new JointMap[7];

    [Header("Right arm joints (1..7)")]
    public JointMap[] right = new JointMap[7];

    void Update()
    {
        if (left != null)  foreach (var j in left)  j?.Apply();
        if (right != null) foreach (var j in right) j?.Apply();
    }
}
