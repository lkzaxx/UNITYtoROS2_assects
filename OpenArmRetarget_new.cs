using System;
using UnityEngine;
using System.Collections.Generic;

public class OpenArmHumanoidRetarget : MonoBehaviour
{
    [Serializable]
    public class OpenArmJoint
    {
        [Header("Joint Info")]
        public string jointName;              // e.g., "joint_1", "joint_2"
        public ArticulationBody articulation; // OpenArm 關節
        public Transform humanoidBone;        // 對應的人形骨骼

        [Header("OpenArm Joint Limits (from DH params)")]
        public float minAngleDeg = -180f;
        public float maxAngleDeg = 180f;

        [Header("Mapping Configuration")]
        public MappingMode mode = MappingMode.DirectQuaternion;
        public Vector3 axisMapping = Vector3.one;     // 軸向映射權重
        public Vector3 rotationOffset = Vector3.zero; // 初始旋轉偏移
        public float responseSpeed = 5f;              // 響應速度 (1-10)

        [Header("Filtering")]
        public bool useSmoothing = true;
        public float smoothingFactor = 0.15f;
        public float deadZone = 0.5f;

        // 內部狀態
        private Quaternion targetRotation;
        private Quaternion smoothedRotation;
        private Vector3 previousEuler;
        private bool initialized = false;

        public enum MappingMode
        {
            DirectQuaternion,    // 直接四元數映射
            EulerMapping,       // Euler角映射
            VelocityBased      // 基於速度的映射
        }

        public void Initialize()
        {
            if (humanoidBone != null && articulation != null)
            {
                targetRotation = humanoidBone.localRotation;
                smoothedRotation = targetRotation;
                previousEuler = humanoidBone.localEulerAngles;
                initialized = true;
            }
        }

        public void UpdateMapping()
        {
            if (!initialized || humanoidBone == null || articulation == null)
                return;

            switch (mode)
            {
                case MappingMode.DirectQuaternion:
                    MapQuaternion();
                    break;
                case MappingMode.EulerMapping:
                    MapEuler();
                    break;
                case MappingMode.VelocityBased:
                    MapVelocity();
                    break;
            }
        }

        private void MapQuaternion()
        {
            // 獲取人形骨骼的旋轉
            Quaternion humanRotation = humanoidBone.localRotation;

            // 應用旋轉偏移
            Quaternion offsetQuat = Quaternion.Euler(rotationOffset);
            humanRotation = offsetQuat * humanRotation;

            // 轉換為Euler角進行軸向映射
            Vector3 euler = humanRotation.eulerAngles;
            euler.x = Mathf.DeltaAngle(0, euler.x) * axisMapping.x;
            euler.y = Mathf.DeltaAngle(0, euler.y) * axisMapping.y;
            euler.z = Mathf.DeltaAngle(0, euler.z) * axisMapping.z;

            // 應用死區
            if (Mathf.Abs(euler.x) < deadZone) euler.x = 0;
            if (Mathf.Abs(euler.y) < deadZone) euler.y = 0;
            if (Mathf.Abs(euler.z) < deadZone) euler.z = 0;

            targetRotation = Quaternion.Euler(euler);

            // 平滑處理
            if (useSmoothing)
            {
                smoothedRotation = Quaternion.Slerp(
                    smoothedRotation,
                    targetRotation,
                    smoothingFactor * responseSpeed * Time.fixedDeltaTime
                );
            }
            else
            {
                smoothedRotation = targetRotation;
            }

            ApplyToArticulation(smoothedRotation);
        }

        private void MapEuler()
        {
            Vector3 currentEuler = humanoidBone.localEulerAngles;

            // 轉換到-180到180範圍
            currentEuler.x = Mathf.DeltaAngle(0, currentEuler.x);
            currentEuler.y = Mathf.DeltaAngle(0, currentEuler.y);
            currentEuler.z = Mathf.DeltaAngle(0, currentEuler.z);

            // 應用偏移和映射
            currentEuler += rotationOffset;
            currentEuler = Vector3.Scale(currentEuler, axisMapping);

            // 應用死區
            if (Mathf.Abs(currentEuler.x) < deadZone) currentEuler.x = 0;
            if (Mathf.Abs(currentEuler.y) < deadZone) currentEuler.y = 0;
            if (Mathf.Abs(currentEuler.z) < deadZone) currentEuler.z = 0;

            // 平滑處理
            if (useSmoothing)
            {
                currentEuler = Vector3.Lerp(
                    previousEuler,
                    currentEuler,
                    smoothingFactor * responseSpeed * Time.fixedDeltaTime
                );
            }

            previousEuler = currentEuler;
            ApplyEulerToArticulation(currentEuler);
        }

        private void MapVelocity()
        {
            // 計算角速度
            Quaternion deltaRotation = humanoidBone.localRotation * Quaternion.Inverse(targetRotation);
            targetRotation = humanoidBone.localRotation;

            // 轉換為角速度
            deltaRotation.ToAngleAxis(out float angle, out Vector3 axis);
            if (angle > 180) angle -= 360;

            Vector3 angularVelocity = axis * angle * Mathf.Deg2Rad / Time.fixedDeltaTime;
            angularVelocity = Vector3.Scale(angularVelocity, axisMapping);

            // 應用到ArticulationBody
            if (articulation.jointType == ArticulationJointType.SphericalJoint)
            {
                var drive = articulation.xDrive;
                drive.targetVelocity = angularVelocity.x * Mathf.Rad2Deg * responseSpeed;
                articulation.xDrive = drive;

                drive = articulation.yDrive;
                drive.targetVelocity = angularVelocity.y * Mathf.Rad2Deg * responseSpeed;
                articulation.yDrive = drive;

                drive = articulation.zDrive;
                drive.targetVelocity = angularVelocity.z * Mathf.Rad2Deg * responseSpeed;
                articulation.zDrive = drive;
            }
        }

        private void ApplyToArticulation(Quaternion rotation)
        {
            Vector3 euler = rotation.eulerAngles;
            ApplyEulerToArticulation(euler);
        }

        private void ApplyEulerToArticulation(Vector3 euler)
        {
            // 根據關節類型設定驅動
            if (articulation.jointType == ArticulationJointType.RevoluteJoint)
            {
                // 單軸旋轉關節：選擇主要軸
                float targetAngle = GetPrimaryAxis(euler);
                targetAngle = Mathf.Clamp(targetAngle, minAngleDeg, maxAngleDeg);

                var drive = articulation.xDrive;
                drive.target = targetAngle;
                articulation.xDrive = drive;
            }
            else if (articulation.jointType == ArticulationJointType.SphericalJoint)
            {
                // 球形關節：設定三個軸
                euler.x = Mathf.Clamp(euler.x, minAngleDeg, maxAngleDeg);
                euler.y = Mathf.Clamp(euler.y, minAngleDeg, maxAngleDeg);
                euler.z = Mathf.Clamp(euler.z, minAngleDeg, maxAngleDeg);

                var drive = articulation.xDrive;
                drive.target = euler.x;
                articulation.xDrive = drive;

                drive = articulation.yDrive;
                drive.target = euler.y;
                articulation.yDrive = drive;

                drive = articulation.zDrive;
                drive.target = euler.z;
                articulation.zDrive = drive;
            }
        }

        private float GetPrimaryAxis(Vector3 euler)
        {
            // 根據軸映射權重選擇主要軸
            float maxWeight = Mathf.Max(Mathf.Abs(axisMapping.x),
                                       Mathf.Abs(axisMapping.y),
                                       Mathf.Abs(axisMapping.z));

            if (Mathf.Abs(axisMapping.x) >= maxWeight) return euler.x;
            if (Mathf.Abs(axisMapping.y) >= maxWeight) return euler.y;
            return euler.z;
        }
    }

    [Header("OpenArm Configuration")]
    [Tooltip("根據OpenArm DH參數設定")]
    public OpenArmJoint[] leftArmJoints = new OpenArmJoint[7];
    public OpenArmJoint[] rightArmJoints = new OpenArmJoint[7];

    [Header("Global Settings")]
    public bool enableRetargeting = true;
    public UpdateMode updateMode = UpdateMode.FixedUpdate;
    public float globalResponseMultiplier = 1f;

    [Header("Drive Settings")]
    [Tooltip("OpenArm標準驅動參數")]
    public float defaultStiffness = 2000f;
    public float defaultDamping = 150f;
    public float defaultForceLimit = 1000f;

    [Header("Calibration")]
    public bool autoCalibrate = true;
    public KeyCode calibrateKey = KeyCode.C;
    public KeyCode resetKey = KeyCode.R;

    [Header("Debug")]
    public bool showDebugInfo = false;
    public bool visualizeJointAxes = false;

    public enum UpdateMode
    {
        Update,
        FixedUpdate,
        LateUpdate
    }

    // OpenArm標準關節限制（根據DH參數）
    private readonly float[,] openArmJointLimits = new float[7, 2]
    {
        {-180f, 180f},    // Joint 1: Base rotation
        {-90f, 90f},      // Joint 2: Shoulder pitch  
        {-180f, 180f},    // Joint 3: Shoulder roll
        {-180f, 0f},      // Joint 4: Elbow pitch
        {-180f, 180f},    // Joint 5: Wrist pitch
        {-90f, 90f},      // Joint 6: Wrist roll
        {-180f, 180f}     // Joint 7: Wrist yaw
    };

    void Start()
    {
        InitializeJoints();
        SetupDriveParameters();

        if (autoCalibrate)
        {
            CalibrateAllJoints();
        }
    }

    void InitializeJoints()
    {
        // 初始化左臂關節
        InitializeArmJoints(leftArmJoints, "Left");

        // 初始化右臂關節
        InitializeArmJoints(rightArmJoints, "Right");
    }

    void InitializeArmJoints(OpenArmJoint[] joints, string side)
    {
        for (int i = 0; i < joints.Length && i < 7; i++)
        {
            if (joints[i] != null)
            {
                joints[i].jointName = $"{side}_Joint_{i + 1}";
                joints[i].minAngleDeg = openArmJointLimits[i, 0];
                joints[i].maxAngleDeg = openArmJointLimits[i, 1];
                joints[i].Initialize();
            }
        }
    }

    void SetupDriveParameters()
    {
        SetupArmDrives(leftArmJoints);
        SetupArmDrives(rightArmJoints);
    }

    void SetupArmDrives(OpenArmJoint[] joints)
    {
        foreach (var joint in joints)
        {
            if (joint?.articulation != null)
            {
                // 設定驅動參數
                var drive = joint.articulation.xDrive;
                drive.stiffness = defaultStiffness;
                drive.damping = defaultDamping;
                drive.forceLimit = defaultForceLimit;
                joint.articulation.xDrive = drive;

                if (joint.articulation.jointType == ArticulationJointType.SphericalJoint)
                {
                    joint.articulation.yDrive = drive;
                    joint.articulation.zDrive = drive;
                }
            }
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(calibrateKey))
        {
            CalibrateAllJoints();
            Debug.Log("OpenArm calibrated to current pose");
        }

        if (Input.GetKeyDown(resetKey))
        {
            ResetToHome();
            Debug.Log("OpenArm reset to home position");
        }

        if (updateMode == UpdateMode.Update && enableRetargeting)
        {
            UpdateRetargeting();
        }
    }

    void FixedUpdate()
    {
        if (updateMode == UpdateMode.FixedUpdate && enableRetargeting)
        {
            UpdateRetargeting();
        }
    }

    void LateUpdate()
    {
        if (updateMode == UpdateMode.LateUpdate && enableRetargeting)
        {
            UpdateRetargeting();
        }
    }

    void UpdateRetargeting()
    {
        // 更新左臂
        foreach (var joint in leftArmJoints)
        {
            joint?.UpdateMapping();
        }

        // 更新右臂
        foreach (var joint in rightArmJoints)
        {
            joint?.UpdateMapping();
        }
    }

    void CalibrateAllJoints()
    {
        CalibrateArm(leftArmJoints);
        CalibrateArm(rightArmJoints);
    }

    void CalibrateArm(OpenArmJoint[] joints)
    {
        foreach (var joint in joints)
        {
            if (joint != null && joint.humanoidBone != null)
            {
                // 記錄當前姿勢作為參考
                Vector3 currentEuler = joint.humanoidBone.localEulerAngles;
                currentEuler.x = Mathf.DeltaAngle(0, currentEuler.x);
                currentEuler.y = Mathf.DeltaAngle(0, currentEuler.y);
                currentEuler.z = Mathf.DeltaAngle(0, currentEuler.z);

                // 設定偏移使當前姿勢映射到零點
                joint.rotationOffset = -currentEuler;
                joint.Initialize();
            }
        }
    }

    void ResetToHome()
    {
        ResetArm(leftArmJoints);
        ResetArm(rightArmJoints);
    }

    void ResetArm(OpenArmJoint[] joints)
    {
        foreach (var joint in joints)
        {
            if (joint?.articulation != null)
            {
                var drive = joint.articulation.xDrive;
                drive.target = 0;
                joint.articulation.xDrive = drive;

                if (joint.articulation.jointType == ArticulationJointType.SphericalJoint)
                {
                    joint.articulation.yDrive = drive;
                    joint.articulation.zDrive = drive;
                }
            }
        }
    }

    void OnDrawGizmos()
    {
        if (!showDebugInfo) return;

        DrawArmGizmos(leftArmJoints, Color.red);
        DrawArmGizmos(rightArmJoints, Color.blue);
    }

    void DrawArmGizmos(OpenArmJoint[] joints, Color color)
    {
        Gizmos.color = color;

        foreach (var joint in joints)
        {
            if (joint?.articulation != null)
            {
                Gizmos.DrawWireSphere(joint.articulation.transform.position, 0.02f);

                if (visualizeJointAxes)
                {
                    // 繪製關節軸
                    DrawAxis(joint.articulation.transform);
                }
            }
        }
    }

    void DrawAxis(Transform t)
    {
        float length = 0.1f;
        Gizmos.color = Color.red;
        Gizmos.DrawLine(t.position, t.position + t.right * length);
        Gizmos.color = Color.green;
        Gizmos.DrawLine(t.position, t.position + t.up * length);
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(t.position, t.position + t.forward * length);
    }
}