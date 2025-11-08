using UnityEngine;

public class BodyBoneBinder : MonoBehaviour
{
    [Header("RealisticCharacterMirrored 根物件")]
    public Animator bodyAnimator;

    [Header("Targets（放在 openarm 底下的 4 個空物件）")]
    public CopyTransform leftElbowTarget, leftWristTarget;
    public CopyTransform rightElbowTarget, rightWristTarget;

    void Reset()
    {
        bodyAnimator = FindObjectOfType<Animator>();
    }

    void Start()
    {
        // 取常用的人體骨頭（Humanoid Avatar 必須是 Valid）
        var LLowerArm = bodyAnimator.GetBoneTransform(HumanBodyBones.LeftLowerArm);
        var LHand     = bodyAnimator.GetBoneTransform(HumanBodyBones.LeftHand);
        var RLowerArm = bodyAnimator.GetBoneTransform(HumanBodyBones.RightLowerArm);
        var RHand     = bodyAnimator.GetBoneTransform(HumanBodyBones.RightHand);

        leftElbowTarget.source  = LLowerArm;  // 左手肘
        leftWristTarget.source  = LHand;      // 左手腕
        rightElbowTarget.source = RLowerArm;  // 右手肘
        rightWristTarget.source = RHand;      // 右手腕
    }
}
