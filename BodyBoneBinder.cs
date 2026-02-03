using UnityEngine;
using System.Collections;

public class BodyBoneBinder : MonoBehaviour
{
    [Header("RealisticCharacterMirrored 根物件")]
    public Animator bodyAnimator;

    [Header("Targets（放在 openarm 底下的 4 個空物件）")]
    public CopyTransform leftElbowTarget, leftWristTarget;
    public CopyTransform rightElbowTarget, rightWristTarget;

    [Header("APK 初始化設定")]
    [Tooltip("啟動時延遲綁定的時間（秒），等待 XR/Body Tracking 系統準備好")]
    public float initDelay = 1.5f;

    bool _initialized = false;

    void Reset()
    {
        bodyAnimator = FindObjectOfType<Animator>();
    }

    void Start()
    {
        // 延遲初始化，等待 XR/Body Tracking 系統準備好
        StartCoroutine(DelayedBind());
    }

    IEnumerator DelayedBind()
    {
        // 等待指定時間
        yield return new WaitForSeconds(initDelay);

        // 額外等待一幀，確保 Animator 已更新
        yield return null;

        BindBones();
    }

    void BindBones()
    {
        if (_initialized) return;

        if (bodyAnimator == null)
        {
            Debug.LogError("[BodyBoneBinder] bodyAnimator 未設定！");
            return;
        }

        // 取常用的人體骨頭（Humanoid Avatar 必須是 Valid）
        var LLowerArm = bodyAnimator.GetBoneTransform(HumanBodyBones.LeftLowerArm);
        var LHand     = bodyAnimator.GetBoneTransform(HumanBodyBones.LeftHand);
        var RLowerArm = bodyAnimator.GetBoneTransform(HumanBodyBones.RightLowerArm);
        var RHand     = bodyAnimator.GetBoneTransform(HumanBodyBones.RightHand);

        // 檢查骨骼是否有效
        if (LLowerArm == null || LHand == null || RLowerArm == null || RHand == null)
        {
            Debug.LogWarning("[BodyBoneBinder] 部分骨骼為 null，可能 Avatar 尚未準備好");
            // 重試
            StartCoroutine(RetryBind());
            return;
        }

        leftElbowTarget.source  = LLowerArm;  // 左手肘
        leftWristTarget.source  = LHand;      // 左手腕
        rightElbowTarget.source = RLowerArm;  // 右手肘
        rightWristTarget.source = RHand;      // 右手腕

        _initialized = true;
        Debug.Log("[BodyBoneBinder] 骨骼綁定完成");
    }

    IEnumerator RetryBind()
    {
        yield return new WaitForSeconds(0.5f);
        BindBones();
    }

    /// <summary>
    /// 當 Quest 被放下/戴回時觸發，重新綁定骨骼
    /// </summary>
    void OnApplicationPause(bool pauseStatus)
    {
        if (!pauseStatus)
        {
            // 從暫停恢復時，重新綁定
            Debug.Log("[BodyBoneBinder] 從暫停恢復，重新綁定骨骼");
            StartCoroutine(RebindAfterResume());
        }
    }

    IEnumerator RebindAfterResume()
    {
        // 等待 XR 系統恢復
        yield return new WaitForSeconds(0.5f);
        yield return null;

        _initialized = false;
        BindBones();
    }
}
