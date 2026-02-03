using UnityEngine;
using System.Collections;

/// <summary>
/// 延遲啟用 CharacterRetargeter，等待 XR Body Tracking 系統準備好
/// 解決 APK 啟動時模型變形的問題
/// </summary>
public class DelayedCharacterEnabler : MonoBehaviour
{
    [Header("目標物件")]
    [Tooltip("RealisticCharacter 物件（包含 CharacterRetargeter）")]
    public GameObject characterObject;

    [Header("延遲設定")]
    [Tooltip("啟動時延遲啟用的時間（秒）")]
    public float enableDelay = 2.0f;

    [Tooltip("啟動時是否先隱藏模型")]
    public bool hideOnStart = true;

    [Header("除錯")]
    public bool showDebugLog = true;

    private MonoBehaviour _characterRetargeter;
    private Renderer[] _renderers;
    private bool _initialized = false;

    void Start()
    {
        if (characterObject == null)
        {
            Debug.LogError("[DelayedCharacterEnabler] characterObject 未設定！");
            return;
        }

        // 找到 CharacterRetargeter 組件（用反射避免直接依賴）
        foreach (var component in characterObject.GetComponents<MonoBehaviour>())
        {
            if (component.GetType().Name.Contains("CharacterRetargeter"))
            {
                _characterRetargeter = component;
                break;
            }
        }

        if (_characterRetargeter == null)
        {
            Debug.LogWarning("[DelayedCharacterEnabler] 找不到 CharacterRetargeter 組件");
        }

        // 獲取所有 Renderer
        _renderers = characterObject.GetComponentsInChildren<Renderer>(true);

        // 啟動時禁用
        if (hideOnStart)
        {
            SetCharacterVisible(false);
        }

        if (_characterRetargeter != null)
        {
            _characterRetargeter.enabled = false;
        }

        // 延遲啟用
        StartCoroutine(DelayedEnable());
    }

    IEnumerator DelayedEnable()
    {
        if (showDebugLog)
            Debug.Log($"[DelayedCharacterEnabler] 等待 {enableDelay} 秒...");

        yield return new WaitForSeconds(enableDelay);

        // 額外等待幾幀確保 XR 系統穩定
        for (int i = 0; i < 3; i++)
        {
            yield return null;
        }

        EnableCharacter();
    }

    void EnableCharacter()
    {
        if (_initialized) return;

        if (_characterRetargeter != null)
        {
            _characterRetargeter.enabled = true;
        }

        // 再等一小段時間讓 retargeter 初始化
        StartCoroutine(ShowCharacterAfterDelay());
    }

    IEnumerator ShowCharacterAfterDelay()
    {
        yield return new WaitForSeconds(0.5f);

        SetCharacterVisible(true);
        _initialized = true;

        if (showDebugLog)
            Debug.Log("[DelayedCharacterEnabler] 角色已啟用");
    }

    void SetCharacterVisible(bool visible)
    {
        if (_renderers == null) return;

        foreach (var renderer in _renderers)
        {
            if (renderer != null)
            {
                renderer.enabled = visible;
            }
        }
    }

    /// <summary>
    /// Quest 放下/戴回時重新初始化
    /// </summary>
    void OnApplicationPause(bool pauseStatus)
    {
        if (!pauseStatus && _initialized)
        {
            if (showDebugLog)
                Debug.Log("[DelayedCharacterEnabler] 從暫停恢復，重新初始化");

            StartCoroutine(ReinitializeAfterResume());
        }
    }

    IEnumerator ReinitializeAfterResume()
    {
        // 先隱藏
        SetCharacterVisible(false);

        // 等待 XR 系統恢復
        yield return new WaitForSeconds(1.0f);

        // 顯示
        SetCharacterVisible(true);

        if (showDebugLog)
            Debug.Log("[DelayedCharacterEnabler] 重新初始化完成");
    }
}
