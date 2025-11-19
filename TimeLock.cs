using UnityEngine;

public class TimeLock : MonoBehaviour
{
    void Awake()
    {
        Time.timeScale = 1f;          // 正常時間流速
        Time.fixedDeltaTime = 0.02f;  // 50 Hz 物理步長（與 Editor 一致）
        Application.targetFrameRate = 90; // 可改 72/90；只是顯示幀率，非必需
    }
}
