using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using System.Collections;

public class UnityTestSub : MonoBehaviour
{
    [SerializeField] private ROSConnection ros; // Inspector 拖場景那顆 ROSConnection 進來（最穩）

    private IEnumerator Start()
    {
        // 先印目前場景裡 ROSConnection 數量
        Debug.Log("ROSConnection count = " + FindObjectsOfType<ROSConnection>().Length);

        // 確保抓到場景那顆
        if (ros == null) ros = FindObjectOfType<ROSConnection>();

        if (ros == null)
        {
            Debug.LogError("Scene 裡找不到 ROSConnection");
            yield break;
        }

        Debug.Log($"ROSConnection used: {ros.gameObject.name} id={ros.GetInstanceID()}");

        // 你的環境有 XR 初始化/焦點切換，保守等久一點
        yield return new WaitForSecondsRealtime(3f);

        // 訂閱測試 topic
        ros.Subscribe<StringMsg>("/unity_test", msg => Debug.Log("RX: " + msg.data));
        Debug.Log("Subscribed /unity_test (scene ros)");
    }
}
