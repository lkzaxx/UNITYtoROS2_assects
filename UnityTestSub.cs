using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CamBytesTest : MonoBehaviour
{
    int n = 0;
    void Start()
    {
        var ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<CompressedImageMsg>("/camera/left/compressed", msg =>
        {
            n++;
            if (n % 30 == 0)
                Debug.Log($"CAM RX {n}, bytes={msg.data?.Length ?? 0}, format={msg.format}");
        });
        Debug.Log("Subscribed /camera/left/compressed");
    }
}
