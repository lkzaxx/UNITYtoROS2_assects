using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public static class RosConn
{
    public static ROSConnection GetSceneROS()
    {
        return Object.FindObjectOfType<ROSConnection>();
    }

    public static int CountROS()
    {
        return Object.FindObjectsOfType<ROSConnection>().Length;
    }
}
