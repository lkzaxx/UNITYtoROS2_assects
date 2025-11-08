using UnityEngine;

public class CopyTransform : MonoBehaviour
{
    public Transform source;     // 指向人體骨頭
    public Vector3 posOffset;    // 需要時微調
    public Vector3 eulOffset;    // 需要時微調
    public bool copyRotation = true;

    void LateUpdate()
    {
        if (!source) return;
        transform.position = source.position + transform.TransformVector(posOffset);
        if (copyRotation)
            transform.rotation = source.rotation * Quaternion.Euler(eulOffset);
    }
}
