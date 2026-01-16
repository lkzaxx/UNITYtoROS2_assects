using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System;
using System.Collections;

public class CameraStreamReceiver : MonoBehaviour
{
    [Header("=== 模式選擇 ===")]
    [Tooltip("UI=RawImage；VRCamera=左右眼各一個Quad；HUD=貼在CenterEye前方(推薦)")]
    public RenderMode renderMode = RenderMode.UI;

    public enum RenderMode
    {
        UI,         // 使用 RawImage 顯示（測試用）
        VRCamera,   // (不建議) 使用 Left/Right Camera 各自掛 Quad（你之前會抖動/撕裂）
        HUD         // ✅ 推薦：只用 CenterEyeCamera + 一個 HUD Quad（貼螢幕）
    }

    [Header("=== ROS Connection Override ===")]
    [Tooltip("如果設定，將使用這個 ROSConnection 實例")]
    [SerializeField] private ROSConnection rosOverride;

    [Header("=== UI 模式設定 ===")]
    public RawImage leftEyeImage;
    public RawImage rightEyeImage;

    [Header("=== VRCamera 模式設定（不建議） ===")]
    public Camera leftEyeCamera;
    public Camera rightEyeCamera;
    public float quadDistance = 1.0f;

    [Header("=== HUD 模式設定（貼螢幕） ===")]
    [Tooltip("請拖 CenterEyeAnchor 上的 Camera")]
    public Camera centerEyeCamera;
    [Tooltip("HUD 距離（m），0.35~0.6 推薦")]
    public float hudDistance = 0.45f;
    [Tooltip("視野填滿倍率，1.0=剛好填滿，1.1~1.2=略超出避免邊緣漏黑")]
    public float hudFovFill = 1.1f;

    [Header("=== Topic 設定 ===")]
    public string leftCameraTopic = "/camera/left/compressed";
    public string rightCameraTopic = "/camera/right/compressed";
    public bool enableLeft = true;
    public bool enableRight = true;

    [Header("=== 狀態監控 ===")]
    [SerializeField] private bool rosConnected = false;
    [SerializeField] private bool leftReceiving = false;
    [SerializeField] private bool rightReceiving = false;
    [SerializeField] private int leftFrameCount = 0;
    [SerializeField] private int rightFrameCount = 0;
    [SerializeField] private int leftRxPackets = 0;
    [SerializeField] private int rightRxPackets = 0;
    [SerializeField] private float leftFps = 0f;
    [SerializeField] private float rightFps = 0f;

    // 內部變數
    private ROSConnection ros;
    private Texture2D leftTexture;
    private Texture2D rightTexture;

    // VRCamera 模式用
    private GameObject leftQuad;
    private GameObject rightQuad;
    private Material leftMaterial;
    private Material rightMaterial;

    // HUD 模式用
    private GameObject hudQuad;
    private Material hudMaterial;

    // FPS 計算
    private float leftLastTime;
    private float rightLastTime;
    private int leftFramesInSecond = 0;
    private int rightFramesInSecond = 0;
    private float lastLogTime = 0f;

    // 執行緒安全的資料緩衝
    private byte[] pendingLeftData = null;
    private byte[] pendingRightData = null;
    private readonly object leftLock = new object();
    private readonly object rightLock = new object();

    void Start()
    {
        Debug.Log("[CameraStreamReceiver] === 初始化開始 ===");
        Debug.Log($"[CameraStreamReceiver] Render Mode: {renderMode}");
        Debug.Log($"[CameraStreamReceiver] Left Topic: {leftCameraTopic}");
        Debug.Log($"[CameraStreamReceiver] Right Topic: {rightCameraTopic}");

        leftTexture = new Texture2D(2, 2);
        rightTexture = new Texture2D(2, 2);

        if (renderMode == RenderMode.VRCamera)
            SetupVRMode();
        else if (renderMode == RenderMode.HUD)
            SetupHUDMode();
        else
            SetupUIMode();

        leftLastTime = Time.time;
        rightLastTime = Time.time;
        lastLogTime = Time.time;

        StartCoroutine(DelayedSubscribe());
    }

    IEnumerator DelayedSubscribe()
    {
        Debug.Log("[CameraStreamReceiver] 等待 3.0 秒讓 ROSConnection 連線穩定...");
        yield return new WaitForSecondsRealtime(3.0f);

        Debug.Log("[CameraStreamReceiver] ROSConnection count = " + RosConn.CountROS());

        ros = rosOverride != null ? rosOverride : RosConn.GetSceneROS();
        if (ros == null)
        {
            Debug.LogError("[CameraStreamReceiver] Scene 裡找不到 ROSConnection（請確認 Hierarchy 只有一顆，且已啟用）");
            rosConnected = false;
            yield break;
        }

        rosConnected = true;
        Debug.Log($"[CameraStreamReceiver] Using ROSConnection: {ros.gameObject.name} id={ros.GetInstanceID()}");

        if (enableLeft)
        {
            ros.Subscribe<CompressedImageMsg>(leftCameraTopic, OnLeftImageReceived);
            Debug.Log($"[CameraStreamReceiver] ✓ Subscribed {leftCameraTopic}");
        }

        if (enableRight)
        {
            ros.Subscribe<CompressedImageMsg>(rightCameraTopic, OnRightImageReceived);
            Debug.Log($"[CameraStreamReceiver] ✓ Subscribed {rightCameraTopic}");
        }

        Debug.Log("[CameraStreamReceiver] === 訂閱完成，等待影像... ===");
    }

    private void SetupUIMode()
    {
        if (enableLeft && leftEyeImage == null)
            Debug.LogWarning("[CameraStreamReceiver] UI 模式：leftEyeImage 未設定！");
        if (enableRight && rightEyeImage == null)
            Debug.LogWarning("[CameraStreamReceiver] UI 模式：rightEyeImage 未設定！");
    }

    private void SetupVRMode()
    {
        Debug.LogWarning("[CameraStreamReceiver] VRCamera 模式容易造成兩張相機疊加抖動，建議改用 HUD 模式。");

        if (enableLeft)
        {
            if (leftEyeCamera == null) Debug.LogWarning("[CameraStreamReceiver] VR 模式：leftEyeCamera 未設定！");
            else leftQuad = CreateDisplayQuad("LeftEyeQuad", leftEyeCamera, out leftMaterial);
        }

        if (enableRight)
        {
            if (rightEyeCamera == null) Debug.LogWarning("[CameraStreamReceiver] VR 模式：rightEyeCamera 未設定！");
            else rightQuad = CreateDisplayQuad("RightEyeQuad", rightEyeCamera, out rightMaterial);
        }
    }

    private void SetupHUDMode()
    {
        Debug.Log("[CameraStreamReceiver] 設定 HUD（貼螢幕）模式...");

        if (centerEyeCamera == null)
        {
            // 嘗試自動抓：CenterEyeAnchor 常常就是 Main Camera
            centerEyeCamera = Camera.main;
        }
        if (centerEyeCamera == null)
        {
            Debug.LogError("[CameraStreamReceiver] HUD 模式需要 centerEyeCamera，請拖 CenterEyeAnchor 上的 Camera！");
            return;
        }

        // 建立 HUD Quad
        hudQuad = GameObject.CreatePrimitive(PrimitiveType.Quad);
        hudQuad.name = "HUDQuad";
        Destroy(hudQuad.GetComponent<Collider>());

        hudQuad.transform.SetParent(centerEyeCamera.transform, false);
        hudQuad.transform.localPosition = new Vector3(0, 0, hudDistance);
        hudQuad.transform.localRotation = Quaternion.identity;

        // 計算填滿視野的 Quad 尺寸
        float h = 2f * hudDistance * Mathf.Tan(centerEyeCamera.fieldOfView * 0.5f * Mathf.Deg2Rad);
        float w = h * centerEyeCamera.aspect;
        hudQuad.transform.localScale = new Vector3(w * hudFovFill, h * hudFovFill, 1);

        // 材質（URP 優先）
        Shader s = Shader.Find("Unlit/StereoHUD");
        if (s == null)
        {
            Debug.LogError("[CameraStreamReceiver] 找不到 Shader 'Unlit/StereoHUD'，請確認 StereoHUD.shader 已建立且無編譯錯誤");
            s = Shader.Find("Universal Render Pipeline/Unlit");
            if (s == null) s = Shader.Find("Unlit/Texture");
        }
        hudMaterial = new Material(s);
        hudQuad.GetComponent<Renderer>().material = hudMaterial;

        Debug.Log("[CameraStreamReceiver] ✓ HUDQuad 建立完成");
    }

    private GameObject CreateDisplayQuad(string name, Camera targetCamera, out Material material)
    {
        GameObject quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
        quad.name = name;

        quad.transform.SetParent(targetCamera.transform, false);
        quad.transform.localPosition = new Vector3(0, 0, quadDistance);
        quad.transform.localRotation = Quaternion.identity;

        float height = 2.0f * quadDistance * Mathf.Tan(targetCamera.fieldOfView * 0.5f * Mathf.Deg2Rad);
        float width = height * targetCamera.aspect;
        quad.transform.localScale = new Vector3(width, height, 1);

        Destroy(quad.GetComponent<Collider>());

        Shader s = Shader.Find("Universal Render Pipeline/Unlit");
        if (s == null) s = Shader.Find("Unlit/Texture");
        material = new Material(s);
        quad.GetComponent<Renderer>().material = material;

        return quad;
    }

    void Update()
    {
        // 左
        byte[] leftData = null;
        lock (leftLock)
        {
            if (pendingLeftData != null)
            {
                leftData = pendingLeftData;
                pendingLeftData = null;
            }
        }
        if (leftData != null)
        {
            ProcessImage(leftData, leftTexture, leftEyeImage, leftMaterial, "Left");
            leftFrameCount++;
            leftFramesInSecond++;
            leftReceiving = true;
        }

        // 右
        byte[] rightData = null;
        lock (rightLock)
        {
            if (pendingRightData != null)
            {
                rightData = pendingRightData;
                pendingRightData = null;
            }
        }
        if (rightData != null)
        {
            ProcessImage(rightData, rightTexture, rightEyeImage, rightMaterial, "Right");
            rightFrameCount++;
            rightFramesInSecond++;
            rightReceiving = true;
        }

        if (renderMode == RenderMode.HUD && hudMaterial != null)
        {
            // StereoHUD：左右眼不同貼圖
            if (hudMaterial.HasProperty("_LeftTex"))  hudMaterial.SetTexture("_LeftTex", leftTexture);
            if (hudMaterial.HasProperty("_RightTex")) hudMaterial.SetTexture("_RightTex", rightTexture);
        }

        UpdateFpsAndLog();
    }

    private void OnLeftImageReceived(CompressedImageMsg msg)
    {
        leftRxPackets++;
        if (leftRxPackets % 60 == 0)
            Debug.Log($"[CameraStreamReceiver] Left RX packets={leftRxPackets}, bytes={msg.data?.Length ?? 0}, format={msg.format}");

        if (msg.data == null || msg.data.Length == 0) return;

        var copy = new byte[msg.data.Length];
        Buffer.BlockCopy(msg.data, 0, copy, 0, msg.data.Length);

        lock (leftLock) pendingLeftData = copy;
    }

    private void OnRightImageReceived(CompressedImageMsg msg)
    {
        rightRxPackets++;
        if (rightRxPackets % 60 == 0)
            Debug.Log($"[CameraStreamReceiver] Right RX packets={rightRxPackets}, bytes={msg.data?.Length ?? 0}, format={msg.format}");

        if (msg.data == null || msg.data.Length == 0) return;

        var copy = new byte[msg.data.Length];
        Buffer.BlockCopy(msg.data, 0, copy, 0, msg.data.Length);

        lock (rightLock) pendingRightData = copy;
    }

    private void ProcessImage(byte[] imageData, Texture2D texture, RawImage uiImage, Material vrMaterial, string side)
    {
        if (imageData == null || imageData.Length == 0) return;

        try
        {
            if (texture.LoadImage(imageData))
            {
                if (renderMode == RenderMode.UI)
                {
                    if (uiImage != null) uiImage.texture = texture;
                }
                else if (renderMode == RenderMode.VRCamera)
                {
                    if (vrMaterial != null) SetMatTexture(vrMaterial, texture);
                }
                // HUD 模式的貼圖更新在 Update() 統一做
            }
            else
            {
                Debug.LogWarning($"[CameraStreamReceiver] {side}: JPEG 解碼失敗");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"[CameraStreamReceiver] {side} 錯誤: {e.Message}");
        }
    }

    // URP / Built-in 都能設定貼圖
    private static void SetMatTexture(Material mat, Texture tex)
    {
        if (mat == null) return;
        if (mat.HasProperty("_BaseMap")) mat.SetTexture("_BaseMap", tex);   // URP Unlit/Lit
        else mat.mainTexture = tex;                                        // Built-in
    }

    private void UpdateFpsAndLog()
    {
        float currentTime = Time.time;

        if (currentTime - leftLastTime >= 1.0f)
        {
            leftFps = leftFramesInSecond / (currentTime - leftLastTime);
            leftFramesInSecond = 0;
            leftLastTime = currentTime;
        }

        if (currentTime - rightLastTime >= 1.0f)
        {
            rightFps = rightFramesInSecond / (currentTime - rightLastTime);
            rightFramesInSecond = 0;
            rightLastTime = currentTime;
        }

        if (currentTime - lastLogTime >= 5.0f)
        {
            Debug.Log($"[CameraStreamReceiver] 狀態: Left={leftFrameCount} ({leftFps:F1} fps), Right={rightFrameCount} ({rightFps:F1} fps)");
            lastLogTime = currentTime;
        }
    }

    void OnDestroy()
    {
        if (leftTexture != null) Destroy(leftTexture);
        if (rightTexture != null) Destroy(rightTexture);

        if (leftQuad != null) Destroy(leftQuad);
        if (rightQuad != null) Destroy(rightQuad);
        if (hudQuad != null) Destroy(hudQuad);

        if (leftMaterial != null) Destroy(leftMaterial);
        if (rightMaterial != null) Destroy(rightMaterial);
        if (hudMaterial != null) Destroy(hudMaterial);
    }

    void OnGUI()
    {
        GUIStyle style = new GUIStyle { fontSize = 16 };
        style.normal.textColor = Color.white;

        GUILayout.BeginArea(new Rect(10, 10, 420, 180));

        GUI.color = rosConnected ? Color.green : Color.red;
        GUILayout.Label($"ROS: {(rosConnected ? "Connected" : "Disconnected")}", style);

        GUI.color = leftReceiving ? Color.green : Color.yellow;
        GUILayout.Label($"Left:  {(leftReceiving ? "Receiving" : "Waiting...")} | {leftFps:F1} fps | {leftFrameCount} frames", style);
        GUILayout.Label($"Left RX packets: {leftRxPackets}", style);

        GUI.color = rightReceiving ? Color.green : Color.yellow;
        GUILayout.Label($"Right: {(rightReceiving ? "Receiving" : "Waiting...")} | {rightFps:F1} fps | {rightFrameCount} frames", style);
        GUILayout.Label($"Right RX packets: {rightRxPackets}", style);

        GUI.color = Color.white;
        GUILayout.Label($"Mode: {renderMode}", style);

        GUILayout.EndArea();
    }
}
