using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System;
using System.Collections;

/// <summary>
/// 接收 ROS2 CompressedImage 並顯示到 VR 雙眼
/// 
/// 支援兩種模式：
/// 1. UI 模式：顯示到 RawImage (測試用)
/// 2. VR 模式：顯示到 Camera 的 RenderTexture (正式 VR 使用)
/// 
/// Topics:
///   /camera/left/compressed  - 左眼影像
///   /camera/right/compressed - 右眼影像
/// </summary>
public class CameraStreamReceiver : MonoBehaviour
{
    [Header("=== 模式選擇 ===")]
    [Tooltip("使用 VR Camera 模式還是 UI RawImage 模式")]
    public RenderMode renderMode = RenderMode.UI;
    
    public enum RenderMode
    {
        UI,         // 使用 RawImage 顯示（測試用）
        VRCamera    // 使用 Camera + RenderTexture（VR 用）
    }

    [Header("=== UI 模式設定 ===")]
    [Tooltip("左眼影像顯示的 RawImage")]
    public RawImage leftEyeImage;
    
    [Tooltip("右眼影像顯示的 RawImage")]
    public RawImage rightEyeImage;

    [Header("=== VR Camera 模式設定 ===")]
    [Tooltip("左眼 Camera（會在前方顯示 Quad）")]
    public Camera leftEyeCamera;
    
    [Tooltip("右眼 Camera（會在前方顯示 Quad）")]
    public Camera rightEyeCamera;
    
    [Tooltip("Quad 距離 Camera 的距離")]
    public float quadDistance = 1.0f;

    [Header("=== Topic 設定 ===")]
    [Tooltip("左眼相機 Topic")]
    public string leftCameraTopic = "/camera/left/compressed";
    
    [Tooltip("右眼相機 Topic")]
    public string rightCameraTopic = "/camera/right/compressed";
    
    [Tooltip("啟用左眼")]
    public bool enableLeft = true;
    
    [Tooltip("啟用右眼")]
    public bool enableRight = true;

    [Header("=== 狀態監控 ===")]
    [SerializeField] private bool rosConnected = false;
    [SerializeField] private bool leftReceiving = false;
    [SerializeField] private bool rightReceiving = false;
    [SerializeField] private int leftFrameCount = 0;
    [SerializeField] private int rightFrameCount = 0;
    [SerializeField] private float leftFps = 0f;
    [SerializeField] private float rightFps = 0f;

    // 內部變數
    private ROSConnection ros;
    private Texture2D leftTexture;
    private Texture2D rightTexture;
    
    // VR 模式用的 Quad
    private GameObject leftQuad;
    private GameObject rightQuad;
    private Material leftMaterial;
    private Material rightMaterial;
    
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
        
        // 初始化 Texture
        leftTexture = new Texture2D(2, 2);
        rightTexture = new Texture2D(2, 2);
        
        // 根據模式初始化顯示
        if (renderMode == RenderMode.VRCamera)
        {
            SetupVRMode();
        }
        else
        {
            SetupUIMode();
        }
        
        leftLastTime = Time.time;
        rightLastTime = Time.time;
        lastLogTime = Time.time;
        
        // 延遲訂閱，確保 ROSTCPManager 已經初始化完成
        StartCoroutine(DelayedSubscribe());
    }

    /// <summary>
    /// 延遲訂閱 - 等待 ROSTCPManager 初始化完成
    /// </summary>
    IEnumerator DelayedSubscribe()
    {
        Debug.Log("[CameraStreamReceiver] 等待 1.5 秒讓 ROSTCPManager 初始化...");
        yield return new WaitForSeconds(1.5f);
        
        // 取得 ROS 連接
        try
        {
            ros = ROSConnection.GetOrCreateInstance();
            rosConnected = true;
            Debug.Log("[CameraStreamReceiver] ROS Connection 取得成功");
        }
        catch (Exception e)
        {
            Debug.LogError($"[CameraStreamReceiver] ROS Connection 失敗: {e.Message}");
            yield break;
        }
        
        // 訂閱 Topic
        if (enableLeft)
        {
            ros.Subscribe<CompressedImageMsg>(leftCameraTopic, OnLeftImageReceived);
            Debug.Log($"[CameraStreamReceiver] ✓ 已訂閱 {leftCameraTopic}");
        }
        
        if (enableRight)
        {
            ros.Subscribe<CompressedImageMsg>(rightCameraTopic, OnRightImageReceived);
            Debug.Log($"[CameraStreamReceiver] ✓ 已訂閱 {rightCameraTopic}");
        }
        
        Debug.Log("[CameraStreamReceiver] === 初始化完成，等待影像... ===");
    }

    /// <summary>
    /// 設定 UI 模式
    /// </summary>
    private void SetupUIMode()
    {
        if (enableLeft && leftEyeImage == null)
        {
            Debug.LogWarning("[CameraStreamReceiver] UI 模式：leftEyeImage 未設定！");
        }
        if (enableRight && rightEyeImage == null)
        {
            Debug.LogWarning("[CameraStreamReceiver] UI 模式：rightEyeImage 未設定！");
        }
    }

    /// <summary>
    /// 設定 VR Camera 模式 - 在每個 Camera 前方建立 Quad
    /// </summary>
    private void SetupVRMode()
    {
        Debug.Log("[CameraStreamReceiver] 設定 VR Camera 模式...");
        
        if (enableLeft)
        {
            if (leftEyeCamera == null)
            {
                Debug.LogWarning("[CameraStreamReceiver] VR 模式：leftEyeCamera 未設定！");
            }
            else
            {
                leftQuad = CreateDisplayQuad("LeftEyeQuad", leftEyeCamera, out leftMaterial);
                Debug.Log("[CameraStreamReceiver] ✓ 左眼 Quad 建立完成");
            }
        }
        
        if (enableRight)
        {
            if (rightEyeCamera == null)
            {
                Debug.LogWarning("[CameraStreamReceiver] VR 模式：rightEyeCamera 未設定！");
            }
            else
            {
                rightQuad = CreateDisplayQuad("RightEyeQuad", rightEyeCamera, out rightMaterial);
                Debug.Log("[CameraStreamReceiver] ✓ 右眼 Quad 建立完成");
            }
        }
    }

    /// <summary>
    /// 建立顯示用的 Quad
    /// </summary>
    private GameObject CreateDisplayQuad(string name, Camera targetCamera, out Material material)
    {
        GameObject quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
        quad.name = name;
        
        // 設定為 Camera 的子物件
        quad.transform.SetParent(targetCamera.transform);
        quad.transform.localPosition = new Vector3(0, 0, quadDistance);
        quad.transform.localRotation = Quaternion.identity;
        
        // 計算 Quad 大小以填滿視野
        float height = 2.0f * quadDistance * Mathf.Tan(targetCamera.fieldOfView * 0.5f * Mathf.Deg2Rad);
        float width = height * targetCamera.aspect;
        quad.transform.localScale = new Vector3(width, height, 1);
        
        // 移除 Collider
        Destroy(quad.GetComponent<Collider>());
        
        // 建立材質
        material = new Material(Shader.Find("Unlit/Texture"));
        quad.GetComponent<Renderer>().material = material;
        
        // 設定 Layer（可選，避免被其他 Camera 看到）
        quad.layer = targetCamera.gameObject.layer;
        
        return quad;
    }

    void Update()
    {
        // 處理左眼影像 (主執行緒)
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
        
        // 處理右眼影像 (主執行緒)
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
        
        // 計算 FPS 和定期 Log
        UpdateFpsAndLog();
    }

    /// <summary>
    /// 左眼影像接收回調 (ROS 執行緒)
    /// </summary>
    private void OnLeftImageReceived(CompressedImageMsg msg)
    {
        lock (leftLock)
        {
            pendingLeftData = msg.data;
        }
    }

    /// <summary>
    /// 右眼影像接收回調 (ROS 執行緒)
    /// </summary>
    private void OnRightImageReceived(CompressedImageMsg msg)
    {
        lock (rightLock)
        {
            pendingRightData = msg.data;
        }
    }

    /// <summary>
    /// 處理並顯示影像
    /// </summary>
    private void ProcessImage(byte[] imageData, Texture2D texture, RawImage uiImage, Material vrMaterial, string side)
    {
        if (imageData == null || imageData.Length == 0)
        {
            Debug.LogWarning($"[CameraStreamReceiver] {side}: 收到空資料");
            return;
        }
        
        try
        {
            // 解碼 JPEG
            if (texture.LoadImage(imageData))
            {
                if (renderMode == RenderMode.UI)
                {
                    // UI 模式
                    if (uiImage != null)
                    {
                        uiImage.texture = texture;
                    }
                }
                else
                {
                    // VR Camera 模式
                    if (vrMaterial != null)
                    {
                        vrMaterial.mainTexture = texture;
                    }
                }
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

    /// <summary>
    /// 更新 FPS 計算和定期 Log
    /// </summary>
    private void UpdateFpsAndLog()
    {
        float currentTime = Time.time;
        
        // 左眼 FPS
        if (currentTime - leftLastTime >= 1.0f)
        {
            leftFps = leftFramesInSecond / (currentTime - leftLastTime);
            leftFramesInSecond = 0;
            leftLastTime = currentTime;
        }
        
        // 右眼 FPS
        if (currentTime - rightLastTime >= 1.0f)
        {
            rightFps = rightFramesInSecond / (currentTime - rightLastTime);
            rightFramesInSecond = 0;
            rightLastTime = currentTime;
        }
        
        // 每 5 秒輸出狀態
        if (currentTime - lastLogTime >= 5.0f)
        {
            Debug.Log($"[CameraStreamReceiver] 狀態: Left={leftFrameCount} frames ({leftFps:F1} fps), Right={rightFrameCount} frames ({rightFps:F1} fps)");
            lastLogTime = currentTime;
        }
    }

    /// <summary>
    /// 取得狀態
    /// </summary>
    public bool IsLeftReceiving => leftReceiving;
    public bool IsRightReceiving => rightReceiving;
    public float LeftFps => leftFps;
    public float RightFps => rightFps;

    void OnDestroy()
    {
        // 清理資源
        if (leftTexture != null) Destroy(leftTexture);
        if (rightTexture != null) Destroy(rightTexture);
        if (leftQuad != null) Destroy(leftQuad);
        if (rightQuad != null) Destroy(rightQuad);
        if (leftMaterial != null) Destroy(leftMaterial);
        if (rightMaterial != null) Destroy(rightMaterial);
    }

    /// <summary>
    /// 在 Game 視窗顯示除錯資訊
    /// </summary>
    void OnGUI()
    {
        GUIStyle style = new GUIStyle();
        style.fontSize = 16;
        style.normal.textColor = Color.white;
        
        GUILayout.BeginArea(new Rect(10, 10, 350, 150));
        
        GUI.color = rosConnected ? Color.green : Color.red;
        GUILayout.Label($"ROS: {(rosConnected ? "Connected" : "Disconnected")}", style);
        
        GUI.color = leftReceiving ? Color.green : Color.yellow;
        GUILayout.Label($"Left:  {(leftReceiving ? "Receiving" : "Waiting...")} | {leftFps:F1} fps | {leftFrameCount} frames", style);
        
        GUI.color = rightReceiving ? Color.green : Color.yellow;
        GUILayout.Label($"Right: {(rightReceiving ? "Receiving" : "Waiting...")} | {rightFps:F1} fps | {rightFrameCount} frames", style);
        
        GUI.color = Color.white;
        GUILayout.Label($"Mode: {renderMode}", style);
        
        GUILayout.EndArea();
    }
}
