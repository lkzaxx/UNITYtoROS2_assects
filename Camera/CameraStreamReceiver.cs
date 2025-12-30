using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System;

/// <summary>
/// 接收 ROS2 CompressedImage 並顯示到 Unity UI
/// 
/// 使用方式：
/// 1. 將此腳本附加到 GameObject
/// 2. 設定 leftEyeImage 和/或 rightEyeImage (RawImage 組件)
/// 3. 選擇顯示模式 (左眼/右眼/雙眼)
/// 
/// Topics:
///   /camera/left/compressed  - 左眼影像
///   /camera/right/compressed - 右眼影像
/// </summary>
public class CameraStreamReceiver : MonoBehaviour
{
    [Header("顯示設定")]
    [Tooltip("左眼影像顯示的 RawImage")]
    public RawImage leftEyeImage;
    
    [Tooltip("右眼影像顯示的 RawImage")]
    public RawImage rightEyeImage;
    
    [Tooltip("顯示模式")]
    public DisplayMode displayMode = DisplayMode.Both;
    
    [Header("Topic 設定")]
    [Tooltip("左眼相機 Topic")]
    public string leftCameraTopic = "/camera/left/compressed";
    
    [Tooltip("右眼相機 Topic")]
    public string rightCameraTopic = "/camera/right/compressed";
    
    [Header("狀態")]
    [SerializeField] private bool leftConnected = false;
    [SerializeField] private bool rightConnected = false;
    [SerializeField] private int leftFrameCount = 0;
    [SerializeField] private int rightFrameCount = 0;
    [SerializeField] private float leftFps = 0f;
    [SerializeField] private float rightFps = 0f;

    public enum DisplayMode
    {
        LeftOnly,   // 只顯示左眼
        RightOnly,  // 只顯示右眼
        Both        // 雙眼都顯示
    }

    // 內部變數
    private ROSConnection ros;
    private Texture2D leftTexture;
    private Texture2D rightTexture;
    
    // FPS 計算
    private float leftLastTime;
    private float rightLastTime;
    private int leftFramesInSecond = 0;
    private int rightFramesInSecond = 0;

    // 執行緒安全的資料緩衝
    private byte[] pendingLeftData = null;
    private byte[] pendingRightData = null;
    private readonly object leftLock = new object();
    private readonly object rightLock = new object();

    void Start()
    {
        // 取得 ROS 連接
        ros = ROSConnection.GetOrCreateInstance();
        
        // 初始化 Texture
        leftTexture = new Texture2D(2, 2);
        rightTexture = new Texture2D(2, 2);
        
        // 根據模式訂閱 Topic
        if (displayMode == DisplayMode.LeftOnly || displayMode == DisplayMode.Both)
        {
            ros.Subscribe<CompressedImageMsg>(leftCameraTopic, OnLeftImageReceived);
            Debug.Log($"[CameraStreamReceiver] Subscribed to {leftCameraTopic}");
        }
        
        if (displayMode == DisplayMode.RightOnly || displayMode == DisplayMode.Both)
        {
            ros.Subscribe<CompressedImageMsg>(rightCameraTopic, OnRightImageReceived);
            Debug.Log($"[CameraStreamReceiver] Subscribed to {rightCameraTopic}");
        }
        
        leftLastTime = Time.time;
        rightLastTime = Time.time;
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
            UpdateTexture(leftData, leftTexture, leftEyeImage);
            leftFrameCount++;
            leftFramesInSecond++;
            leftConnected = true;
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
            UpdateTexture(rightData, rightTexture, rightEyeImage);
            rightFrameCount++;
            rightFramesInSecond++;
            rightConnected = true;
        }
        
        // 計算 FPS (每秒更新)
        UpdateFpsCalculation();
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
    /// 更新 Texture 並顯示到 RawImage
    /// </summary>
    private void UpdateTexture(byte[] imageData, Texture2D texture, RawImage targetImage)
    {
        if (imageData == null || imageData.Length == 0)
        {
            return;
        }
        
        try
        {
            // 解碼 JPEG
            if (texture.LoadImage(imageData))
            {
                if (targetImage != null)
                {
                    targetImage.texture = texture;
                }
            }
            else
            {
                Debug.LogWarning("[CameraStreamReceiver] Failed to decode image");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"[CameraStreamReceiver] Error updating texture: {e.Message}");
        }
    }

    /// <summary>
    /// 更新 FPS 計算
    /// </summary>
    private void UpdateFpsCalculation()
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
    }

    /// <summary>
    /// 取得連接狀態
    /// </summary>
    public bool IsLeftConnected => leftConnected;
    public bool IsRightConnected => rightConnected;
    
    /// <summary>
    /// 取得 FPS
    /// </summary>
    public float LeftFps => leftFps;
    public float RightFps => rightFps;

    void OnDestroy()
    {
        // 清理 Texture
        if (leftTexture != null)
        {
            Destroy(leftTexture);
        }
        if (rightTexture != null)
        {
            Destroy(rightTexture);
        }
    }

    /// <summary>
    /// 在 Inspector 顯示狀態 (Editor only)
    /// </summary>
    void OnGUI()
    {
        #if UNITY_EDITOR
        if (!Application.isPlaying) return;
        
        GUILayout.BeginArea(new Rect(10, 10, 250, 100));
        GUILayout.Label($"Left Camera: {(leftConnected ? "Connected" : "Waiting...")} ({leftFps:F1} fps)");
        GUILayout.Label($"Right Camera: {(rightConnected ? "Connected" : "Waiting...")} ({rightFps:F1} fps)");
        GUILayout.Label($"Total Frames: L={leftFrameCount} R={rightFrameCount}");
        GUILayout.EndArea();
        #endif
    }
}
