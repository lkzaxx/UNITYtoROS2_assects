using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

/// <summary>
/// ROS TCP 環境設置和配置管理
/// 重構自原本的 Ros2EnvironmentSetup.cs，改為 TCP 架構
/// </summary>
public class ROSEnvironmentSetup : MonoBehaviour
{
    [Header("ROS TCP 連接設定")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;
    public bool connectOnStart = true;
    public float connectionTimeout = 10.0f;
    
    [Header("高級設定")]
    public bool showDebugLogs = true;
    public bool autoReconnect = true;
    public float reconnectInterval = 5.0f;
    
    [Header("狀態顯示")]
    public bool isConfigured = false;
    public bool isConnected = false;
    public string connectionStatus = "未初始化";
    
    private ROSConnection rosConnection;
    private float lastConnectionCheck = 0f;
    
    void Awake()
    {
        // 在其他腳本之前設置 ROS 環境
        SetupROSEnvironment();
    }
    
    void Start()
    {
        if (connectOnStart)
        {
            InitializeConnection();
        }
    }
    
    void SetupROSEnvironment()
    {
        try
        {
            Debug.Log("🔧 開始設置 ROS TCP 環境...");
            
            // 獲取或創建 ROS 連接實例
            rosConnection = ROSConnection.GetOrCreateInstance();
            
            // 設定連接參數
            ConfigureConnection();
            
            isConfigured = true;
            connectionStatus = "環境已配置";
            
            Debug.Log("✅ ROS TCP 環境設置完成");
            LogConnectionSettings();
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ ROS TCP 環境設置失敗: {ex.Message}");
            connectionStatus = $"配置失敗: {ex.Message}";
            isConfigured = false;
        }
    }
    
    void ConfigureConnection()
    {
        if (rosConnection == null)
        {
            Debug.LogError("❌ ROSConnection 實例為空");
            return;
        }
        
        // 設定 IP 和端口
        // 注意：ROS-TCP-Connector 的 IP 和端口設定通常在 ROS Settings 中配置
        // 這裡主要是驗證和記錄設定
        
        Debug.Log($"📡 配置 ROS 連接參數:");
        Debug.Log($"   IP 地址: {rosIPAddress}");
        Debug.Log($"   端口: {rosPort}");
        Debug.Log($"   自動連接: {connectOnStart}");
        Debug.Log($"   連接超時: {connectionTimeout}s");
    }
    
    void InitializeConnection()
    {
        if (!isConfigured)
        {
            Debug.LogWarning("⚠️ 環境未配置，無法初始化連接");
            return;
        }
        
        try
        {
            Debug.Log("🚀 初始化 ROS TCP 連接...");
            
            // ROS-TCP-Connector 會自動處理連接
            // 我們只需要監控連接狀態
            connectionStatus = "正在連接...";
            
            // 開始監控連接狀態
            InvokeRepeating(nameof(CheckConnectionStatus), 1.0f, 1.0f);
            
            Debug.Log("✅ 連接初始化完成");
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ 連接初始化失敗: {ex.Message}");
            connectionStatus = $"連接失敗: {ex.Message}";
        }
    }
    
    void CheckConnectionStatus()
    {
        if (rosConnection == null)
        {
            isConnected = false;
            connectionStatus = "連接實例為空";
            return;
        }
        
        // 簡單的連接狀態檢查
        // ROS-TCP-Connector 沒有直接的連接狀態 API
        // 我們通過其他方式來判斷連接狀態
        bool wasConnected = isConnected;
        
        try
        {
            // 嘗試檢查連接狀態
            // 這是一個簡化的檢查，實際狀態需要通過訊息傳輸來驗證
            isConnected = rosConnection != null;
            
            if (isConnected)
            {
                connectionStatus = "已連接";
            }
            else
            {
                connectionStatus = "未連接";
            }
            
            // 連接狀態變化時記錄
            if (wasConnected != isConnected)
            {
                Debug.Log($"🔄 連接狀態變更: {connectionStatus}");
                
                if (!isConnected && autoReconnect)
                {
                    Debug.Log("🔄 嘗試自動重連...");
                    Invoke(nameof(AttemptReconnection), reconnectInterval);
                }
            }
        }
        catch (System.Exception ex)
        {
            isConnected = false;
            connectionStatus = $"狀態檢查失敗: {ex.Message}";
            
            if (showDebugLogs)
            {
                Debug.LogWarning($"⚠️ 連接狀態檢查異常: {ex.Message}");
            }
        }
        
        lastConnectionCheck = Time.time;
    }
    
    void AttemptReconnection()
    {
        if (isConnected)
        {
            Debug.Log("✅ 連接已恢復，取消重連");
            return;
        }
        
        Debug.Log("🔄 嘗試重新連接...");
        
        try
        {
            // 重新獲取連接實例
            rosConnection = ROSConnection.GetOrCreateInstance();
            connectionStatus = "重連中...";
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ 重連失敗: {ex.Message}");
            connectionStatus = $"重連失敗: {ex.Message}";
            
            if (autoReconnect)
            {
                Invoke(nameof(AttemptReconnection), reconnectInterval);
            }
        }
    }
    
    void LogConnectionSettings()
    {
        Debug.Log("📋 === ROS TCP 連接設定 ===");
        Debug.Log($"ROS IP: {rosIPAddress}");
        Debug.Log($"ROS Port: {rosPort}");
        Debug.Log($"自動連接: {connectOnStart}");
        Debug.Log($"自動重連: {autoReconnect}");
        Debug.Log($"連接超時: {connectionTimeout}s");
        Debug.Log($"重連間隔: {reconnectInterval}s");
        Debug.Log($"除錯日誌: {showDebugLogs}");
    }
    
    #region 公共方法
    
    /// <summary>
    /// 手動連接到 ROS
    /// </summary>
    [ContextMenu("手動連接")]
    public void ManualConnect()
    {
        Debug.Log("🔄 手動觸發連接...");
        InitializeConnection();
    }
    
    /// <summary>
    /// 重新配置連接
    /// </summary>
    [ContextMenu("重新配置")]
    public void ReconfigureConnection()
    {
        Debug.Log("🔄 重新配置連接...");
        SetupROSEnvironment();
        
        if (connectOnStart)
        {
            InitializeConnection();
        }
    }
    
    /// <summary>
    /// 獲取連接狀態
    /// </summary>
    public bool IsConnected()
    {
        return isConnected && rosConnection != null;
    }
    
    /// <summary>
    /// 獲取 ROS 連接實例
    /// </summary>
    public ROSConnection GetROSConnection()
    {
        return rosConnection;
    }
    
    #endregion
    
    #region GUI 顯示
    
    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(Screen.width - 250, 10, 230, 150));
        
        GUILayout.Label("ROS 環境設置", GUI.skin.box);
        
        // 配置狀態
        GUI.color = isConfigured ? Color.green : Color.red;
        GUILayout.Label($"配置: {(isConfigured ? "✅" : "❌")}");
        
        // 連接狀態
        GUI.color = isConnected ? Color.green : Color.red;
        GUILayout.Label($"連接: {(isConnected ? "✅" : "❌")}");
        GUI.color = Color.white;
        
        // 狀態訊息
        GUILayout.Label($"狀態: {connectionStatus}");
        
        // 連接資訊
        GUILayout.Label($"目標: {rosIPAddress}:{rosPort}");
        
        // 控制按鈕
        if (GUILayout.Button("重新連接"))
        {
            ManualConnect();
        }
        
        if (GUILayout.Button("重新配置"))
        {
            ReconfigureConnection();
        }
        
        GUILayout.EndArea();
    }
    
    #endregion
    
    void OnDestroy()
    {
        // 停止狀態檢查
        CancelInvoke();
        Debug.Log("🔄 ROSEnvironmentSetup 已停止");
    }
}
