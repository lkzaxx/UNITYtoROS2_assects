using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Threading.Tasks;
using System.Collections;

/// <summary>
/// ROS TCP 連接測試和診斷工具
/// 整合 NetworkConnectionTest.cs 和 Ros2DiagnosticTool.cs 功能
/// </summary>
public class ROSConnectionTest : MonoBehaviour
{
    [Header("測試設定")]
    public string rosIPAddress = "127.0.0.1";
    public int rosTCPPort = 10000;
    public bool runTestOnStart = true;
    public float testInterval = 30.0f; // 定期測試間隔
    
    [Header("測試結果")]
    public bool networkReachable = false;
    public bool tcpPortOpen = false;
    public bool rosConnectionActive = false;
    public string lastTestResult = "";
    public float lastTestTime = 0f;
    
    [Header("診斷資訊")]
    public int pingTime = -1;
    public string connectionStatus = "未測試";
    public int testCount = 0;
    
    private ROSConnection rosConnection;
    private bool isTestingInProgress = false;
    
    void Start()
    {
        Debug.Log("🔍 ROSConnectionTest 啟動...");
        
        if (runTestOnStart)
        {
            // 延遲一點開始測試，讓其他組件先初始化
            Invoke(nameof(RunFullDiagnostic), 2.0f);
        }
        
        // 定期測試
        if (testInterval > 0)
        {
            InvokeRepeating(nameof(RunPeriodicTest), testInterval, testInterval);
        }
    }
    
    #region 完整診斷
    
    [ContextMenu("執行完整診斷")]
    public async void RunFullDiagnostic()
    {
        if (isTestingInProgress)
        {
            Debug.LogWarning("⚠️ 測試正在進行中，請稍候...");
            return;
        }
        
        isTestingInProgress = true;
        testCount++;
        lastTestTime = Time.time;
        
        Debug.Log("🔍 === ROS TCP 完整診斷開始 ===");
        
        try
        {
            // 1. 網路連通性測試
            await TestNetworkConnectivity();
            
            // 2. TCP 端口測試
            await TestTCPPort();
            
            // 3. ROS 連接狀態檢查
            TestROSConnectionStatus();
            
            // 4. 提供診斷總結
            ProvideDiagnosticSummary();
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"❌ 診斷過程發生異常: {ex.Message}");
            lastTestResult = $"診斷異常: {ex.Message}";
        }
        finally
        {
            isTestingInProgress = false;
            Debug.Log("🔍 === ROS TCP 完整診斷結束 ===");
        }
    }
    
    void RunPeriodicTest()
    {
        if (!isTestingInProgress)
        {
            Debug.Log("🔄 執行定期連接測試...");
            RunFullDiagnostic();
        }
    }
    
    #endregion
    
    #region 網路連通性測試
    
    async Task TestNetworkConnectivity()
    {
        Debug.Log("📡 測試網路連通性...");
        
        try
        {
            System.Net.NetworkInformation.Ping ping = new System.Net.NetworkInformation.Ping();
            PingReply reply = await Task.Run(() => ping.Send(rosIPAddress, 5000));
            
            if (reply.Status == IPStatus.Success)
            {
                networkReachable = true;
                pingTime = (int)reply.RoundtripTime;
                string result = $"✅ Ping 成功: {rosIPAddress} ({pingTime}ms)";
                Debug.Log(result);
                lastTestResult = result;
            }
            else
            {
                networkReachable = false;
                pingTime = -1;
                string result = $"❌ Ping 失敗: {rosIPAddress} - {reply.Status}";
                Debug.LogWarning(result);
                lastTestResult = result;
            }
        }
        catch (System.Exception ex)
        {
            networkReachable = false;
            pingTime = -1;
            string result = $"❌ Ping 異常: {ex.Message}";
            Debug.LogError(result);
            lastTestResult = result;
        }
    }
    
    #endregion
    
    #region TCP 端口測試
    
    async Task TestTCPPort()
    {
        Debug.Log($"🔌 測試 TCP 端口 {rosTCPPort}...");
        
        try
        {
            using (TcpClient tcpClient = new TcpClient())
            {
                // 設定超時
                tcpClient.ReceiveTimeout = 3000;
                tcpClient.SendTimeout = 3000;
                
                // 嘗試連接
                var connectTask = tcpClient.ConnectAsync(rosIPAddress, rosTCPPort);
                var timeoutTask = Task.Delay(3000);
                
                var completedTask = await Task.WhenAny(connectTask, timeoutTask);
                
                if (completedTask == connectTask && tcpClient.Connected)
                {
                    tcpPortOpen = true;
                    Debug.Log($"✅ TCP 端口 {rosTCPPort} 開放且可連接");
                    lastTestResult = $"TCP 端口 {rosTCPPort} 連接成功";
                }
                else
                {
                    tcpPortOpen = false;
                    Debug.LogWarning($"⚠️ TCP 端口 {rosTCPPort} 連接超時或失敗");
                    lastTestResult = $"TCP 端口 {rosTCPPort} 連接失敗";
                }
            }
        }
        catch (SocketException ex)
        {
            tcpPortOpen = false;
            string result = $"❌ TCP 端口測試失敗: {ex.SocketErrorCode}";
            Debug.LogError(result);
            lastTestResult = result;
            
            // 提供具體的錯誤診斷
            switch (ex.SocketErrorCode)
            {
                case SocketError.ConnectionRefused:
                    Debug.LogError("🔍 連接被拒絕 - ROS TCP Endpoint 可能未啟動");
                    break;
                case SocketError.TimedOut:
                    Debug.LogError("🔍 連接超時 - 檢查網路或防火牆設定");
                    break;
                case SocketError.HostUnreachable:
                    Debug.LogError("🔍 主機無法到達 - 檢查 IP 地址和網路連接");
                    break;
            }
        }
        catch (System.Exception ex)
        {
            tcpPortOpen = false;
            string result = $"❌ TCP 端口測試異常: {ex.Message}";
            Debug.LogError(result);
            lastTestResult = result;
        }
    }
    
    #endregion
    
    #region ROS 連接狀態檢查
    
    void TestROSConnectionStatus()
    {
        Debug.Log("🤖 檢查 ROS 連接狀態...");
        
        try
        {
            // 獲取 ROS 連接實例
            rosConnection = ROSConnection.GetOrCreateInstance();
            
            if (rosConnection != null)
            {
                rosConnectionActive = true;
                connectionStatus = "ROS 連接實例存在";
                Debug.Log("✅ ROS 連接實例正常");
                
                // 檢查是否有其他 ROS 相關組件
                CheckROSComponents();
            }
            else
            {
                rosConnectionActive = false;
                connectionStatus = "ROS 連接實例為空";
                Debug.LogWarning("⚠️ ROS 連接實例不存在");
            }
        }
        catch (System.Exception ex)
        {
            rosConnectionActive = false;
            connectionStatus = $"ROS 連接檢查失敗: {ex.Message}";
            Debug.LogError($"❌ ROS 連接狀態檢查失敗: {ex.Message}");
        }
    }
    
    void CheckROSComponents()
    {
        // 檢查場景中的 ROS 相關組件
        var tcpManagers = FindObjectsByType<ROSTCPManager>(FindObjectsSortMode.None);
        var environmentSetups = FindObjectsByType<ROSEnvironmentSetup>(FindObjectsSortMode.None);
        var openArmControllers = FindObjectsByType<OpenArmController>(FindObjectsSortMode.None);
        
        Debug.Log($"📊 場景中的 ROS 組件:");
        Debug.Log($"   ROSTCPManager: {tcpManagers.Length} 個");
        Debug.Log($"   ROSEnvironmentSetup: {environmentSetups.Length} 個");
        Debug.Log($"   OpenArmController: {openArmControllers.Length} 個");
        
        if (tcpManagers.Length > 1)
        {
            Debug.LogWarning("⚠️ 發現多個 ROSTCPManager，可能造成衝突");
        }
        
        if (environmentSetups.Length > 1)
        {
            Debug.LogWarning("⚠️ 發現多個 ROSEnvironmentSetup，可能造成衝突");
        }
    }
    
    #endregion
    
    #region 診斷總結
    
    void ProvideDiagnosticSummary()
    {
        Debug.Log("📊 === 診斷總結 ===");
        
        bool allTestsPassed = networkReachable && tcpPortOpen && rosConnectionActive;
        
        if (allTestsPassed)
        {
            Debug.Log("🎉 所有測試通過！ROS TCP 連接應該可以正常工作");
            connectionStatus = "所有測試通過";
            
            Debug.Log("💡 建議:");
            Debug.Log("   1. 確保 ROS TCP Endpoint 服務正在運行");
            Debug.Log("   2. 在 Unity 中啟用 ROSTCPManager");
            Debug.Log("   3. 觀察 Console 輸出確認訊息傳輸");
        }
        else
        {
            Debug.LogWarning("⚠️ 發現問題，需要修復:");
            
            if (!networkReachable)
            {
                Debug.LogError("   ❌ 網路連通性問題");
                Debug.LogError("      - 檢查 IP 地址是否正確");
                Debug.LogError("      - 確認目標主機是否可達");
            }
            
            if (!tcpPortOpen)
            {
                Debug.LogError("   ❌ TCP 端口連接問題");
                Debug.LogError("      - 確認 ROS TCP Endpoint 服務正在運行");
                Debug.LogError("      - 檢查端口號是否正確 (預設 10000)");
                Debug.LogError("      - 檢查防火牆設定");
            }
            
            if (!rosConnectionActive)
            {
                Debug.LogError("   ❌ ROS 連接組件問題");
                Debug.LogError("      - 確認已安裝 ROS-TCP-Connector 套件");
                Debug.LogError("      - 檢查 ROS Settings 配置");
            }
            
            connectionStatus = "測試未完全通過";
            
            Debug.Log("🔧 建議修復步驟:");
            Debug.Log("   1. 確認 Docker 容器正在運行");
            Debug.Log("   2. 啟動 ROS TCP Endpoint 服務");
            Debug.Log("   3. 檢查網路和防火牆設定");
            Debug.Log("   4. 重新執行診斷測試");
        }
    }
    
    #endregion
    
    #region 公共方法
    
    /// <summary>
    /// 快速連接測試
    /// </summary>
    public async void QuickConnectionTest()
    {
        Debug.Log("⚡ 執行快速連接測試...");
        await TestTCPPort();
    }
    
    /// <summary>
    /// 獲取測試結果摘要
    /// </summary>
    public string GetTestSummary()
    {
        return $"網路:{(networkReachable ? "✅" : "❌")} TCP:{(tcpPortOpen ? "✅" : "❌")} ROS:{(rosConnectionActive ? "✅" : "❌")}";
    }
    
    #endregion
    
    #region GUI 顯示
    
    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(Screen.width - 280, 170, 260, 200));
        
        GUILayout.Label("ROS 連接診斷", GUI.skin.box);
        
        // 測試結果
        GUI.color = networkReachable ? Color.green : Color.red;
        GUILayout.Label($"網路連通: {(networkReachable ? "✅" : "❌")} ({pingTime}ms)");
        
        GUI.color = tcpPortOpen ? Color.green : Color.red;
        GUILayout.Label($"TCP 端口: {(tcpPortOpen ? "✅" : "❌")}");
        
        GUI.color = rosConnectionActive ? Color.green : Color.red;
        GUILayout.Label($"ROS 連接: {(rosConnectionActive ? "✅" : "❌")}");
        GUI.color = Color.white;
        
        // 狀態資訊
        GUILayout.Label($"狀態: {connectionStatus}");
        GUILayout.Label($"測試次數: {testCount}");
        
        if (lastTestTime > 0)
        {
            float timeSinceTest = Time.time - lastTestTime;
            GUILayout.Label($"上次測試: {timeSinceTest:F1}s 前");
        }
        
        // 控制按鈕
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("完整診斷"))
        {
            RunFullDiagnostic();
        }
        
        if (GUILayout.Button("快速測試"))
        {
            QuickConnectionTest();
        }
        GUILayout.EndHorizontal();
        
        if (isTestingInProgress)
        {
            GUI.color = Color.yellow;
            GUILayout.Label("🔄 測試進行中...");
            GUI.color = Color.white;
        }
        
        GUILayout.EndArea();
    }
    
    #endregion
    
    void OnDestroy()
    {
        CancelInvoke();
        Debug.Log("🔄 ROSConnectionTest 已停止");
    }
}
