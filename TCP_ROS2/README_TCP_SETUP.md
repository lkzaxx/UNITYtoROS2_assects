# Unity TCP ROS2 整合設置指南

## 概述

本專案已重構為使用 TCP 架構連接到 ROS2 系統，取代原本的 DDS 直接通信方式。

## 核心腳本架構

### 1. ROSTCPManager.cs (連線管理)
- **功能**: 統一的 TCP 連接、訊息處理、心跳功能
- **取代**: UnityRos2Talker.cs, StatusSubscriber.cs, CmdVelPublisher.cs
- **主要職責**:
  - TCP 連接管理和自動重連
  - 統一的訊息發送/接收接口
  - 心跳機制和狀態監控
  - 支援多個 topic 的發布/訂閱

### 2. ROSEnvironmentSetup.cs (環境設置)
- **功能**: TCP 環境配置和連接參數管理
- **取代**: 原本的 Ros2EnvironmentSetup.cs (DDS 版本)
- **主要職責**:
  - ROS-TCP-Connector 設定
  - 連接參數配置 (IP: 127.0.0.1, Port: 10000)
  - 環境檢查和診斷

### 3. ROSConnectionTest.cs (測試連線)
- **功能**: TCP 連接測試和診斷工具
- **取代**: NetworkConnectionTest.cs 和 Ros2DiagnosticTool.cs
- **主要職責**:
  - TCP 端口 10000 連通性測試
  - ROS2 服務狀態檢查
  - 實時連接監控和診斷

### 4. OpenArmController.cs (OpenArm 控制器)
- **功能**: 機械手臂控制，已重構為 TCP 架構
- **主要職責**:
  - 關節控制和狀態接收
  - OpenArm 專用邏輯
  - 與 ROSTCPManager 協作進行通信

## 設置步驟

### 1. 安裝 ROS-TCP-Connector
在 Unity Package Manager 中添加：
```
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
```

### 2. 配置 ROS Settings
1. 開啟 **Window > ROS Settings**
2. 設定參數：
   - **ROS IP Address**: `127.0.0.1`
   - **ROS Port**: `10000`
   - **Protocol**: `TCP`

### 3. 場景設置
1. 創建空的 GameObject 命名為 "ROSManager"
2. 添加以下腳本：
   - `ROSEnvironmentSetup.cs`
   - `ROSTCPManager.cs`
   - `ROSConnectionTest.cs`

3. 創建另一個 GameObject 命名為 "OpenArmController"
4. 添加 `OpenArmController.cs` 腳本

### 4. 參數配置
在 Inspector 中確認以下設定：

**ROSEnvironmentSetup**:
- ROS IP Address: `127.0.0.1`
- ROS Port: `10000`
- Connect On Start: `true`

**ROSTCPManager**:
- ROS IP Address: `127.0.0.1`
- ROS Port: `10000`
- Heartbeat Interval: `1.0`

**ROSConnectionTest**:
- ROS IP Address: `127.0.0.1`
- ROS TCP Port: `10000`
- Run Test On Start: `true`

## 主要 Topics

- `/unity/joint_commands` - 關節命令 (sensor_msgs/JointState)
- `/openarm/joint_states` - 關節狀態 (sensor_msgs/JointState)
- `/unity/heartbeat` - 心跳訊號 (std_msgs/String)
- `/unity/status` - 狀態訊息 (std_msgs/String)
- `/openarm/status` - OpenArm 狀態 (std_msgs/String)
- `/cmd_vel` - 速度命令 (geometry_msgs/Twist)

## 技術架構

```
Unity (Windows)
├── ROSEnvironmentSetup (配置)
├── ROSTCPManager ←→ TCP:10000 ←→ ROS2 (Container)
├── ROSConnectionTest (診斷)
└── OpenArmController (控制)
```

## ROS2 端設置

確保 ROS2 端運行以下服務：

### 1. 啟動 TCP Endpoint 伺服器
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
```

### 2. 啟動橋接節點
```bash
python3 /path/to/tcp_bridge_node.py
```

## 測試流程

### 1. 檢查連接
1. 運行 Unity 場景
2. 觀察 Console 輸出：
   ```
   🚀 ROSTCPManager 啟動...
   📡 設定 ROS 連接: 127.0.0.1:10000
   ✅ ROSTCPManager 初始化完成
   ```

### 2. 驗證心跳
在 ROS2 端檢查心跳：
```bash
ros2 topic echo /unity/heartbeat
```

### 3. 測試關節控制
在 Unity 中點擊 OpenArmController 的 "測試移動" 按鈕，然後在 ROS2 端檢查：
```bash
ros2 topic echo /unity/joint_commands
```

## 故障排除

### 常見問題

1. **TCP 連接失敗**
   - 確保 ROS TCP Endpoint 服務正在運行
   - 檢查端口 10000 是否被佔用
   - 驗證防火牆設定

2. **收不到心跳訊號**
   - 確認橋接節點正在運行
   - 檢查主題名稱是否正確
   - 驗證 Unity 發布設定

3. **關節命令無回應**
   - 確認 OpenArm 控制器正在運行
   - 檢查關節名稱映射
   - 驗證訊息格式

### 診斷工具

使用 `ROSConnectionTest.cs` 進行診斷：
1. 在 Unity 中點擊 "完整診斷" 按鈕
2. 觀察測試結果：
   - 網路連通: ✅/❌
   - TCP 端口: ✅/❌
   - ROS 連接: ✅/❌

## 已停用的舊腳本

以下腳本已重新命名並停用，功能已整合到新的核心腳本中：

- `Ros2EnvironmentSetup_DISABLED.cs` → `ROSEnvironmentSetup.cs`
- `CmdVelPublisher_DISABLED.cs` → `ROSTCPManager.cs`
- `UnityRos2Talker_DISABLED.cs` → `ROSTCPManager.cs`
- `StatusSubscriber_DISABLED.cs` → `ROSTCPManager.cs`
- `NetworkConnectionTest_DISABLED.cs` → `ROSConnectionTest.cs`
- `Ros2DiagnosticTool_DISABLED.cs` → `ROSConnectionTest.cs`

這些舊腳本可以安全刪除，但建議先確認新系統運作正常。
