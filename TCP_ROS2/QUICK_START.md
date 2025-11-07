# Unity TCP ROS2 快速設置指南

## 📋 腳本掛載步驟

### 步驟 1: 創建 ROSManager GameObject

1. 在 Hierarchy 中右鍵 → **Create Empty**
2. 命名為 **"ROSManager"**
3. 按照以下**順序**添加腳本（重要！）：

#### 掛載順序（從上到下）：

```
ROSManager (GameObject)
├── 1. ROSEnvironmentSetup.cs      ← 最先掛載（環境配置）
├── 2. ROSTCPManager.cs            ← 第二個（連接管理）
└── 3. ROSConnectionTest.cs        ← 最後（診斷工具）
```

**為什麼這個順序很重要？**
- `ROSEnvironmentSetup` 需要在最前面，因為它設置 ROS 環境
- `ROSTCPManager` 依賴環境設置完成後才能初始化
- `ROSConnectionTest` 需要檢查前兩者的狀態

### 步驟 2: 配置 ROSManager 參數

在 Inspector 中檢查並設定：

#### ROSEnvironmentSetup 組件：
- ✅ **ROS IP Address**: `127.0.0.1`
- ✅ **ROS Port**: `10000`
- ✅ **Connect On Start**: `✓` (勾選)
- ✅ **Auto Reconnect**: `✓` (勾選)

#### ROSTCPManager 組件：
- ✅ **ROS IP Address**: `127.0.0.1`
- ✅ **ROS Port**: `10000`
- ✅ **Heartbeat Interval**: `1.0`
- ✅ **Is Heartbeat Active**: `✓` (勾選)

#### ROSConnectionTest 組件：
- ✅ **ROS IP Address**: `127.0.0.1`
- ✅ **ROS TCP Port**: `10000`
- ✅ **Run Test On Start**: `✓` (勾選)

### 步驟 3: 創建 OpenArmController GameObject

1. 在 Hierarchy 中右鍵 → **Create Empty**
2. 命名為 **"OpenArmController"**
3. 添加腳本：
   - `OpenArmController.cs` (從 `Scripts/` 資料夾)

### 步驟 4: 配置 OpenArmController 參數

在 Inspector 中檢查：

#### OpenArmController 組件：
- ✅ **Joint Commands Topic**: `/unity/joint_commands`
- ✅ **Joint States Topic**: `/openarm/joint_states`
- ✅ **Status Topic**: `/openarm/status`
- ✅ **Joint Names**: 確認有 6 個關節名稱（預設：joint1~joint6）

## 🎯 最終場景結構

```
Hierarchy
├── ROSManager
│   ├── ROSEnvironmentSetup.cs
│   ├── ROSTCPManager.cs
│   └── ROSConnectionTest.cs
│
└── OpenArmController
    └── OpenArmController.cs
```

## ✅ 驗證設置

### 1. 檢查腳本順序
在 ROSManager 的 Inspector 中，確認腳本順序為：
1. ROSEnvironmentSetup
2. ROSTCPManager  
3. ROSConnectionTest

### 2. 運行場景測試
1. 點擊 **Play** 按鈕
2. 觀察 Console 輸出，應該看到：
   ```
   🔧 開始設置 ROS TCP 環境...
   ✅ ROS TCP 環境設置完成
   🚀 ROSTCPManager 啟動...
   ✅ ROSTCPManager 初始化完成
   🔍 ROSConnectionTest 啟動...
   ```

### 3. 檢查 GUI 顯示
運行時應該在螢幕上看到：
- **左上角**: ROS TCP 連接管理器狀態
- **右上角**: ROS 環境設置狀態
- **右上角下方**: ROS 連接診斷狀態
- **右下角**: OpenArm 控制器狀態

## 🔧 故障排除

### 如果連接失敗：
1. 確認 ROS2 TCP Endpoint 服務正在運行
2. 檢查端口 10000 是否被佔用
3. 使用 ROSConnectionTest 的「完整診斷」按鈕

### 如果找不到 ROSTCPManager：
- 確認 ROSManager GameObject 存在
- 確認 ROSTCPManager.cs 已掛載
- 檢查腳本順序是否正確

## 📝 重要提醒

1. **腳本順序很重要**：必須按照上述順序掛載
2. **IP 和端口必須一致**：所有組件都使用 `127.0.0.1:10000`
3. **ROS2 服務必須先運行**：確保 Docker 容器中的 TCP Endpoint 已啟動
4. **檢查 Console 輸出**：所有錯誤和警告都會顯示在 Console 中

## 🚀 下一步

設置完成後，參考 `README_TCP_SETUP.md` 了解：
- 如何測試連接
- 如何發送關節命令
- 如何監控狀態
- 完整的故障排除指南
