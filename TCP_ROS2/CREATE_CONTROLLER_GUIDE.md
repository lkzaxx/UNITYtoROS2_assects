# 創建或查找 VR Controller 指南

## 方法 1：查找現有的 Controller

### 步驟 1：在 Hierarchy 中搜索

1. 在 Hierarchy 窗口頂部的搜索框輸入：
   - `Controller`
   - `Hand`
   - `XR`
   - `Left`
   - `Right`

2. 展開所有對象，查找可能包含 Controller 的對象：
   - `[BuildingBlock] Camera Rig`
   - `XR Origin`
   - `Camera Offset`
   - 任何包含 "Hand" 或 "Controller" 的對象

### 步驟 2：檢查 Camera Rig

如果你的場景中有 `[BuildingBlock] Camera Rig`：

1. 展開它，查看子對象
2. 可能的名稱：
   - `LeftHand`
   - `RightHand`
   - `Left Controller`
   - `Right Controller`
   - `LeftHand Controller`
   - `RightHand Controller`

3. 如果找到，直接使用（跳到"添加 XR Ray Interactor"步驟）

## 方法 2：創建 XR Origin（推薦，最簡單）

### 步驟 1：安裝 XR Interaction Toolkit（如果還沒安裝）

1. **Window** → **Package Manager**
2. 切換到 **Unity Registry**
3. 搜索 **"XR Interaction Toolkit"**
4. 點擊 **Install**

### 步驟 2：創建 XR Origin

1. 在 Hierarchy 中右鍵
2. 選擇 **XR** → **XR Origin (VR)**
3. 這會自動創建：
   - `XR Origin`
   - `Camera Offset` → `Main Camera`
   - `LeftHand Controller`（已包含 XR Ray Interactor）
   - `RightHand Controller`（已包含 XR Ray Interactor）

### 步驟 3：配置 Controller

1. 展開 `XR Origin`
2. 找到 `RightHand Controller` 或 `LeftHand Controller`
3. 選中它
4. 在 Inspector 中應該已經有 **XR Ray Interactor** 組件
5. 按照之前的指南設置 **Interaction Manager** 和 **Ray Origin Transform**

## 方法 3：手動創建 Controller（如果沒有 XR Interaction Toolkit）

### 步驟 1：創建 Controller 對象

1. 在 Hierarchy 中右鍵 → **Create Empty**
2. 命名為 `RightHand Controller`
3. 設置位置（可選）：
   - 放在 Camera Rig 下作為子對象
   - 或放在場景根目錄

### 步驟 2：添加 XR Ray Interactor

1. 選中 `RightHand Controller`
2. **Add Component** → 搜索 **"XR Ray Interactor"**
3. 如果找不到，說明 XR Interaction Toolkit 未安裝

### 步驟 3：配置設置

在 XR Ray Interactor 組件中：
- **Interaction Manager**: 拖拽 `XR Interaction Manager`
- **Ray Origin Transform**: 拖拽 `RightHand Controller` 自己
- **UI Interaction**: 勾選

## 方法 4：使用現有的 Camera Rig 結構

如果你的場景使用 `[BuildingBlock] Camera Rig`：

### 步驟 1：找到合適的位置

1. 展開 `[BuildingBlock] Camera Rig`
2. 查看其子對象結構
3. 找到合適的位置添加 Controller

### 步驟 2：創建 Controller

1. 在 Camera Rig 或 Camera Offset 下右鍵
2. **Create Empty** → 命名為 `RightHand Controller`
3. 設置 Transform：
   - **Position**: `(0.2, -0.3, 0.5)`（相對於 Camera）
   - **Rotation**: `(0, 0, 0)`

### 步驟 3：添加組件

1. **Add Component** → **XR Ray Interactor**
2. 配置設置（見方法 3 的步驟 3）

## 快速檢查清單

創建 Controller 後，確認：

- [ ] Controller 對象已創建
- [ ] XR Ray Interactor 組件已添加
- [ ] Interaction Manager 已設置
- [ ] Ray Origin Transform 已設置
- [ ] UI Interaction 已勾選

## 測試

1. 運行場景
2. 查看 Console：
   - `✅ 已配置 XR Ray Interactor: RightHand Controller`
3. 使用手柄指向 UI
4. 應該能看到射線

