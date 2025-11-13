# XR Ray Interactor 設置指南

## 概述

XR Ray Interactor 是讓 VR 手柄能夠與 UI 元素交互的關鍵組件。本指南將詳細說明如何設置。

## 方法 1：使用 XR Interaction Toolkit（推薦）

### 步驟 1：安裝 XR Interaction Toolkit

1. 打開 Unity Package Manager
   - **Window** → **Package Manager**

2. 切換到 **Unity Registry**

3. 搜索 **"XR Interaction Toolkit"**

4. 點擊 **Install** 安裝

5. 安裝完成後，會提示導入示例，可以選擇 **Import** 或 **Skip**

### 步驟 2：檢查場景中是否有 XR Origin

1. 在 **Hierarchy** 窗口中查找：
   - `XR Origin` 或
   - `[BuildingBlock] Camera Rig` 或
   - 任何包含 "XR" 或 "Camera Rig" 的對象

2. **如果已有 XR Origin**：
   - 跳轉到步驟 3

3. **如果沒有 XR Origin**：
   - 繼續下面的步驟創建

### 步驟 3：創建或設置 XR Origin

#### 選項 A：使用 XR Interaction Toolkit 的預設

1. 在 Hierarchy 中右鍵 → **XR** → **XR Origin (VR)**

2. 這會自動創建：
   - `XR Origin` 對象
   - `Camera Offset` 子對象
   - `Main Camera` 子對象
   - `LeftHand Controller` 子對象
   - `RightHand Controller` 子對象

#### 選項 B：手動添加到現有的 Camera Rig

如果你的場景中已有 `[BuildingBlock] Camera Rig`：

1. 找到 Controller 對象（通常是 `LeftHand` 和 `RightHand`）

2. 在每個 Controller 上添加組件：
   - **Add Component** → 搜索 **"XR Ray Interactor"**
   - 添加 **XR Ray Interactor** 組件

### 步驟 4：配置 XR Ray Interactor

對於每個 Controller（左手和右手）：

1. 選中 Controller 對象

2. 在 Inspector 中，找到 **XR Ray Interactor** 組件

3. 檢查以下設置：
   - **Ray Origin Transform**: 應該指向 Controller 的 Transform
   - **Max Raycast Distance**: 建議設置為 `10` 或更大
   - **Raycast Mask**: 確保包含 **UI** 圖層
   - **Line Type**: 可以選擇 **Straight Line** 或 **Projectile Curve**

4. **重要**：確保 **Interaction Layer Mask** 包含 UI 圖層

### 步驟 5：配置 XR Interaction Manager

1. 在 Hierarchy 中查找 **XR Interaction Manager**
   - 如果沒有，創建一個：
     - 右鍵 → **Create Empty** → 命名為 `XR Interaction Manager`
     - 添加組件：**Add Component** → **XR Interaction Manager**

2. 在每個 **XR Ray Interactor** 組件中：
   - 將 **Interaction Manager** 字段設置為剛才創建的 XR Interaction Manager

## 方法 2：使用 Unity UI 的 Graphic Raycaster（簡單方法）

如果你的場景中沒有 XR Interaction Toolkit，可以使用更簡單的方法：

### 步驟 1：確保 Canvas 設置正確

1. 找到 `IPConfigCanvas` 對象（由 ROSTCPManager 自動創建）

2. 確認有以下組件：
   - ✅ **Canvas**（Render Mode: World Space）
   - ✅ **Graphic Raycaster**
   - ✅ **Canvas Scaler**

### 步驟 2：確保有 EventSystem

1. 在 Hierarchy 中查找 **EventSystem**
   - 如果沒有，ROSTCPManager 會自動創建
   - 如果已有，確保它處於激活狀態

2. EventSystem 應該有：
   - **Event System** 組件
   - **Standalone Input Module** 組件

### 步驟 3：添加 XR UI Input Module（如果使用 XR）

1. 在 EventSystem 上添加組件：
   - **Add Component** → 搜索 **"XR UI Input Module"**

2. 如果找不到，可能需要：
   - 安裝 **XR Plugin Management** 包
   - 或使用 **Standalone Input Module**（已自動添加）

## 方法 3：手動添加 XR Ray Interactor（不使用 Toolkit）

如果你不想安裝 XR Interaction Toolkit，可以手動創建：

### 步驟 1：創建射線對象

1. 在 Controller 下創建子對象：
   - 右鍵 Controller → **Create Empty** → 命名為 `Ray Origin`

2. 設置位置：
   - **Position**: `(0, 0, 0)`（相對於 Controller）
   - 或稍微向前：`(0, 0, 0.1)`

### 步驟 2：添加射線組件

1. 在 Controller 上添加：
   - **Add Component** → **Line Renderer**（可選，用於視覺化射線）
   - **Add Component** → **Script** → 創建自定義射線檢測腳本

### 步驟 3：創建簡單的射線檢測腳本

創建新腳本 `SimpleVRRaycast.cs`：

```csharp
using UnityEngine;
using UnityEngine.EventSystems;

public class SimpleVRRaycast : MonoBehaviour
{
    public float maxDistance = 10f;
    public LayerMask uiLayer;
    
    void Update()
    {
        Ray ray = new Ray(transform.position, transform.forward);
        RaycastHit hit;
        
        if (Physics.Raycast(ray, out hit, maxDistance, uiLayer))
        {
            // 發送點擊事件到 UI
            PointerEventData pointerData = new PointerEventData(EventSystem.current);
            pointerData.position = hit.point;
            
            ExecuteEvents.Execute(hit.collider.gameObject, pointerData, 
                ExecuteEvents.pointerClickHandler);
        }
    }
}
```

## 快速檢查清單

### ✅ 必須有的組件：

1. **Canvas**（World Space）
   - ✅ Render Mode: World Space
   - ✅ Graphic Raycaster 組件

2. **EventSystem**
   - ✅ Event System 組件
   - ✅ Standalone Input Module 或 XR UI Input Module

3. **XR Ray Interactor**（如果使用 XR Interaction Toolkit）
   - ✅ 在 LeftHand Controller 上
   - ✅ 在 RightHand Controller 上
   - ✅ Interaction Manager 已設置
   - ✅ Raycast Mask 包含 UI 圖層

### 🔍 檢查方法：

1. **運行場景**
2. **使用手柄指向 UI**
3. **查看是否有射線顯示**（如果 Line Renderer 啟用）
4. **嘗試點擊按鈕**
5. **查看 Console 是否有錯誤**

## 常見問題

### Q: 手柄射線看不到？

**A:** 
- 檢查 XR Ray Interactor 的 **Line Renderer** 是否啟用
- 檢查 **Max Raycast Distance** 是否足夠大
- 確認 Controller 的位置和旋轉正確

### Q: 能看見射線但點擊沒反應？

**A:**
- 檢查 Canvas 的 **Graphic Raycaster** 是否存在
- 檢查 EventSystem 是否激活
- 確認 UI 元素的 **Raycast Target** 已勾選（默認是勾選的）
- 檢查 **Layer** 設置是否正確

### Q: 找不到 XR Interaction Toolkit？

**A:**
- 確認 Unity 版本支持（Unity 2020.3 或更高）
- 嘗試通過 **Window** → **Package Manager** → **Unity Registry** 搜索
- 或手動添加包：`com.unity.xr.interaction.toolkit`

### Q: 使用 OpenXR 還是其他 XR SDK？

**A:**
- XR Interaction Toolkit 支持多種 XR SDK
- 確保已安裝對應的 XR Plugin（如 OpenXR Plugin）
- 在 **Edit** → **Project Settings** → **XR Plug-in Management** 中啟用

## 推薦設置（Quest 3）

對於 Quest 3，推薦使用：

1. **XR Interaction Toolkit**（最新版本）
2. **OpenXR Plugin**
3. **XR Origin (VR)** 預設
4. **XR Ray Interactor** 在兩個 Controller 上

## 測試步驟

1. ✅ 安裝 XR Interaction Toolkit
2. ✅ 創建或確認 XR Origin 存在
3. ✅ 在 Controller 上添加 XR Ray Interactor
4. ✅ 配置 Interaction Manager
5. ✅ 運行場景
6. ✅ 使用手柄指向 UI
7. ✅ 按下扳機測試點擊

## 下一步

設置完成後：
- 測試手柄能否點擊 IP 配置界面的按鈕
- 測試能否選擇輸入框
- 如果設置了虛擬鍵盤，測試能否點擊鍵盤按鈕

如果遇到問題，請檢查 Console 的錯誤信息，並確認所有組件都已正確配置。

