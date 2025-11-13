# VR IP 配置界面使用指南

## 功能概述

在 Quest 3 VR 环境中，可以通过手柄点击虚拟界面来配置 ROS TCP 连接的 IP 地址和端口。

## 主要特性

- ✅ **VR 友好的 World Space UI**：界面以 3D 形式显示在 VR 空间中
- ✅ **手柄交互支持**：使用手柄射线点击按钮和输入框
- ✅ **虚拟键盘输入**：点击输入框时自动显示虚拟键盘
- ✅ **IP 地址验证**：自动验证输入的 IP 地址格式
- ✅ **动态创建或使用 Prefab**：支持两种方式创建界面

## 设置步骤

### 方式 1：使用动态创建（推荐，最简单）

1. 在 Unity Inspector 中找到 `ROSTCPManager` 组件
2. 在 **VR IP 配置界面** 区域：
   - ✅ 勾选 `Show IP Config UI`
   - 留空 `IP Config Canvas Prefab`（使用动态创建）
   - （可选）设置 `Virtual Keyboard Prefab`（如果已创建）
   - 调整 `UI Position` 和 `UI Scale` 以适应你的场景

3. 运行场景，界面会自动创建

### 方式 2：使用 Prefab

1. 创建 IP 配置 Canvas Prefab：
   - 创建 World Space Canvas
   - 添加必要的 UI 元素（InputField、Button 等）
   - 确保命名规范（见下方说明）

2. 创建虚拟键盘 Prefab：
   - 创建包含数字按钮（0-9）和功能按钮的 UI
   - 添加 `VirtualKeyboard` 组件
   - 按钮命名建议：
     - 数字按钮：`Key0`, `Key1`, ..., `Key9`
     - 点按钮：`Dot` 或 `Point`
     - 功能按钮：`Backspace`, `Clear`, `Confirm`, `Cancel`

3. 在 Inspector 中：
   - 将 Prefab 拖拽到 `IP Config Canvas Prefab` 字段
   - 将虚拟键盘 Prefab 拖拽到 `Virtual Keyboard Prefab` 字段

## 使用说明

### 显示/隐藏配置界面

- 点击 **"显示配置"** 按钮显示界面
- 点击 **"隐藏配置"** 按钮隐藏界面

### 配置 IP 地址

1. 点击 **IP 地址** 输入框
2. 如果设置了虚拟键盘 Prefab，会自动显示虚拟键盘
3. 使用手柄点击虚拟键盘上的数字和点（.）输入 IP 地址
4. 点击 **应用** 按钮保存配置
5. 系统会自动验证 IP 地址格式，如果无效会显示错误

### 配置端口

1. 点击 **端口** 输入框
2. 使用虚拟键盘输入端口号（仅数字）
3. 点击 **应用** 按钮保存配置

### 取消配置

- 点击 **取消** 按钮可以放弃当前修改，恢复原始值

## 代码说明

### VirtualKeyboard.cs

虚拟键盘组件，支持：
- 自动查找按钮（根据命名规则）
- 手动指定按钮引用
- 字符输入、删除、清空、确认、取消功能

### ROSTCPManager.cs 新增功能

- `InitializeIPConfigUI()`: 初始化 IP 配置界面
- `CreateIPConfigUI()`: 动态创建 UI 界面
- `ShowVirtualKeyboard()`: 显示虚拟键盘
- `OnApplyIPConfig()`: 应用配置并重新连接
- `IsValidIPAddress()`: 验证 IP 地址格式

## VR 交互要求

### 必需组件

1. **Graphic Raycaster**：Canvas 上需要添加此组件以支持射线交互
2. **XR Ray Interactor**：场景中需要设置 XR 射线交互器（通常由 XR Interaction Toolkit 提供）

### 手柄交互

- 使用手柄射线指向 UI 元素
- 按下扳机或确认按钮进行点击
- 输入框会自动响应选择事件并显示虚拟键盘

## 自定义界面位置

在 Inspector 中调整以下参数：

- **UI Position**: 界面在 3D 空间中的位置（默认：前方 2 米，高度 1.6 米）
- **UI Scale**: 界面缩放（默认：0.001，适合 VR）

## 故障排除

### 界面不显示

1. 检查 `Show IP Config UI` 是否勾选
2. 检查 Canvas 是否被正确创建
3. 查看 Console 是否有错误信息

### 手柄无法点击

1. 确认 Canvas 上有 `Graphic Raycaster` 组件
2. 确认场景中有 XR Ray Interactor
3. 检查 Canvas 的 `Event Camera` 是否设置正确

### 虚拟键盘不显示

1. 检查是否设置了 `Virtual Keyboard Prefab`
2. 如果没有 Prefab，在 Android 上会尝试使用系统键盘（可能不可用）
3. 建议创建自定义虚拟键盘 Prefab

### IP 地址验证失败

- 确保 IP 地址格式为：`xxx.xxx.xxx.xxx`（四个 0-255 的数字）
- 例如：`192.168.1.100` ✅
- 例如：`127.0.0.1` ✅
- 例如：`192.168.1` ❌（缺少第四段）

## 示例场景设置

```
Hierarchy
├── XR Origin (XR Interaction Toolkit)
│   ├── Camera Offset
│   │   └── Main Camera
│   └── Left/Right Controller (with XR Ray Interactor)
│
└── ROSManager
    └── ROSTCPManager (脚本)
        ├── Show IP Config UI: ✓
        ├── IP Config Canvas Prefab: (留空，使用动态创建)
        └── Virtual Keyboard Prefab: (可选)
```

## 注意事项

1. **TextMeshPro 依赖**：需要安装 TextMeshPro 包
2. **VR 性能**：World Space Canvas 可能影响性能，建议优化 UI 元素数量
3. **网络权限**：Android 打包时需要确保有 INTERNET 权限
4. **IP 地址**：在 Quest 3 上，`127.0.0.1` 无法连接到外部服务器，需要使用实际网络 IP

## 下一步

- 创建自定义虚拟键盘 Prefab 以获得更好的用户体验
- 调整界面样式和布局以适应你的应用风格
- 添加更多配置选项（如连接超时、重连间隔等）

