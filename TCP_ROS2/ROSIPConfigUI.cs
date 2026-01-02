using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using TMPro;
using Unity.Robotics.ROSTCPConnector;

/// <summary>
/// VR IP 配置界面管理器
/// 從 ROSTCPManager 分離出來的獨立組件
/// 
/// 功能：
/// - 動態創建或從 Prefab 建立 IP 配置界面
/// - VR 手柄交互支持
/// - 虛擬鍵盤
/// </summary>
public class ROSIPConfigUI : MonoBehaviour
{
    #region Inspector 設定

    [Header("界面設定")]
    [Tooltip("是否啟用 IP 配置界面")]
    public bool enableUI = true;
    
    [Tooltip("IP 配置 Canvas Prefab（可選，留空則動態創建）")]
    public GameObject ipConfigCanvasPrefab;
    
    [Tooltip("虛擬鍵盤 Prefab（可選）")]
    public GameObject virtualKeyboardPrefab;
    
    [Tooltip("TextMeshPro 字體資源（必須指定！）")]
    public TMP_FontAsset tmpFont;
    
    [Tooltip("界面位置（相對於主攝像機）")]
    public Vector3 uiPosition = new Vector3(0, 1.6f, 2f);
    
    [Tooltip("界面縮放")]
    public Vector3 uiScale = new Vector3(0.001f, 0.001f, 0.001f);

    [Header("顯示用連接資訊（唯讀）")]
    [SerializeField] private string displayIPAddress = "192.168.0.15";
    [SerializeField] private int displayPort = 10000;

    #endregion

    #region 私有變數

    private GameObject ipConfigCanvasInstance;
    private TMP_InputField ipAddressInputField;
    private TMP_InputField portInputField;
    private Button applyButton;
    private Button cancelButton;
    private Button toggleButton;
    private VirtualKeyboard virtualKeyboard;
    private bool isIPConfigUIVisible = false;
    private string tempIPAddress;
    private int tempPort;

    #endregion

    #region Unity 生命週期

    void Start()
    {
        if (enableUI)
        {
            InitializeIPConfigUI();
        }
    }

    void OnDestroy()
    {
        if (ipConfigCanvasInstance != null)
        {
            Destroy(ipConfigCanvasInstance);
        }
    }

    #endregion

    #region 公開方法

    /// <summary>
    /// 從 ROSConnection 讀取實際的 IP/Port
    /// </summary>
    public void UpdateDisplayFromROSConnection()
    {
        var ros = ROSConnection.GetOrCreateInstance();
        if (ros == null) return;

        try
        {
            var rosType = ros.GetType();

            var ipField = rosType.GetField("m_RosIPAddress",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            if (ipField != null)
            {
                displayIPAddress = ipField.GetValue(ros) as string ?? displayIPAddress;
            }

            var portField = rosType.GetField("m_RosPort",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            if (portField != null)
            {
                displayPort = (int)portField.GetValue(ros);
            }

            Debug.Log($"📡 ROS 連接目標: {displayIPAddress}:{displayPort}（來自 Project Settings）");
        }
        catch (System.Exception ex)
        {
            Debug.LogWarning($"無法讀取 ROSConnection 連接信息: {ex.Message}");
        }
    }

    /// <summary>
    /// 切換界面顯示
    /// </summary>
    public void ToggleUI()
    {
        OnToggleIPConfigUI();
    }

    /// <summary>
    /// 取得顯示用 IP
    /// </summary>
    public string DisplayIPAddress => displayIPAddress;

    /// <summary>
    /// 取得顯示用 Port
    /// </summary>
    public int DisplayPort => displayPort;

    #endregion

    #region IP 配置界面

    void InitializeIPConfigUI()
    {
        // 如果提供了 Prefab，使用 Prefab
        if (ipConfigCanvasPrefab != null)
        {
            ipConfigCanvasInstance = Instantiate(ipConfigCanvasPrefab);
            SetupIPConfigUIFromPrefab();
        }
        else
        {
            // 否則動態創建
            CreateIPConfigUI();
        }

        // 從 ROSConnection 讀取實際連接資訊
        UpdateDisplayFromROSConnection();

        // 初始化臨時值
        tempIPAddress = displayIPAddress;
        tempPort = displayPort;

        // 更新界面顯示
        UpdateIPConfigUI();

        // 在 Play 模式下默認顯示界面
        if (ipConfigCanvasInstance != null)
        {
            ipConfigCanvasInstance.SetActive(true);
            isIPConfigUIVisible = true;

            Debug.Log($"✅ IP 配置界面已創建並顯示");
            Debug.Log($"   位置: {ipConfigCanvasInstance.transform.position}");
            Debug.Log($"   縮放: {ipConfigCanvasInstance.transform.localScale}");
        }
        else
        {
            Debug.LogError("❌ IP 配置界面創建失敗！");
        }
    }

    void SetupIPConfigUIFromPrefab()
    {
        // 查找組件
        ipAddressInputField = ipConfigCanvasInstance.GetComponentInChildren<TMP_InputField>();
        if (ipAddressInputField == null)
        {
            TMP_InputField[] inputs = ipConfigCanvasInstance.GetComponentsInChildren<TMP_InputField>();
            if (inputs.Length > 0) ipAddressInputField = inputs[0];
            if (inputs.Length > 1) portInputField = inputs[1];
        }

        Button[] buttons = ipConfigCanvasInstance.GetComponentsInChildren<Button>();
        foreach (Button btn in buttons)
        {
            string btnName = btn.name.ToLower();
            if (btnName.Contains("apply") || btnName.Contains("確認") || btnName.Contains("應用"))
                applyButton = btn;
            else if (btnName.Contains("cancel") || btnName.Contains("取消"))
                cancelButton = btn;
            else if (btnName.Contains("toggle") || btnName.Contains("顯示") || btnName.Contains("隱藏"))
                toggleButton = btn;
        }

        virtualKeyboard = ipConfigCanvasInstance.GetComponentInChildren<VirtualKeyboard>();

        // 綁定按鈕事件
        if (applyButton != null)
            applyButton.onClick.AddListener(OnApplyIPConfig);
        if (cancelButton != null)
            cancelButton.onClick.AddListener(OnCancelIPConfig);
        if (toggleButton != null)
            toggleButton.onClick.AddListener(OnToggleIPConfigUI);
    }

    void CreateIPConfigUI()
    {
        // 創建 Canvas（World Space，適合 VR）
        GameObject canvasObj = new GameObject("IPConfigCanvas");
        Canvas canvas = canvasObj.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;

        // 嘗試找到 XR Camera
        Camera xrCamera = Camera.main;
        if (xrCamera == null)
        {
            xrCamera = FindFirstObjectByType<Camera>();
        }
        canvas.worldCamera = xrCamera;

        // 添加 Canvas Scaler
        CanvasScaler scaler = canvasObj.AddComponent<CanvasScaler>();
        scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1920, 1080);

        // 添加 Graphic Raycaster
        canvasObj.AddComponent<GraphicRaycaster>();

        // 確保有 EventSystem
        if (FindFirstObjectByType<EventSystem>() == null)
        {
            GameObject eventSystemObj = new GameObject("EventSystem");
            eventSystemObj.AddComponent<EventSystem>();
            eventSystemObj.AddComponent<StandaloneInputModule>();
        }

        // 自動配置 XR Ray Interactor
        ConfigureXRRayInteractors();

        // 設置 Canvas 位置和縮放
        canvasObj.transform.position = uiPosition;
        canvasObj.transform.localScale = uiScale;

        // 創建背景面板
        GameObject panel = CreateUIElement("Panel", canvasObj.transform);
        Image panelImage = panel.AddComponent<Image>();
        panelImage.color = new Color(0.1f, 0.1f, 0.1f, 0.9f);
        SetRectTransform(panel, Vector2.zero, Vector2.one, Vector2.zero, Vector2.zero);

        // 創建標題
        CreateTextLabel(panel.transform, "Title", "ROS TCP Connection Config",
            new Vector2(0, 200), new Vector2(800, 60), 36, TextAlignmentOptions.Center);

        // 創建 IP 地址標籤和輸入框
        CreateTextLabel(panel.transform, "IPLabel", "IP Address:",
            new Vector2(-250, 120), new Vector2(150, 40), 24, TextAlignmentOptions.Left);

        GameObject ipInputObj = CreateInputField(panel.transform, "IPInput",
            new Vector2(0, 120), new Vector2(400, 50), displayIPAddress);
        ipAddressInputField = ipInputObj.GetComponent<TMP_InputField>();
        ipAddressInputField.onSelect.AddListener((string value) => ShowVirtualKeyboard(ipAddressInputField));

        // 創建端口標籤和輸入框
        CreateTextLabel(panel.transform, "PortLabel", "Port:",
            new Vector2(-250, 40), new Vector2(150, 40), 24, TextAlignmentOptions.Left);

        GameObject portInputObj = CreateInputField(panel.transform, "PortInput",
            new Vector2(0, 40), new Vector2(200, 50), displayPort.ToString());
        portInputField = portInputObj.GetComponent<TMP_InputField>();
        portInputField.contentType = TMP_InputField.ContentType.IntegerNumber;
        portInputField.onSelect.AddListener((string value) => ShowVirtualKeyboard(portInputField));

        // 創建按鈕
        applyButton = CreateButton(panel.transform, "ApplyButton", "Apply",
            new Vector2(-100, -60), new Vector2(150, 50), OnApplyIPConfig);

        cancelButton = CreateButton(panel.transform, "CancelButton", "Cancel",
            new Vector2(100, -60), new Vector2(150, 50), OnCancelIPConfig);

        toggleButton = CreateButton(panel.transform, "ToggleButton", "Show Config",
            new Vector2(0, -140), new Vector2(200, 50), OnToggleIPConfigUI);

        // 添加 VR 交互支持
        AddVRInteractionSupport(ipInputObj);
        AddVRInteractionSupport(portInputObj);
        AddVRInteractionSupport(applyButton.gameObject);
        AddVRInteractionSupport(cancelButton.gameObject);
        AddVRInteractionSupport(toggleButton.gameObject);

        ipConfigCanvasInstance = canvasObj;
    }

    #endregion

    #region UI 輔助方法

    GameObject CreateUIElement(string name, Transform parent)
    {
        GameObject obj = new GameObject(name);
        obj.transform.SetParent(parent, false);
        return obj;
    }

    void SetRectTransform(GameObject obj, Vector2 anchorMin, Vector2 anchorMax,
        Vector2 sizeDelta, Vector2 anchoredPosition)
    {
        RectTransform rect = obj.GetComponent<RectTransform>();
        if (rect == null) rect = obj.AddComponent<RectTransform>();
        rect.anchorMin = anchorMin;
        rect.anchorMax = anchorMax;
        rect.sizeDelta = sizeDelta;
        rect.anchoredPosition = anchoredPosition;
    }

    GameObject CreateTextLabel(Transform parent, string name, string text,
        Vector2 position, Vector2 size, int fontSize, TextAlignmentOptions alignment)
    {
        GameObject labelObj = CreateUIElement(name, parent);

        TextMeshProUGUI textComp = labelObj.AddComponent<TextMeshProUGUI>();
        textComp.text = text;
        textComp.fontSize = fontSize;
        textComp.alignment = alignment;
        textComp.color = Color.white;

        LoadTMPFont(textComp);
        SetRectTransform(labelObj, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), size, position);

        return labelObj;
    }

    void LoadTMPFont(TextMeshProUGUI textComponent)
    {
        if (textComponent == null) return;

        // 優先使用手動指定的字體
        if (tmpFont != null)
        {
            textComponent.font = tmpFont;
            return;
        }

        // 使用默認字體
        if (TMP_Settings.defaultFontAsset != null)
        {
            textComponent.font = TMP_Settings.defaultFontAsset;
            return;
        }

        // 嘗試載入常見的 TMP 字體
        string[] fontPaths = new string[]
        {
            "Fonts & Materials/LiberationSans SDF",
            "TextMesh Pro/Resources/Fonts & Materials/LiberationSans SDF"
        };

        foreach (string path in fontPaths)
        {
            TMP_FontAsset font = Resources.Load<TMP_FontAsset>(path);
            if (font != null)
            {
                textComponent.font = font;
                return;
            }
        }

        Debug.LogWarning("⚠️ 找不到 TextMeshPro 字體，請導入 TMP Essentials");
    }

    GameObject CreateInputField(Transform parent, string name,
        Vector2 position, Vector2 size, string placeholderText)
    {
        GameObject inputObj = CreateUIElement(name, parent);

        Image bgImage = inputObj.AddComponent<Image>();
        bgImage.color = new Color(0.2f, 0.2f, 0.2f, 1f);

        TMP_InputField inputField = inputObj.AddComponent<TMP_InputField>();
        SetRectTransform(inputObj, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), size, position);

        // 創建文字區域
        GameObject textArea = CreateUIElement("TextArea", inputObj.transform);
        RectTransform textAreaRect = textArea.AddComponent<RectTransform>();
        SetRectTransform(textArea, Vector2.zero, Vector2.one, Vector2.zero, Vector2.zero);

        // 創建文字組件
        GameObject textObj = CreateUIElement("Text", textArea.transform);
        TextMeshProUGUI textComp = textObj.AddComponent<TextMeshProUGUI>();
        textComp.text = "";
        textComp.fontSize = 24;
        textComp.color = Color.white;
        textComp.alignment = TextAlignmentOptions.MidlineLeft;
        LoadTMPFont(textComp);

        RectTransform textRect = textObj.GetComponent<RectTransform>();
        SetRectTransform(textObj, Vector2.zero, Vector2.one, Vector2.zero, Vector2.zero);
        textRect.offsetMin = new Vector2(10, 5);
        textRect.offsetMax = new Vector2(-10, -5);

        // 創建佔位符
        GameObject placeholderObj = CreateUIElement("Placeholder", textArea.transform);
        TextMeshProUGUI placeholderComp = placeholderObj.AddComponent<TextMeshProUGUI>();
        placeholderComp.text = placeholderText;
        placeholderComp.fontSize = 24;
        placeholderComp.color = new Color(0.5f, 0.5f, 0.5f, 1f);
        placeholderComp.alignment = TextAlignmentOptions.MidlineLeft;
        LoadTMPFont(placeholderComp);

        RectTransform placeholderRect = placeholderObj.GetComponent<RectTransform>();
        SetRectTransform(placeholderObj, Vector2.zero, Vector2.one, Vector2.zero, Vector2.zero);
        placeholderRect.offsetMin = new Vector2(10, 5);
        placeholderRect.offsetMax = new Vector2(-10, -5);

        inputField.textViewport = textAreaRect;
        inputField.textComponent = textComp;
        inputField.placeholder = placeholderComp;

        return inputObj;
    }

    Button CreateButton(Transform parent, string name, string text,
        Vector2 position, Vector2 size, UnityEngine.Events.UnityAction onClick)
    {
        GameObject buttonObj = CreateUIElement(name, parent);

        Image buttonImage = buttonObj.AddComponent<Image>();
        buttonImage.color = new Color(0.2f, 0.5f, 0.8f, 1f);

        Button button = buttonObj.AddComponent<Button>();
        button.onClick.AddListener(onClick);

        GameObject textObj = CreateUIElement("Text", buttonObj.transform);
        TextMeshProUGUI textComp = textObj.AddComponent<TextMeshProUGUI>();
        textComp.text = text;
        textComp.fontSize = 24;
        textComp.color = Color.white;
        textComp.alignment = TextAlignmentOptions.Center;
        LoadTMPFont(textComp);

        SetRectTransform(textObj, Vector2.zero, Vector2.one, Vector2.zero, Vector2.zero);
        SetRectTransform(buttonObj, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), size, position);

        return button;
    }

    #endregion

    #region XR 交互

    void ConfigureXRRayInteractors()
    {
#if UNITY_XR_INTERACTION_TOOLKIT
        // 原始的 XR Ray Interactor 配置邏輯
        Debug.Log("ℹ️ XR Interaction Toolkit 偵測到，正在配置...");
#else
        Debug.Log("ℹ️ XR Interaction Toolkit 未安裝");
#endif
    }

    void AddVRInteractionSupport(GameObject uiElement)
    {
        EventTrigger trigger = uiElement.GetComponent<EventTrigger>();
        if (trigger == null)
        {
            trigger = uiElement.AddComponent<EventTrigger>();
        }

        Button btn = uiElement.GetComponent<Button>();
        if (btn != null)
        {
            EventTrigger.Entry clickEntry = new EventTrigger.Entry();
            clickEntry.eventID = EventTriggerType.PointerClick;
            clickEntry.callback.AddListener((eventData) => btn.onClick.Invoke());
            trigger.triggers.Add(clickEntry);
        }

        TMP_InputField inputField = uiElement.GetComponent<TMP_InputField>();
        if (inputField != null)
        {
            EventTrigger.Entry clickEntry = new EventTrigger.Entry();
            clickEntry.eventID = EventTriggerType.PointerClick;
            clickEntry.callback.AddListener((eventData) =>
            {
                inputField.Select();
                inputField.ActivateInputField();
                ShowVirtualKeyboard(inputField);
            });
            trigger.triggers.Add(clickEntry);
        }
    }

    #endregion

    #region 虛擬鍵盤

    void ShowVirtualKeyboard(TMP_InputField targetField)
    {
        if (virtualKeyboardPrefab != null)
        {
            if (virtualKeyboard == null || !virtualKeyboard.gameObject.activeSelf)
            {
                GameObject keyboardObj = Instantiate(virtualKeyboardPrefab, ipConfigCanvasInstance.transform);
                virtualKeyboard = keyboardObj.GetComponent<VirtualKeyboard>();
                if (virtualKeyboard == null)
                {
                    virtualKeyboard = keyboardObj.AddComponent<VirtualKeyboard>();
                }
                keyboardObj.transform.localPosition = new Vector3(0, -300, 0);
            }

            if (virtualKeyboard != null)
            {
                virtualKeyboard.Show(targetField);
            }
        }
        else
        {
            CreateSimpleVirtualKeyboard(targetField);
        }
    }

    void CreateSimpleVirtualKeyboard(TMP_InputField targetField)
    {
        if (ipConfigCanvasInstance == null) return;

        GameObject keyboardPanel = CreateUIElement("VirtualKeyboard", ipConfigCanvasInstance.transform);
        Image panelImage = keyboardPanel.AddComponent<Image>();
        panelImage.color = new Color(0.15f, 0.15f, 0.15f, 0.95f);
        SetRectTransform(keyboardPanel, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f),
            new Vector2(600, 400), new Vector2(0, -300));

        // 標題
        GameObject titleObj = CreateUIElement("Title", keyboardPanel.transform);
        Text titleText = titleObj.AddComponent<Text>();
        titleText.text = "Virtual Keyboard";
        titleText.fontSize = 28;
        titleText.color = Color.white;
        titleText.alignment = TextAnchor.MiddleCenter;
        titleText.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        SetRectTransform(titleObj, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f),
            new Vector2(500, 40), new Vector2(0, 160));

        // 數字按鈕
        float buttonSize = 80f;
        float spacing = 10f;
        float startX = -120f;
        float startY = 80f;

        for (int i = 1; i <= 9; i++)
        {
            int row = (i - 1) / 3;
            int col = (i - 1) % 3;
            CreateKeyboardButton(keyboardPanel.transform, $"Key{i}", i.ToString(),
                new Vector2(startX + col * (buttonSize + spacing), startY - row * (buttonSize + spacing)),
                new Vector2(buttonSize, buttonSize));
        }

        CreateKeyboardButton(keyboardPanel.transform, "Key0", "0",
            new Vector2(startX, startY - 3 * (buttonSize + spacing)),
            new Vector2(buttonSize, buttonSize));
        CreateKeyboardButton(keyboardPanel.transform, "KeyDot", ".",
            new Vector2(startX + (buttonSize + spacing), startY - 3 * (buttonSize + spacing)),
            new Vector2(buttonSize, buttonSize));
        CreateKeyboardButton(keyboardPanel.transform, "Backspace", "Del",
            new Vector2(startX + 2 * (buttonSize + spacing), startY - 3 * (buttonSize + spacing)),
            new Vector2(buttonSize, buttonSize));

        VirtualKeyboard keyboard = keyboardPanel.AddComponent<VirtualKeyboard>();
        keyboard.SetTargetInputField(targetField);
        virtualKeyboard = keyboard;

        // 綁定按鈕
        Button[] buttons = keyboardPanel.GetComponentsInChildren<Button>();
        foreach (var btn in buttons)
        {
            btn.onClick.RemoveAllListeners();
            string btnName = btn.name;

            if (btnName.StartsWith("Key") && btnName != "KeyDot")
            {
                string numStr = btnName.Replace("Key", "");
                if (int.TryParse(numStr, out int num))
                {
                    btn.onClick.AddListener(() => keyboard.AddCharacter(num.ToString()));
                }
            }
            else if (btnName == "KeyDot")
            {
                btn.onClick.AddListener(() => keyboard.AddCharacter("."));
            }
            else if (btnName == "Backspace")
            {
                btn.onClick.AddListener(() => keyboard.Backspace());
            }

            AddVRInteractionSupport(btn.gameObject);
        }
    }

    Button CreateKeyboardButton(Transform parent, string name, string text,
        Vector2 position, Vector2 size)
    {
        GameObject buttonObj = CreateUIElement(name, parent);

        Image buttonImage = buttonObj.AddComponent<Image>();
        buttonImage.color = new Color(0.3f, 0.3f, 0.3f, 1f);

        Button button = buttonObj.AddComponent<Button>();

        GameObject textObj = CreateUIElement("Text", buttonObj.transform);
        Text textComp = textObj.AddComponent<Text>();
        textComp.text = text;
        textComp.fontSize = 32;
        textComp.color = Color.white;
        textComp.alignment = TextAnchor.MiddleCenter;

        Font defaultFont = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        if (defaultFont == null) defaultFont = Resources.GetBuiltinResource<Font>("Arial.ttf");
        if (defaultFont != null) textComp.font = defaultFont;

        SetRectTransform(textObj, Vector2.zero, Vector2.one, Vector2.zero, Vector2.zero);
        SetRectTransform(buttonObj, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), size, position);

        return button;
    }

    #endregion

    #region 配置操作

    void UpdateIPConfigUI()
    {
        if (ipAddressInputField != null)
        {
            ipAddressInputField.text = tempIPAddress;
        }

        if (portInputField != null)
        {
            portInputField.text = tempPort.ToString();
        }
    }

    void OnApplyIPConfig()
    {
        if (ipAddressInputField != null)
        {
            tempIPAddress = ipAddressInputField.text;
        }

        if (portInputField != null)
        {
            if (int.TryParse(portInputField.text, out int port))
            {
                tempPort = port;
            }
        }

        if (IsValidIPAddress(tempIPAddress))
        {
            displayIPAddress = tempIPAddress;
            displayPort = tempPort;

            Debug.Log($"✅ IP 配置已更新: {displayIPAddress}:{displayPort}");
            Debug.LogWarning("⚠️ 注意：實際連接 IP 需要在 Project Settings 中修改！");

            OnToggleIPConfigUI();
        }
        else
        {
            Debug.LogError($"❌ 無效的 IP 地址格式: {tempIPAddress}");
        }
    }

    void OnCancelIPConfig()
    {
        tempIPAddress = displayIPAddress;
        tempPort = displayPort;
        UpdateIPConfigUI();
        OnToggleIPConfigUI();
    }

    void OnToggleIPConfigUI()
    {
        if (ipConfigCanvasInstance != null)
        {
            isIPConfigUIVisible = !isIPConfigUIVisible;
            ipConfigCanvasInstance.SetActive(isIPConfigUIVisible);

            if (toggleButton != null)
            {
                TextMeshProUGUI toggleText = toggleButton.GetComponentInChildren<TextMeshProUGUI>();
                if (toggleText != null)
                {
                    toggleText.text = isIPConfigUIVisible ? "Hide Config" : "Show Config";
                }
            }
        }
    }

    bool IsValidIPAddress(string ip)
    {
        if (string.IsNullOrEmpty(ip)) return false;

        string[] parts = ip.Split('.');
        if (parts.Length != 4) return false;

        foreach (string part in parts)
        {
            if (!int.TryParse(part, out int num) || num < 0 || num > 255)
                return false;
        }

        return true;
    }

    #endregion
}
