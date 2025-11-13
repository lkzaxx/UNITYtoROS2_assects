using UnityEngine;
using UnityEngine.UI;
using TMPro;

/// <summary>
/// VR 虛擬鍵盤組件 - 用於 IP 地址和端口輸入
/// </summary>
public class VirtualKeyboard : MonoBehaviour
{
    [Header("鍵盤設定")]
    [Tooltip("目標輸入框")]
    public TMP_InputField targetInputField;
    
    [Header("按鈕引用（自動查找）")]
    public Button[] numberButtons; // 0-9
    public Button dotButton;         // .
    public Button backspaceButton;   // 刪除
    public Button clearButton;       // 清空
    public Button confirmButton;     // 確認
    public Button cancelButton;      // 取消
    
    private string currentText = "";
    
    void Start()
    {
        InitializeButtons();
    }
    
    /// <summary>
    /// 初始化按鈕事件
    /// </summary>
    void InitializeButtons()
    {
        // 自動查找按鈕（如果未手動指定）
        if (numberButtons == null || numberButtons.Length == 0)
        {
            FindButtons();
        }
        
        // 綁定數字按鈕
        if (numberButtons != null)
        {
            for (int i = 0; i < numberButtons.Length && i < 10; i++)
            {
                int num = i;
                if (numberButtons[i] != null)
                {
                    numberButtons[i].onClick.AddListener(() => AddCharacter(num.ToString()));
                }
            }
        }
        
        // 綁定點按鈕
        if (dotButton != null)
        {
            dotButton.onClick.AddListener(() => AddCharacter("."));
        }
        
        // 綁定功能按鈕
        if (backspaceButton != null)
        {
            backspaceButton.onClick.AddListener(Backspace);
        }
        
        if (clearButton != null)
        {
            clearButton.onClick.AddListener(Clear);
        }
        
        if (confirmButton != null)
        {
            confirmButton.onClick.AddListener(Confirm);
        }
        
        if (cancelButton != null)
        {
            cancelButton.onClick.AddListener(Cancel);
        }
    }
    
    /// <summary>
    /// 自動查找按鈕（根據命名規則）
    /// </summary>
    void FindButtons()
    {
        Button[] allButtons = GetComponentsInChildren<Button>();
        System.Collections.Generic.List<Button> numberList = new System.Collections.Generic.List<Button>();
        
        foreach (Button btn in allButtons)
        {
            string btnName = btn.name.ToLower();
            
            // 查找數字按鈕
            for (int i = 0; i < 10; i++)
            {
                if (btnName.Contains($"key{i}") || btnName.Contains($"button{i}") || 
                    btnName.Contains($"num{i}") || btnName == i.ToString())
                {
                    if (numberList.Count <= i)
                    {
                        numberList.AddRange(new Button[i + 1 - numberList.Count]);
                    }
                    numberList[i] = btn;
                    break;
                }
            }
            
            // 查找功能按鈕
            if (btnName.Contains("dot") || btnName.Contains("point") || btnName == ".")
            {
                dotButton = btn;
            }
            else if (btnName.Contains("backspace") || btnName.Contains("delete") || btnName.Contains("del"))
            {
                backspaceButton = btn;
            }
            else if (btnName.Contains("clear") || btnName.Contains("cls"))
            {
                clearButton = btn;
            }
            else if (btnName.Contains("confirm") || btnName.Contains("ok") || btnName.Contains("enter"))
            {
                confirmButton = btn;
            }
            else if (btnName.Contains("cancel") || btnName.Contains("close"))
            {
                cancelButton = btn;
            }
        }
        
        numberButtons = numberList.ToArray();
    }
    
    /// <summary>
    /// 設置目標輸入框
    /// </summary>
    public void SetTargetInputField(TMP_InputField field)
    {
        targetInputField = field;
        if (field != null)
        {
            currentText = field.text;
        }
    }
    
    /// <summary>
    /// 添加字符
    /// </summary>
    public void AddCharacter(string character)
    {
        if (targetInputField != null)
        {
            currentText += character;
            targetInputField.text = currentText;
            targetInputField.caretPosition = currentText.Length;
        }
    }
    
    /// <summary>
    /// 刪除最後一個字符
    /// </summary>
    public void Backspace()
    {
        if (targetInputField != null && currentText.Length > 0)
        {
            currentText = currentText.Substring(0, currentText.Length - 1);
            targetInputField.text = currentText;
            targetInputField.caretPosition = currentText.Length;
        }
    }
    
    /// <summary>
    /// 清空輸入
    /// </summary>
    public void Clear()
    {
        if (targetInputField != null)
        {
            currentText = "";
            targetInputField.text = "";
            targetInputField.caretPosition = 0;
        }
    }
    
    /// <summary>
    /// 確認輸入
    /// </summary>
    public void Confirm()
    {
        if (targetInputField != null)
        {
            targetInputField.text = currentText;
            targetInputField.onEndEdit?.Invoke(currentText);
        }
        
        // 隱藏鍵盤
        gameObject.SetActive(false);
    }
    
    /// <summary>
    /// 取消輸入
    /// </summary>
    public void Cancel()
    {
        // 恢復原始值
        if (targetInputField != null)
        {
            currentText = targetInputField.text;
        }
        
        // 隱藏鍵盤
        gameObject.SetActive(false);
    }
    
    /// <summary>
    /// 顯示鍵盤
    /// </summary>
    public void Show(TMP_InputField field)
    {
        SetTargetInputField(field);
        if (field != null)
        {
            currentText = field.text;
        }
        gameObject.SetActive(true);
    }
}

