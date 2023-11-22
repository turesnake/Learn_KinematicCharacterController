using System.Collections;
using System.Collections.Generic;
using UnityEngine;


/// <summary>
///  确保成为 全场景最先被执行的代码
/// </summary>


[DefaultExecutionOrder(-1000000)]
public class SceneFirst : MonoSingleton<SceneFirst>
{
    //--------- 一些全局配置参数 ----------
    public bool expectedMobile = true; // 希望在 win平台 强制使用 移动模式 控制;





    protected override void Awake()
    {
        base.Awake();
        //---
        TextDebug.SetActive(true);   
        Screen.orientation = ScreenOrientation.LandscapeLeft; // 禁止屏幕自动旋转, 锁死为宽屏



        
    }
    




}
