using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
// using Engine.I18N;
// using Engine.Lib;

//using Engine.I18N;
//using TMPro;



namespace GameDebug
{


public class DebugUtils
{



    public static Canvas FindOrCreateUICanvas()
    {
        /*
            uiCanvas_ForDebug 默认为全屏 canvas;
        */
        GameObject newGo = GameObject.Find("/uiCanvas_ForDebug"); // 非递归地在 root 层查找
        if( newGo == null )
        {
            newGo = new GameObject( "uiCanvas_ForDebug" );
        }
        newGo.layer = LayerMask.NameToLayer("UI");
        var canvasComp = KTool.GetOrAddComponent<Canvas>(newGo);
        var CanvasScalerComp = KTool.GetOrAddComponent<CanvasScaler>(newGo);
        canvasComp.renderMode = RenderMode.ScreenSpaceOverlay;
        canvasComp.sortingOrder = 500; // 尽可能排在前面
        UnityEngine.Object.DontDestroyOnLoad(newGo);  // todo: 有待商榷
        return canvasComp;
    }





}

}

