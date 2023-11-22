using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
// using Engine.I18N;
// using Engine.Lib;
using System;

//using Engine.I18N;
//using TMPro;






public class KTool
{

    static bool? isWindowsPlatform = null;
    public static bool IsWindowsPlatform()
    {
        if(isWindowsPlatform == null)
        {
            var expectedMobile = SceneFirst.Instance.expectedMobile;

            if( expectedMobile )
            {
                isWindowsPlatform = false;
            }
            else 
            {
                var p = Application.platform;
                isWindowsPlatform =     
                            (p == RuntimePlatform.WindowsEditor) 
                        || (p == RuntimePlatform.WindowsPlayer) 
                        || (p == RuntimePlatform.OSXEditor)
                        || (p == RuntimePlatform.OSXPlayer)
                        || (p == RuntimePlatform.LinuxEditor)
                        || (p == RuntimePlatform.LinuxPlayer);
            }
        }
        return isWindowsPlatform.Value;
    }


    public static T GetComponent<T>(GameObject instance) where T : Component
    {
        return instance != null ? instance.GetComponent<T>() : null;
    }

    /// <summary>
    /// 获取或者添加lua调用
    /// </summary>
    /// <param name="instance"></param>
    /// <param name="t"></param>
    /// <returns></returns>
    public static Component GetComponent(GameObject instance, Type t)
    {
        var result = instance.GetComponent(t);
        return result;
    }


    public static T AddComponent<T>(GameObject instance) where T : Component
    {
        return instance != null ? instance.AddComponent<T>() : null;
    }


    public static T GetOrAddComponent<T>(GameObject instance) where T : Component
    {
        var result = GetComponent<T>(instance);

        if (result == null)
        {
            result = AddComponent<T>(instance);
        }

        return result;
    }

    /// <summary>
    /// 获取或者添加lua调用
    /// </summary>
    /// <param name="instance"></param>
    /// <param name="t"></param>
    /// <returns></returns>
    public static Component GetOrAddComponent(GameObject instance, Type t)
    {
        var result = instance.GetComponent(t);

        if (result == null)
        {
            result = instance.AddComponent(t);
        }

        return result;
    }



    public static void SafeDelete(UnityEngine.Object instance, bool immediate = false)
    {
        if (instance != null)
        {
            var gameObject = instance as GameObject;

            if (gameObject != null)
            {
                gameObject.SetActive(false);

                if (!(gameObject.transform is RectTransform))
                {
                    gameObject.transform.parent = null;
                }
            }

            if (!Application.isPlaying || immediate)
            {
                UnityEngine.Object.DestroyImmediate(instance);
            }
            else
            {
                UnityEngine.Object.Destroy(instance);
            }
        }
    }


    public static bool IsNull(object obj)
    {
        var unityObj = obj as UnityEngine.Object;

        if (!ReferenceEquals(unityObj, null))
        {
            return unityObj == null;
        }

        return obj == null;
    }


    // todo: c# 原生的 operator 好像不支持泛型... 除非加库
    // https://jonskeet.uk/csharp/miscutil/usage/genericoperators.html 
    //
    // 参数 x 在区间[t1,t2] 中, 计算得到 区间[s1,s2] 中同比例的点的值;
    public static float Remap(float t1, float t2, float s1, float s2, float x)
    {
        return ((x - t1) / (t2 - t1) * (s2 - s1) + s1);
    }
    public static Vector2 Remap(float t1, float t2, Vector2 s1, Vector2 s2, float x)
    {
        return ((x - t1) / (t2 - t1) * (s2 - s1) + s1);
    }
    public static Vector3 Remap(float t1, float t2, Vector3 s1, Vector3 s2, float x)
    {
        return ((x - t1) / (t2 - t1) * (s2 - s1) + s1);
    }
    public static Vector4 Remap(float t1, float t2, Vector4 s1, Vector4 s2, float x)
    {
        return ((x - t1) / (t2 - t1) * (s2 - s1) + s1);
    }



    // oldSelfPos_ is the follower's old position, oldTgtPos_ is the target's old position, newTgtPos_ is the target's new position, t is the elapsed time, and k is the lerp rate
    public static Vector3 SuperSmoothLerp(Vector3 oldSelfPos_, Vector3 oldTgtPos_, Vector3 newTgtPos_, float deltaTime_, float k)
    {
        float kt = k * deltaTime_;
        Vector3 f = oldSelfPos_ - oldTgtPos_ + (newTgtPos_ - oldTgtPos_) / kt;
        return newTgtPos_ - (newTgtPos_ - oldTgtPos_) / kt + f * Mathf.Exp( - kt);
    }


}



