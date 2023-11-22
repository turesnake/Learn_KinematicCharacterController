using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;



public abstract class MonoSingleton<T> : MonoBehaviour
                                                        where T : MonoSingleton<T>
{
    static T _instance;
    public static T Instance
    {
        get
        {
            if (_instance == null)
            {
                var objects = FindObjectsOfType(typeof(T));
                if (objects.Length > 0)
                {
                    if (objects.Length > 1)
                    {
                        Debug.LogError( "禁止重复的 singleton 实例: " + typeof(T).Name );
                    }
                    
                    _instance = (T) System.Convert.ChangeType(objects[0], typeof(T));
                    return _instance;
                }

                if (_instance == null && Application.isPlaying)
                {
                    Debug.LogError("需手动在场景中配置 MonoSingleton 实例"); // !!! 若有需要的话
                    var go = new GameObject("_" + typeof(T) + "_Instance_", typeof(T));
                    go.hideFlags = HideFlags.DontSave;
                    _instance = go.GetComponent<T>();
                    _instance.gameObject.SetActive(true);
                    _instance.enabled = true;
                }
            }
            return _instance;
        }
    }

    protected virtual void Awake()
    {
        string className = this.GetType().Name;
        Debug.Log( $"实例化: {className}" );
        if (_instance == null)
        {
            _instance = this as T;
        }
        else if (_instance != this)
        {
            Debug.LogError("Multiple " + typeof(T).Name + " in scene. please fix it.", this.gameObject);
            enabled = false;
            if (Application.isPlaying)
            {
                KTool.SafeDelete(this);
            }
            return;
        }

        // Singleton has DontDestroy flag.
        //DontDestroyOnLoad(gameObject);     // !!! 可选
    }


    public static void DestroyInstance()
    {
        if (!KTool.IsNull(_instance))
        {
            KTool.SafeDelete(_instance.gameObject);
        }
    }

    protected void OnDestroy()
    {
        if (_instance == this)
        {
            _instance = null;
        }
    }
}


