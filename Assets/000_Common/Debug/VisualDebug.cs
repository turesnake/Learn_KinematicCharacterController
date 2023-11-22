using System.Collections;
using System.Collections.Generic;
using UnityEngine;







public class VisualDebug : MonoSingleton<VisualDebug>
{

    //public Transform baseCubePrefab;

    Dictionary<string, Transform> entities = new Dictionary<string, Transform>();


    Vector3 farPos = new Vector3( 999f, 999f, 0f );


    void Start()
    {
        //Debug.Assert(baseCubePrefab);
    }


    // ------------------------------------


    public Transform CreateLine( string name_, Color color_ )
    {
        if( entities.ContainsKey(name_) )
        {
            Debug.LogError("已存在同名元素, 放弃生成");
            return null;
        }
        GameObject newgo = GameObject.CreatePrimitive(PrimitiveType.Cube);
        newgo.name = name_;
        var tf = newgo.transform;
        tf.SetParent(transform, false);
        tf.position = farPos;
        //--
        ChangeColor( tf, color_ );
        entities.Add( name_, tf );
        return tf;
    }


    // 设置一个 cube 杆子的 形状
    public void DrawLine( Transform tf_, Vector3 fromPos_, Vector3 toPos_, float radius_ ) 
    {
        Vector3 midPos = (fromPos_ + toPos_) * 0.5f;
        tf_.position = midPos;
        tf_.LookAt( toPos_, Vector3.up );
        tf_.localScale = new Vector3( radius_, radius_, (toPos_ - fromPos_).magnitude ); 
    }


    // ------------------------------------
    static void ChangeColor( Transform tf_, Color color_ ) 
    {
        var renderer = tf_.GetComponent<Renderer>();
        Debug.Assert( renderer && renderer.material );
        renderer.material.SetColor( "_BaseColor", color_ ); // 因为是 debug 工具. 无需考虑这个 material 的释放问题
    }





}
