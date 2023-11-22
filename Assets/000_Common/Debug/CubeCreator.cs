using System.Collections;
using System.Collections.Generic;
using UnityEngine;



public class CubeCreator : MonoBehaviour
{

    void Start()
    {
        Create();
    }


    public static Transform Create()
    {
        GameObject newgo = GameObject.CreatePrimitive(PrimitiveType.Cube);
        newgo.name = "Cube";


        // GameObject newgo = new GameObject("Cube");
        // MeshFilter meshFilter = newgo.AddComponent<MeshFilter>();
        // meshFilter.mesh = CreateCubeMesh();
        // MeshRenderer meshRenderer = newgo.AddComponent<MeshRenderer>();
        // meshRenderer.material = new Material(Shader.Find("Universal Render Pipeline/Lit"));
        // newgo.transform.position = new Vector3(0, 0, 0);
        // return newgo.transform;

        return newgo.transform;
    }

    // Create a cube mesh
    static Mesh CreateCubeMesh()
    {
        Mesh mesh = new Mesh();

        Vector3[] vertices = new Vector3[]
        {
            new Vector3(-0.5f, -0.5f, -0.5f),
            new Vector3(0.5f, -0.5f, -0.5f),
            new Vector3(0.5f, 0.5f, -0.5f),
            new Vector3(-0.5f, 0.5f, -0.5f),
            new Vector3(-0.5f, -0.5f, 0.5f),
            new Vector3(0.5f, -0.5f, 0.5f),
            new Vector3(0.5f, 0.5f, 0.5f),
            new Vector3(-0.5f, 0.5f, 0.5f)
        };

        int[] triangles = new int[]
        {
            0, 2, 1, // front
            0, 3, 2,
            1, 6, 5, // right
            1, 2, 6,
            5, 7, 4, // back
            5, 6, 7,
            4, 3, 0, // left
            4, 7, 3,
            3, 6, 2, // top
            3, 7, 6,
            4, 1, 5, // bottom
            4, 0, 1
        };

        mesh.vertices = vertices;
        mesh.triangles = triangles;

        return mesh;
    }
}
