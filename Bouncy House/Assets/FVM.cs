using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.Threading.Tasks;

public class FVM : MonoBehaviour
{
    float dt = 0.003f;
    float mass = 1;
    float stiffness_0 = 20000.0f;
    float stiffness_1 = 5000.0f;
    float damp = 0.999f;
    Vector3 gravity = new Vector3(0, -9.8f, 0);

    int[] Tet;
    int tet_number;         //The number of tetrahedra

    Vector3[] Force;
    Vector3[] V;
    Vector3[] X;
    int number;             //The number of vertices

    Matrix4x4[] inv_Dm;

    //For Laplacian smoothing.
    Vector3[] V_sum;
    int[] V_num;

    SVD svd = new SVD();

    // Start is called before the first frame update
    void Start()
    {
        // FILO IO: Read the house model from files.
        // The model is from Jonathan Schewchuk's Stellar lib.
        {
            string fileContent = File.ReadAllText("Assets/house2.ele");
            string[] Strings = fileContent.Split(new char[] { ' ', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);

            tet_number = int.Parse(Strings[0]);
            Tet = new int[tet_number * 4];

            for (int tet = 0; tet < tet_number; tet++)
            {
                Tet[tet * 4 + 0] = int.Parse(Strings[tet * 5 + 4]) - 1;
                Tet[tet * 4 + 1] = int.Parse(Strings[tet * 5 + 5]) - 1;
                Tet[tet * 4 + 2] = int.Parse(Strings[tet * 5 + 6]) - 1;
                Tet[tet * 4 + 3] = int.Parse(Strings[tet * 5 + 7]) - 1;
            }
        }
        {
            string fileContent = File.ReadAllText("Assets/house2.node");
            string[] Strings = fileContent.Split(new char[] { ' ', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
            number = int.Parse(Strings[0]);
            X = new Vector3[number];
            for (int i = 0; i < number; i++)
            {
                X[i].x = float.Parse(Strings[i * 5 + 5]) * 0.4f;
                X[i].y = float.Parse(Strings[i * 5 + 6]) * 0.4f;
                X[i].z = float.Parse(Strings[i * 5 + 7]) * 0.4f;
            }
            //Centralize the model.
            Vector3 center = Vector3.zero;
            for (int i = 0; i < number; i++) center += X[i];
            center = center / number;
            for (int i = 0; i < number; i++)
            {
                X[i] -= center;
                float temp = X[i].y;
                X[i].y = X[i].z;
                X[i].z = temp;
            }
        }
        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/


        //Create triangle mesh.
        Vector3[] vertices = new Vector3[tet_number * 12];
        int vertex_number = 0;
        for (int tet = 0; tet < tet_number; tet++)
        {
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];

            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];

            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];

            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
        }

        int[] triangles = new int[tet_number * 12];
        for (int t = 0; t < tet_number * 4; t++)
        {
            triangles[t * 3 + 0] = t * 3 + 0;
            triangles[t * 3 + 1] = t * 3 + 1;
            triangles[t * 3 + 2] = t * 3 + 2;
        }
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();


        V = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

        //TODO: Need to allocate and assign inv_Dm
        inv_Dm = new Matrix4x4[tet_number];
        for (int i = 0; i < tet_number; i++)
        {
            inv_Dm[i] = Build_Edge_Matrix(i).inverse;
        }
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
        Matrix4x4 ret = Matrix4x4.zero;
        //TODO: Need to build edge matrix here.
        ret[0, 0] = X[Tet[tet * 4 + 1]].x - X[Tet[tet * 4 + 0]].x;
        ret[1, 0] = X[Tet[tet * 4 + 1]].y - X[Tet[tet * 4 + 0]].y;
        ret[2, 0] = X[Tet[tet * 4 + 1]].z - X[Tet[tet * 4 + 0]].z;
        ret[3, 0] = 0;

        ret[0, 1] = X[Tet[tet * 4 + 2]].x - X[Tet[tet * 4 + 0]].x;
        ret[1, 1] = X[Tet[tet * 4 + 2]].y - X[Tet[tet * 4 + 0]].y;
        ret[2, 1] = X[Tet[tet * 4 + 2]].z - X[Tet[tet * 4 + 0]].z;
        ret[3, 1] = 0;

        ret[0, 2] = X[Tet[tet * 4 + 3]].x - X[Tet[tet * 4 + 0]].x;
        ret[1, 2] = X[Tet[tet * 4 + 3]].y - X[Tet[tet * 4 + 0]].y;
        ret[2, 2] = X[Tet[tet * 4 + 3]].z - X[Tet[tet * 4 + 0]].z;
        ret[3, 2] = 0;

        ret[0, 3] = 0;
        ret[1, 3] = 0;
        ret[2, 3] = 0;
        ret[3, 3] = 1;

        return ret;
    }

    Matrix4x4 AddMatrix4X4(Matrix4x4 a, Matrix4x4 b)
    {
        Matrix4x4 res = a;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                res[i, j] += b[i, j];
            }
        }

        return res;
    }

    Matrix4x4 SubMatrix4X4(Matrix4x4 a, Matrix4x4 b)
    {
        Matrix4x4 res = a;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                res[i, j] -= b[i, j];
            }
        }

        return res;
    }

    Matrix4x4 MulMatrix4X4(float a, Matrix4x4 b)
    {
        Matrix4x4 res = b;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                res[i, j] *= a;
            }
        }

        return res;
    }

    void Smooth_V()
    {
        for (int i = 0; i < number; i++)
        {
            V_sum[i] = new Vector3(0, 0, 0);
            V_num[i] = 0;
        }

        for (int tet = 0; tet < tet_number; tet++)
        {
            Vector3 sum = V[Tet[tet * 4 + 0]] + V[Tet[tet * 4 + 1]] + V[Tet[tet * 4 + 2]] + V[Tet[tet * 4 + 3]];
            V_sum[Tet[tet * 4 + 0]] += sum;
            V_sum[Tet[tet * 4 + 1]] += sum;
            V_sum[Tet[tet * 4 + 2]] += sum;
            V_sum[Tet[tet * 4 + 3]] += sum;
            V_num[Tet[tet * 4 + 0]] += 4;
            V_num[Tet[tet * 4 + 1]] += 4;
            V_num[Tet[tet * 4 + 2]] += 4;
            V_num[Tet[tet * 4 + 3]] += 4;
        }

        for (int i = 0; i < number; i++)
        {
            V[i] = 0.9f * V[i] + 0.1f * V_sum[i] / V_num[i];
        }
    }
    void _Update()
    {
        // Jump up.
        if (Input.GetKeyDown(KeyCode.Space))
        {
            for (int i = 0; i < number; i++)
                V[i].y += 0.2f;
        }

        for (int i = 0; i < number; i++)
        {
            //TODO: Add gravity to Force.
            Force[i] = Vector3.zero;
            Force[i] += mass * gravity;
        }

        Parallel.For(0, tet_number, tet =>
        {
            //TODO: Deformation Gradient
            Matrix4x4 F = Build_Edge_Matrix(tet) * inv_Dm[tet];

            //TODO: Green Strain
            Matrix4x4 G = SubMatrix4X4((F.transpose * F), Matrix4x4.identity);
            G = MulMatrix4X4(0.5f, G);

            //TODO: Second PK Stress
            float trace = G[0, 0] + G[1, 1] + G[2, 2];
            Matrix4x4 S = AddMatrix4X4(MulMatrix4X4(2 * stiffness_1, G), MulMatrix4X4(trace * stiffness_0, Matrix4x4.identity)); //反？

            //TODO: Elastic Force
            float a = 1 / inv_Dm[tet].determinant / 6;
            Matrix4x4 f = MulMatrix4X4(-a, F * S * inv_Dm[tet].transpose);

            Force[Tet[tet * 4 + 1]].x += f[0, 0];
            Force[Tet[tet * 4 + 1]].y += f[1, 0];
            Force[Tet[tet * 4 + 1]].z += f[2, 0];

            Force[Tet[tet * 4 + 2]].x += f[0, 1];
            Force[Tet[tet * 4 + 2]].y += f[1, 1];
            Force[Tet[tet * 4 + 2]].z += f[2, 1];

            Force[Tet[tet * 4 + 3]].x += f[0, 2];
            Force[Tet[tet * 4 + 3]].y += f[1, 2];
            Force[Tet[tet * 4 + 3]].z += f[2, 2];

            Force[Tet[tet * 4 + 0]].x -= f[0, 0] + f[0, 1] + f[0, 2];
            Force[Tet[tet * 4 + 0]].y -= f[1, 0] + f[1, 1] + f[1, 2];
            Force[Tet[tet * 4 + 0]].z -= f[2, 0] + f[2, 1] + f[2, 2];
        });

        for (int tet = 0; tet < tet_number; tet++)
        {

        }

        for (int i = 0; i < number; i++)
        {
            //TODO: Update X and V here.
            V[i] += dt * Force[i] / mass;
            V[i] *= damp;
            X[i] += dt * V[i];

            //TODO: (Particle) collision with floor.
            if (X[i].y < -3f)
            {
                X[i].y = -3f;
                if (V[i].y < 0)
                {
                    V[i].y = -V[i].y;
                    V[i] *= 0.8f;
                }
            }
        }

        Smooth_V();
    }

    // Update is called once per frame
    void Update()
    {
        for (int l = 0; l < 10; l++)
            _Update();

        // Dump the vertex array for rendering.
        Vector3[] vertices = new Vector3[tet_number * 12];
        int vertex_number = 0;
        for (int tet = 0; tet < tet_number; tet++)
        {
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
        }
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        mesh.vertices = vertices;
        mesh.RecalculateNormals();
    }
}
