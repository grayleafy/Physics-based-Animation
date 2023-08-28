using UnityEngine;
using System.Collections;
using UnityEngine.Rendering;

public class PBD_model : MonoBehaviour
{

    [SerializeField] float t = 0.0333f;
    [SerializeField] float damping = 0.99f;
    [SerializeField] Vector3 gravity = new Vector3(0, -9.8f, 0);
    [SerializeField] int iterations = 32;
    [SerializeField] float alpha = 0.2f;

    [Header("拉伸度限制")]
    [SerializeField] float minSigma = 0.8f;
    [SerializeField] float maxSigma = 1.2f;

    int[] E;
    float[] L;
    Vector3[] V;
    Vector3[] sum_x;
    float[] sum_n;

    // Use this for initialization
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;

        //Resize the mesh.
        int n = 21;
        Vector3[] X = new Vector3[n * n];
        sum_n = new float[n * n];
        sum_x = new Vector3[n * n];
        Vector2[] UV = new Vector2[n * n];
        int[] T = new int[(n - 1) * (n - 1) * 6];
        for (int j = 0; j < n; j++)
        {
            for (int i = 0; i < n; i++)
            {
                X[j * n + i] = new Vector3(5 - 10.0f * i / (n - 1), 0, 5 - 10.0f * j / (n - 1));
                UV[j * n + i] = new Vector3(i / (n - 1.0f), j / (n - 1.0f));
            }
        }
        int t = 0;
        for (int j = 0; j < n - 1; j++)
        {
            for (int i = 0; i < n - 1; i++)
            {
                T[t * 6 + 0] = j * n + i;
                T[t * 6 + 1] = j * n + i + 1;
                T[t * 6 + 2] = (j + 1) * n + i + 1;
                T[t * 6 + 3] = j * n + i;
                T[t * 6 + 4] = (j + 1) * n + i + 1;
                T[t * 6 + 5] = (j + 1) * n + i;
                t++;
            }
        }
        mesh.vertices = X;
        mesh.triangles = T;
        mesh.uv = UV;
        mesh.RecalculateNormals();

        //Construct the original edge list
        int[] _E = new int[T.Length * 2];
        for (int i = 0; i < T.Length; i += 3)
        {
            _E[i * 2 + 0] = T[i + 0];
            _E[i * 2 + 1] = T[i + 1];
            _E[i * 2 + 2] = T[i + 1];
            _E[i * 2 + 3] = T[i + 2];
            _E[i * 2 + 4] = T[i + 2];
            _E[i * 2 + 5] = T[i + 0];
        }
        //Reorder the original edge list
        for (int i = 0; i < _E.Length; i += 2)
            if (_E[i] > _E[i + 1])
                Swap(ref _E[i], ref _E[i + 1]);
        //Sort the original edge list using quicksort
        Quick_Sort(ref _E, 0, _E.Length / 2 - 1);

        int e_number = 0;
        for (int i = 0; i < _E.Length; i += 2)
            if (i == 0 || _E[i + 0] != _E[i - 2] || _E[i + 1] != _E[i - 1])
                e_number++;

        E = new int[e_number * 2];
        for (int i = 0, e = 0; i < _E.Length; i += 2)
            if (i == 0 || _E[i + 0] != _E[i - 2] || _E[i + 1] != _E[i - 1])
            {
                E[e * 2 + 0] = _E[i + 0];
                E[e * 2 + 1] = _E[i + 1];
                e++;
            }

        L = new float[E.Length / 2];
        for (int e = 0; e < E.Length / 2; e++)
        {
            int i = E[e * 2 + 0];
            int j = E[e * 2 + 1];
            L[e] = (X[i] - X[j]).magnitude;
        }

        V = new Vector3[X.Length];
        for (int i = 0; i < X.Length; i++)
            V[i] = new Vector3(0, 0, 0);
    }

    void Quick_Sort(ref int[] a, int l, int r)
    {
        int j;
        if (l < r)
        {
            j = Quick_Sort_Partition(ref a, l, r);
            Quick_Sort(ref a, l, j - 1);
            Quick_Sort(ref a, j + 1, r);
        }
    }

    int Quick_Sort_Partition(ref int[] a, int l, int r)
    {
        int pivot_0, pivot_1, i, j;
        pivot_0 = a[l * 2 + 0];
        pivot_1 = a[l * 2 + 1];
        i = l;
        j = r + 1;
        while (true)
        {
            do ++i; while (i <= r && (a[i * 2] < pivot_0 || a[i * 2] == pivot_0 && a[i * 2 + 1] <= pivot_1));
            do --j; while (a[j * 2] > pivot_0 || a[j * 2] == pivot_0 && a[j * 2 + 1] > pivot_1);
            if (i >= j) break;
            Swap(ref a[i * 2], ref a[j * 2]);
            Swap(ref a[i * 2 + 1], ref a[j * 2 + 1]);
        }
        Swap(ref a[l * 2 + 0], ref a[j * 2 + 0]);
        Swap(ref a[l * 2 + 1], ref a[j * 2 + 1]);
        return j;
    }

    void Swap(ref int a, ref int b)
    {
        int temp = a;
        a = b;
        b = temp;
    }

    void Strain_Limiting()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;

        //Apply PBD here.
        //...

        for (int i = 0; i < sum_n.Length; i++)
        {
            sum_n[i] = 0;
            sum_x[i] = Vector3.zero;
        }


        for (int i = 0; i < E.Length; i += 2)
        {
            Vector3 dis = vertices[E[i]] - vertices[E[i + 1]];
            float len = dis.magnitude;
            float sigma = len / L[i / 2];
            sigma = Mathf.Max(Mathf.Min(sigma, maxSigma), minSigma);
            float Le = L[i / 2] * sigma;
            Vector3 center = vertices[E[i]] + vertices[E[i + 1]];

            sum_x[E[i]] += 0.5f * (center + Le * dis.normalized);
            sum_n[E[i]]++;

            sum_x[E[i + 1]] += 0.5f * (center - Le * dis.normalized);
            sum_n[E[i + 1]]++;
        }

        for (int i = 0; i < vertices.Length; i++)
        {
            if (i == 0 || i == 20) continue;

            Vector3 X_new = (alpha * vertices[i] + sum_x[i]) / (alpha + sum_n[i]);
            V[i] = V[i] + (X_new - vertices[i]) / t;
            vertices[i] = X_new;
        }


        mesh.vertices = vertices;
    }

    void Collision_Handling()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] X = mesh.vertices;

        //For every vertex, detect collision and apply impulse if needed.
        //...

        GameObject sphere = GameObject.Find("ColliderSphere");
        Vector3 c = sphere.transform.position;
        float r = sphere.transform.localScale.x / 2.0f;
        r += 0.2f;


        for (int i = 0; i < X.Length; i++)
        {
            if (i == 0 || i == 20) continue;

            Vector3 dis = X[i] - c;
            if (dis.magnitude <= r)
            {
                Vector3 X_new = c + r * dis.normalized;
                V[i] += (X_new - X[i]) / t;
                X[i] = X_new;
            }
        }

        mesh.vertices = X;
    }

    // Update is called once per frame
    void Update()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] X = mesh.vertices;

        for (int i = 0; i < X.Length; i++)
        {
            if (i == 0 || i == 20) continue;
            //Initial Setup
            //...

            //重力
            V[i] = V[i] + t * gravity;
            //阻尼
            V[i] = damping * V[i];

            X[i] = X[i] + t * V[i];
        }
        mesh.vertices = X;

        for (int l = 0; l < iterations; l++)
        {
            Strain_Limiting();
        }
        Collision_Handling();

        mesh.RecalculateNormals();

    }


}

