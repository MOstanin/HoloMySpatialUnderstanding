using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using HoloToolkit.Unity.SpatialMapping;
using HoloToolkit.Unity;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;


public class MyScript : MonoBehaviour
{
    public GameObject realRobot;
    int f;
    public bool flag;
    GameObject robot;

    public void StartFit()
    {
        flag = true;
        
    }
    
    public void ChangeModel()
    {
        Instantiate(realRobot, gameObject.transform.position, gameObject.transform.rotation);
        Destroy(robot);
    }

    private void Start()
    {
        f = 0;
        flag = false;
        //robot = GameObject.Find("IIWA_model");
        robot = GameObject.Find("Agilus_model");
        
    }

    private void Update()
    {
        if (flag)
        {
            Vector3 oldPos = robot.transform.position;
            Vector3 oldOri = robot.transform.eulerAngles;

            MyClick();
            if (f>11 && (robot.transform.eulerAngles - oldOri).sqrMagnitude < 0.1 && (robot.transform.position - oldPos).sqrMagnitude < 0.01)
            {
                flag = false;
                
            }
        }
    }


    public void MyClick()
    {

        GameObject[] points = GameObject.FindGameObjectsWithTag("point");

        foreach (GameObject p in points)
        {
            Destroy(p);
        }
        
        

        MeshFilter[] meshFilterRobot = robot.GetComponentsInChildren<MeshFilter>();

        //List<MeshFilter> meshFiltersScene = SpatialMappingManager.Instance.GetMeshFilters();
        List<MeshFilter> meshFiltersScene = SpatialUnderstanding.Instance.UnderstandingCustomMesh.GetMeshFilters();
        // myScene = GameObject.Find("Selection");

        //MeshFilter[] meshFiltersScene = myScene.GetComponentsInChildren<MeshFilter>();

        int n = 0;
        foreach (MeshFilter meshFilter in meshFiltersScene)
        {
            n = n + meshFilter.sharedMesh.vertices.Length;
        }
        int j = 0;
        List<Vector3> pointsList = new List<Vector3>();
        //Vector3[] pointsScene = new Vector3[n];
        foreach (MeshFilter meshFilter in meshFiltersScene)
        {
            foreach (Vector3 point in meshFilter.sharedMesh.vertices)
            {
                Vector3 p = point + meshFilter.gameObject.transform.position;
                //j = j + 1;
                if ((p - meshFilterRobot[0].gameObject.transform.position).sqrMagnitude < 2)
                {
                    pointsList.Add(p);
                }
            }
        }
        Vector3[] pointsScene = pointsList.ToArray();
        pointsList.Clear();

        n = meshFilterRobot[0].sharedMesh.vertices.Length;
        int n_R = 1000;
        int k = n / n_R;
        Vector3[] pointsRobot = new Vector3[n_R];
        Vector3[] nearestPoints = new Vector3[pointsRobot.Length];

        int c = 0;
        while (c < 1)
        {
            Vector3 robotPos = meshFilterRobot[0].gameObject.transform.position;
            Quaternion robotOri = meshFilterRobot[0].gameObject.transform.rotation;
            for (int i = 0; i < n_R; i++)
            {
                pointsRobot[i] = robotOri * meshFilterRobot[0].sharedMesh.vertices[i * k] + robotPos; 
            }

            //Vector3[] pointsRobot = meshFilterRobot[0].sharedMesh.vertices;

            Vector3 midRobot = Vector3.zero;
            Vector3 highRobot = new Vector3(0, -100, 0);
            for (int i = 0; i < pointsRobot.Length; i++)
            {
                midRobot += pointsRobot[i];
                if (pointsRobot[i].y > highRobot.y)
                {
                    highRobot = pointsRobot[i];
                }
            }
            midRobot = midRobot / pointsRobot.Length;



            Vector3 midNearest = Vector3.zero;
            Vector3 higtNearest = new Vector3(0, -100, 0);

            for (int i = 0; i < pointsRobot.Length; i++)
            {

                Vector3 pointR = pointsRobot[i];
                Vector3 nearestPoint = Vector3.zero;
                float minDistanceSqr = Mathf.Infinity;
                foreach (Vector3 pointS in pointsScene)
                {
                    Vector3 diff = pointR - pointS;
                    float distSqr = diff.sqrMagnitude;
                    if (distSqr < minDistanceSqr)
                    {
                        minDistanceSqr = distSqr;
                        nearestPoint = pointS;
                    }
                }

                nearestPoints[i] = nearestPoint;
                midNearest += nearestPoint;
                if (nearestPoint.y > higtNearest.y)
                {
                    higtNearest = nearestPoint;
                }
            }
            midNearest = midNearest / pointsRobot.Length;

            Matrix<float> H = Matrix<float>.Build.Dense(3, 3, 0);

            for (int i = 0; i < pointsRobot.Length; i++)
            {
                float[,] del_1 = new float[3, 1] {
                    { pointsRobot[i].x - midRobot.x },
                    { pointsRobot[i].z - midRobot.z },
                    { pointsRobot[i].y - midRobot.y },
                };

                float[,] del_2 = new float[1, 3] {{
                    nearestPoints[i].x - midNearest.x,
                    nearestPoints[i].z - midNearest.z,
                    nearestPoints[i].y - midNearest.y,
                    }};

                Matrix<float> v1 = Matrix<float>.Build.DenseOfArray(del_1);
                Matrix<float> v2 = Matrix<float>.Build.DenseOfArray(del_2);

                H = H + v1 * v2;
            }

            Svd<float> svd = H.Svd();

            Matrix<float> R = svd.VT.Transpose() * svd.U.Transpose();

            float angle = Mathf.Atan2(R[1, 0], R[0, 0]) * 180 / Mathf.PI;

            robot.transform.position = robot.transform.position + (midNearest - midRobot);
            robot.transform.Rotate(new Vector3(0, -angle, 0));
            c += 1;
            if (f > 10 && (higtNearest - highRobot).sqrMagnitude > 0.04)
            {
                robot.transform.eulerAngles = robot.transform.eulerAngles + (new Vector3(0, 180, 0));
            }
            f++;
        }

        
        //DrawPoints(pointsScene);
        //DrawPoints(pointsRobot);
    }


    private void DrawPoints( Vector3[] vectors)
    {
        //Vector3[] vectors = meshFilter.sharedMesh.vertices;
        int n = vectors.Length;
        //int k = n / 10000;
        for (int i = 0; i < n; i++)
        {
            GameObject point = GameObject.CreatePrimitive(PrimitiveType.Cube);
            point.tag = "point";
            point.transform.localScale = new Vector3(0.01f, 0.01f, 0.01f);
            Vector3 vector = vectors[i];
            point.transform.position = vector;

        }
    }
}
