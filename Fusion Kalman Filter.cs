using UnityEngine;
using System.IO.Ports;
using System;
using Matrix = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using OpenCVForUnity;
using System.Collections;

public class RightForeArmController : MonoBehaviour
{
    public KalmanFilter kalman = new KalmanFilter(6, 6, 6, CvType.CV_64FC4);
    private SerialPort port = new SerialPort("COM3", 9600);
    private String CurrentReading = "ABCD";
    private double[] q0 = { 0, 0, 0, 0 };
    private double[] q1 = { 0, 0, 0, 0 };
    private double[] q2 = { 0, 0, 0, 0 };
    private double[] xvalues = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    double dx, dy, dz, t1=0, t2=0, xsum;
    float ax, ay, az;
    private String[] SubStrings;
    string path = "Assets/test.txt";
    

    Matrix A = Matrix.Build.DenseOfArray(new double[,] {
             {0,0,0,0},
             {0,0,0,0},
             {0,0,0,0},
             {0,0,0,0}});
    Matrix B = Matrix.Build.DenseOfArray(new double[,] {
             {0,0,0,0},
             {0,0,0,0},
             {0,0,0,0},
             {0,0,0,0}});
    Matrix Y = Matrix.Build.DenseOfArray(new double[,] {
             {0,0,0,0}});
    Matrix IA = Matrix.Build.DenseOfArray(new double[,] {
             {0,0,0,0},
             {0,0,0,0},
             {0,0,0,0},
             {0,0,0,0}});

    void Start()
    {
        port.ReadTimeout = 2000;
        port.Close();
        if (!port.IsOpen)
            port.Open();
    }

    void Update()
    {
        CurrentReading = port.ReadLine();
        SubStrings = CurrentReading.Split(',');
        print(CurrentReading);
        q2[0] = Convert.ToSingle(SubStrings[0]);
        q2[1] = Convert.ToSingle(SubStrings[1]);
        q2[2] = Convert.ToSingle(SubStrings[2]);
        q2[3] = Convert.ToSingle(SubStrings[3]);
        ax = Convert.ToSingle(SubStrings[4]);
        ay = Convert.ToSingle(SubStrings[5]);
        az = Convert.ToSingle(SubStrings[6]);
        t2 = Convert.ToSingle(SubStrings[7]);

        B[0, 0] = q2[0] - q1[0];
        B[0, 1] = q2[1] - q1[1];
        B[0, 2] = q2[2] - q1[2];
        B[0, 3] = q2[3] - q1[3];

        A[0, 0] = q1[3] - q0[3];
        A[0, 1] = q1[2] - q0[2];
        A[0, 2] = -(q1[1] - q0[1]);
        A[0, 3] = q1[0] - q0[0];

        A[1, 0] = -(q1[2] - q0[2]);
        A[1, 1] = q1[3] - q0[3];
        A[1, 2] = q1[0] - q0[0];
        A[1, 3] = q1[1] - q0[1];

        A[2, 0] = q1[1] - q0[1];
        A[2, 1] = -(q1[2] - q0[2]);
        A[2, 2] = q1[3] - q0[3];
        A[2, 3] = q1[2] - q0[2];

        A[3, 0] = -(q1[1] - q0[1]);
        A[3, 1] = -(q1[1] - q0[1]);
        A[3, 2] = -(q1[2] - q0[2]);
        A[3, 3] = q1[3] - q0[3];
        IA = A.Inverse();
        Y = IA * B;
        dx = ((2 * Math.Acos(Y[0, 3])) / ((t2 - t1) * (Math.Pow(1 - Math.Pow(Y[0, 3], 2), 0.5)))) * Y[0, 0];
        dy = ((2 * Math.Acos(Y[0, 3])) / ((t2 - t1) * (Math.Pow(1 - Math.Pow(Y[0, 3], 2), 0.5)))) * Y[0, 1];
        dz = ((2 * Math.Acos(Y[0, 3])) / ((t2 - t1) * (Math.Pow(1 - Math.Pow(Y[0, 3], 2), 0.5)))) * Y[0, 2];
        if (dx == double.NaN)
        {
            ax = ax - Convert.ToSingle(dx);
        }
        if (dy == double.NaN)
        {
            ay = ay - Convert.ToSingle(dy);
        }
        if (dz == double.NaN)
        {
            az = az - Convert.ToSingle(dz);
        }
        q0[0] = q1[0];
        q0[1] = q1[1];
        q0[2] = q1[2];
        q0[3] = q1[3];
        q1[0] = q2[0];
        q1[1] = q2[1];
        q1[2] = q2[2];
        q1[3] = q2[3];
        t1 = t2;
        transform.localEulerAngles = new Vector3(az, ax, ay);
        
    }
}
