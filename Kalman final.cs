//Luis Andrés Puértolas Bálint
//Queen Mary University of London
//Advanced Robotics Lab
//l.a.puertolasbalint@qmul.ac.uk
//Do not use without authorization

using System;
using System.IO.Ports;
using MathNet.Filtering.Kalman;
using Matrix = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using MathNet.Spatial.Euclidean;

namespace Kalman_Filter_Nuget
{
    class Program
    {
        static void Main(string[] args)
        {
            double[] IndividualValues = { 0, 0, 0, 0, 0, 0 };
            String[] SubStrings;
            float[] q = { 0, 0, 0, 0 };
            double x=0, xq, y = 0, z = 0, t2 = 0, x1, y1, z1, time, velx, vely, velz, accx, accy, accz, temp;
            bool correctRead;
            int comaCount = 0;
            char[] test;
            SerialPort port = new SerialPort("COM3", 19200);
            String CurrentReading = "ABCD";
            Matrix initial = Matrix.Build.DenseOfArray(new double[,] {
                         {0},
                         {0},
                         {0},
                         {0},
                         {0},
                         {0},
                         {0},
                         {0},
                         {0} });
            Matrix covariance = Matrix.Build.DenseOfArray(new double[,] {
                         {0.1,0,0,0,0,0,0,0,0},
                         {0,0.1,0,0,0,0,0,0,0},
                         {0,0,0.1,0,0,0,0,0,0},
                         {0,0,0,0.1,0,0,0,0,0},
                         {0,0,0,0,0.1,0,0,0,0},
                         {0,0,0,0,0,0.1,0,0,0},
                         {0,0,0,0,0,0,0.1,0,0},
                         {0,0,0,0,0,0,0,0.1,0},
                         {0,0,0,0,0,0,0,0,0.1 } });
            DiscreteKalmanFilter kalman = new DiscreteKalmanFilter(initial, covariance);

            port.ReadTimeout = 2000;
            port.Close();
            if (!port.IsOpen)
                port.Open();

            while (t2 <= 20000000000)
            {
                CurrentReading = port.ReadLine();
                test = CurrentReading.ToCharArray();
                correctRead = true;
                comaCount = 0;
                for (int i = test.Length - 1; i >= 0; i--)
                {
                    if (Char.IsNumber(test[i]) == false & test[i] != ',' & test[i] != '.' & Char.IsWhiteSpace(test[i]) == false & test[i] != '-')
                    {
                        correctRead = false;
                    }
                    if (test[i] == ',')
                    {
                        comaCount = comaCount + 1;
                    }
                }
                if (correctRead == true & comaCount == 9)
                {
                    SubStrings = CurrentReading.Split(',');
                    q[0] = Convert.ToSingle(SubStrings[0]);
                    q[1] = Convert.ToSingle(SubStrings[1]);
                    q[2] = Convert.ToSingle(SubStrings[2]);
                    q[3] = Convert.ToSingle(SubStrings[3]);
                    Quaternion quaternion = new Quaternion(q[0], q[1], q[2], q[3]);
                    x1 = Convert.ToDouble(SubStrings[4]);
                    y1 = Convert.ToDouble(SubStrings[5]);
                    z1 = Convert.ToDouble(SubStrings[6]);
                    temp = Convert.ToDouble(SubStrings[7]);
                    time = Convert.ToSingle(SubStrings[8]);
                    velx = (x1 - x) / (time - t2);
                    vely = (y1 - y) / (time - t2);
                    velz = (z1 - z) / (time - t2);
                    accx = velx / (time - t2);
                    accy = vely / (time - t2);
                    accz = velz / (time - t2);
                    //Dummy Data : Generate a very noisy position estimate
                    Matrix Meassurement = Matrix.Build.DenseOfArray(new double[,] {
                         {0.1},
                         {0.1},
                         {0.1},
                         {0.1},
                         {0.1},
                         {0.1},
                         {0.1},
                         {0.1},
                         {0.1} });
                    Matrix Noise = Matrix.Build.DenseOfArray(new double[,] {
                         {50.94324709,0,0,0,0,0,0,0,0},
                         {0,345.3342043,0,0,0,0,0,0,0},
                         {0,0,184.5258328,0,0,0,0,0,0},
                         {0,0,0,0.000105439,0,0,0,0,0},
                         {0,0,0,0,0.000107133,0,0,0,0},
                         {0,0,0,0,0,0.000586316,0,0,0},
                         {0,0,0,0,0,0,5.7569E-06,0,0},
                         {0,0,0,0,0,0,0,1.20007E-05,0},
                         {0,0,0,0,0,0,0,0,3.71021E-052 } });
                    //veryfie
                    Matrix startetrans = Matrix.Build.DenseOfArray(new double[,] {
                         {x1,0,0,0,0,0,0,0,0},
                         {0,y1,0,0,0,0,0,0,0},
                         {0,0,z1,0,0,0,0,0,0},
                         {0,0,0,velx,0,0,0,0,0},
                         {0,0,0,0,vely,0,0,0,0},
                         {0,0,0,0,0,velz,0,0,0},
                         {0,0,0,0,0,0,accx,0,0},
                         {0,0,0,0,0,0,0,accy,0},
                         {0,0,0,0,0,0,0,0,accz } });

                    kalman.Predict(startetrans, kalman.Cov);
                    kalman.Update(Meassurement, Noise, covariance);

                    x1 += kalman.State.At(0, 0);
                    y1 += kalman.State.At(1, 0);
                    z1 += kalman.State.At(2, 0);
                    Console.WriteLine("x: " + quaternion.ToEulerAngles().Alpha.Degrees + ", y: "+ quaternion.ToEulerAngles().Beta.Degrees + ", z: " + z1 + ", temperature: " + temp + ", time: " + t2);
                    t2 = time;
                    x = x1;
                    y = y1;
                    z = z1;
                }
            }

        }
    }
}
