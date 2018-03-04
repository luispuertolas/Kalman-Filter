using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;
using System;
using System.Text;
using System.Threading;
using System.IO;

//using Kalman;


public class RightForeArmController : MonoBehaviour
{
    private SerialPort port = new SerialPort("COM3", 115200);
    private String CurrentReading = "ABCD";
    private float[] IndividualValues = { 0, 0, 0, 0, 0, 0 };
    private String[] SubStrings;
    string path = "Assets/test.txt";

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
        IndividualValues[0] = Convert.ToSingle(SubStrings[0]);
        IndividualValues[1] = Convert.ToSingle(SubStrings[1]);
        IndividualValues[2] = Convert.ToSingle(SubStrings[2]);
        transform.localEulerAngles = new Vector3(IndividualValues[2], IndividualValues[0], -IndividualValues[2]);
        StreamWriter writer = new StreamWriter(path, true);
        writer.WriteLine(CurrentReading);
        writer.Close();

    }

}