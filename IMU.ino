#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <TimeLib.h>

#include <Wire.h>
float angles[3];
float q[4]; // yaw pitch roll
float t;
float temperature;

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();
ITG3200 gyro = ITG3200();

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(5);
  gyro.init(0x68, NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_XGYRO_REF, true, true);
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
}


void loop() {  
  sixDOF.getQ(q); 
  sixDOF.getAngles(angles); 
  gyro.readTemp(&temperature);
  t = millis();
  Serial.print(q[0]);
  Serial.print(",");
  Serial.print(q[1]);
  Serial.print(",");
  Serial.print(q[2]);
  Serial.print(","); 
  Serial.print(q[3]);
  Serial.print(",");   
  Serial.print(angles[0]);
  Serial.print(",");   
  Serial.print(angles[1]);
  Serial.print(",");   
  Serial.print(angles[2]);
  Serial.print(",");   
  Serial.print(temperature);
  Serial.print(",");  
  Serial.print(t);  
  Serial.println(",");
}
