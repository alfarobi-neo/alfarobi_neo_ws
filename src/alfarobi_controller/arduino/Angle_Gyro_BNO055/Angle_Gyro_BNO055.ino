#include "BNO055_support.h"		//Contains the bridge code between the API and Arduino
#include <Wire.h>

//The device address is set to BNO055_I2C_ADDR2 in this example. You can change this in the BNO055.h file in the code segment shown below.
// /* bno055 I2C Address */
// #define BNO055_I2C_ADDR1                0x28
// #define BNO055_I2C_ADDR2                0x29
// #define BNO055_I2C_ADDR                 BNO055_I2C_ADDR2

//This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data
struct bno055_accel myAccelData;
struct bno055_gyro  myGyroData;
struct bno055_linear_accel myLinearAccelData;
struct bno055_mag myMagneto;


float angleYaw, angleRoll, anglePitch;
float gyroYaw, gyroRoll, gyroPitch;
float magX, magY, magZ;

unsigned long lastTime = 0;

void setup() //This code is executed once
{
  //Initialize I2C communication
  Wire.begin(); 
  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device
  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  delay(1);
  //Initialize the Serial Port to view information on the Serial Monitor
  Serial.begin(115200);
}

void loop() //This code is looped forever
{
  if ((millis() - lastTime) >= 100) //To stream at 10Hz without using additional timers
  {
    lastTime = millis();

    bno055_read_euler_hrp(&myEulerData);			//Update Euler data into the structure=
    bno055_read_accel_xyz(&myAccelData);
    bno055_read_gyro_xyz(&myGyroData);
    bno055_read_linear_accel_xyz(&myLinearAccelData);
    bno055_read_mag_xyz(&myMagneto);
    //hrp
    angleYaw   = float(myEulerData.h)/16.00; //Convert to degrees
    angleRoll  = float(myEulerData.r)/16.00; 
    anglePitch = float(myEulerData.p)/16.00;
    
//    printAngle();
    sendAngle();

  }
}

void sendAngle()
{
  Serial.print('U');
  Serial.print(String(angleRoll));
  Serial.print('G');
  Serial.print(String(anglePitch));
  Serial.print('M');
  Serial.print(String(angleYaw));
  Serial.print('A');
  
}

void printAngle()
{
    Serial.print("Yaw: ");        //To read out the Heading (Yaw)
    Serial.print(float(myEulerData.h) / 16.00);   //Convert to degrees
    Serial.print("\t");
    
    Serial.print("Roll: ");         //To read out the Roll
    Serial.print(float(myEulerData.r) / 16.00);   //Convert to degrees
    Serial.print("\t");
    
    Serial.print("Pitch: ");        //To read out the Pitch
    Serial.println(float(myEulerData.p) / 16.00);   //Convert to degrees
}

