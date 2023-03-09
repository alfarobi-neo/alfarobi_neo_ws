////////////////////////////:::BNO:::///////////////////////////////////
#include "BNO055_support.h"    //Contains the bridge code between the API and Arduino
#include <Wire.h>
//The device address is set to BNO055_I2C_ADDR2 in this example. You can change this in the BNO055.h file in the code segment shown below.
// /* bno055 I2C Address */
// #define BNO055_I2C_ADDR1                0x28
// #define BNO055_I2C_ADDR2                0x29
// #define BNO055_I2C_ADDR                 BNO055_I2C_ADDR2
//This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data
struct bno055_gyro  myGyroData;
float angleYaw, angleRoll, anglePitch;
float gyroYaw, gyroRoll, gyroPitch;
unsigned long lastTime = 0;
///////////////////////////////////////////////////////////////////////

//Buzzer & button variables
byte buzzer=9;
int on=0; 
float sensorV=A3;
const int buttonR=A1;
const int buttonL=A0;
int numButton;
float voltage, bR, bL;
int bootState = 0;
bool battState = 0;
bool acState = 0;
unsigned long currentMillis = 0;
long previousMillis = 0;

void setup()
{
  //BNO055 IMU Initialize
  //Initialize I2C communication
  Wire.begin(); 
  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device
  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  delay(1);
  //Initialize the Serial Port to view information on the Serial Monitor
  Serial.begin(115200);
  
  pinMode(buzzer,OUTPUT);
}

void loop()
{
  currentMillis = millis();
//BOOTING-BEEP
//  /bootingBeep();
  
//ANALOG-READ
/////////////Voltase///////////////////////////
 voltage=analogRead(sensorV);
 voltage=voltage*0.024219653;
 ////////////ButtonR//////////////////////////
 bR=analogRead(buttonR);
 ////////////ButtonL//////////////////////////
 bL=analogRead(buttonL);
 
/*
//BATT-WARNING
  if(voltage<12)
  {
    batteryWarning(1);
  } 

//AC-MODE
  if(voltage>12 && voltage<13.8 && on==1)
  {
     ACwarning();
  }
 
//BATT-WARNING
  if(voltage>13.8 && voltage<14.2)
  {
    batteryWarning(2);
  } 
 */
 if(bR>450 && bR<1050)
  {
    if(bR>900 && bR<1030 ){numButton = 1;}
    else if(bR>800 && bR<880 ){numButton = 2;}
    else if(bR>650&& bR<750 ){numButton = 3;}
    else if(bR>500 && bR<600){numButton = 4;}
    digitalWrite(buzzer,HIGH); //delay(10);
   }
/////////////////////////BUTTON LEFT/////////////////////////////////////
   else if(bL>450 && bL<1050)
   {
    if(bL>900 && bL<1030 ){numButton = 5;}
    else if(bL>800 && bL<880 ){numButton = 6;}
    else if(bL>650&& bL<750 ){numButton = 7;}
    else if(bL>500 && bL<600){numButton = 8;}
    digitalWrite(buzzer,HIGH); //delay(10);
   }
   else
   {
      digitalWrite(buzzer,LOW); //delay(10);
   } 

////////////////////////////:::BNO:::///////////////////////////////////
  if ((millis() - lastTime) >= 100) //To stream at 10Hz without using additional timers
  {
    lastTime = millis();

    bno055_read_euler_hrp(&myEulerData);      //Update Euler data into the structure=
    bno055_read_gyro_xyz(&myGyroData);
    //hrp
    angleYaw   = float(myEulerData.h)/16.00; //Convert to degrees
    angleRoll  = float(myEulerData.r)/16.00; 
    anglePitch = float(myEulerData.p)/16.00;
    gyroYaw   = float(myGyroData.z)/16.00; 
    gyroRoll  = float(myGyroData.y)/16.00; 
    gyroPitch = float(myGyroData.x)/16.00;
    
//    printAngle();
//    printGyro();
//    printButton();
    sendAngle();
    sendGyro();
    sendButton();
    Serial.print('A'); //Ending of string
    //SERIAL-PRINT-VOLTAGE
//    Serial.print(voltage); Serial.println(" Volt");
  }
///////////////////////////////////////////////////////////////////////

////END-VOIDLOOP////
 
}

////////////////////////////:::BNO:::///////////////////////////////////

void sendAngle()
{
  Serial.print('U');
  Serial.print(String(angleRoll));
  Serial.print('G');
  Serial.print(String(anglePitch));
  Serial.print('M');
  Serial.print(String(angleYaw));
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

void sendGyro()
{
  Serial.print('D');
  Serial.print(String(gyroRoll));
  Serial.print('I');
  Serial.print(String(gyroPitch));
  Serial.print('Y');
  Serial.print(String(gyroYaw));
}

void printGyro()
{
  Serial.print("GyroYaw: ");
  Serial.print(gyroYaw);
  Serial.print("\t");
  Serial.print("GyroRoll: ");
  Serial.print(gyroRoll);
   Serial.print("\t");
  Serial.print("GyroPitch: ");
  Serial.println(gyroPitch);
}

void printButton()
{
  ////////////////////////BUTTON RIGHT/////////////////////////////////////
  if(bR>450 && bR<1050)
  {
    Serial.print("Button R: ");
//    Serial.print(bR);
    if(bR>900 && bR<1030 ){Serial.println("1");}
    else if(bR>800 && bR<880 ){Serial.println("2");}
    else if(bR>650&& bR<750 ){Serial.println("3");}
    else if(bR>500 && bR<600){Serial.println("4");}
    Serial.println();
    digitalWrite(buzzer,HIGH); //delay(10);
   }
/////////////////////////BUTTON LEFT/////////////////////////////////////
   else if(bL>450 && bL<1050)
   {
    Serial.print("Button L: ");
//    Serial.println(bL);
    if(bL>900 && bL<1030 ){Serial.print("5");}
    else if(bL>800 && bL<880 ){Serial.print("6");}
    else if(bL>650&& bL<750 ){Serial.print("7");}
    else if(bL>500 && bL<600){Serial.print("8");}
    Serial.println();
    digitalWrite(buzzer,HIGH); //delay(10);
   }
   else
   {
      Serial.println();
      digitalWrite(buzzer,LOW); //delay(10);
   }
}

void sendButton()
{
  Serial.print('B');
  Serial.print(String(numButton));
}

void bootingBeep() 
{
  if(on==0)
  {
    //state, currentNum, nextNum, logic, duration
    rectWave(bootState, 0, 1, HIGH, 1000); Serial.println(0);
    rectWave(bootState, 1, 2, LOW, 1000);Serial.println(1);
    rectWave(bootState, 2, 3, HIGH, 100);Serial.println(2);
    rectWave(bootState, 3, 4, LOW, 10);Serial.println(3);
    rectWave(bootState, 4, 5, HIGH, 100);Serial.println(4);
    rectWave(bootState, 5, 6, HIGH, 100);Serial.println(5);
    on++;
  }
}

void batteryWarning(int warningMode)
{
    if((battState == 0) && ((currentMillis - previousMillis) < 1000/warningMode))
    {
      digitalWrite(buzzer, HIGH);
//      Serial.print("battState: "); Serial.print(battState); Serial.println(" ON");
//      Serial.println(currentMillis - previousMillis);
    }
    else if((battState == 0) && (currentMillis - previousMillis) >= 1000/warningMode)
    {
        battState = 1;
        previousMillis = currentMillis;
//        Serial.print("ChangeToLow");
//        Serial.println(currentMillis - previousMillis);
    }
    else if((battState == 1) && ((currentMillis - previousMillis) < 100/warningMode))
    {
      digitalWrite(buzzer, LOW);
//      Serial.print("battState: "); Serial.print(battState); Serial.println(" OFF");
//      Serial.println(currentMillis - previousMillis);
    }
    else if((battState == 1) && (currentMillis - previousMillis) >= 100/warningMode)
    {
        battState = 0;
        previousMillis = currentMillis;
//        Serial.print("ChangetoHigh");
//        Serial.println(currentMillis - previousMillis);
    }
}

void rectWave(int &state, int currentNum, int nextNum, bool logic, int duration)
{
    if((state == currentNum) && ((currentMillis - previousMillis) < duration))
    {
      digitalWrite(buzzer, logic);
//      Serial.print("state: "); Serial.print(currentNum); Serial.println(logic);
//      Serial.println(currentMillis - previousMillis);
    }
    else if((state == 0) && (currentMillis - previousMillis) >= duration)
    {
        state = nextNum;
        previousMillis = currentMillis;
//        Serial.print("ChangeTo:"); Serial.print(!logic);
//        Serial.println(currentMillis - previousMillis);
    }
}

void ACwarning()
{
  if((acState == 0) && ((currentMillis - previousMillis) < 2000))
    {
      digitalWrite(buzzer, HIGH);
      if((currentMillis - previousMillis) >= 2000)
      {
        acState = 1;
        previousMillis = currentMillis;
      }
    }
    else if((acState == 1) && ((currentMillis - previousMillis) < 10))
    {
      digitalWrite(buzzer, LOW);
      if((currentMillis - previousMillis) >= 10)
      {
        acState = 0;
        previousMillis = currentMillis;
      }
    }
}
///////////////////////////////////////////////////////////////////////

