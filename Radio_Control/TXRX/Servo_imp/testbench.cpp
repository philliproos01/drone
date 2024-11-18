#include<Wire.h>
#include<Servo.h>

template <typename T_ty> struct TypeInfo { static const char * name; };
template <typename T_ty> const char * TypeInfo<T_ty>::name = "unknown";

// Handy macro to make querying stuff easier.
#define TYPE_NAME(var) TypeInfo< typeof(var) >::name

// Handy macro to make defining stuff easier.
#define MAKE_TYPE_INFO(type)  template <> const char * TypeInfo<type>::name = #type;

// Type-specific implementations.
MAKE_TYPE_INFO( int )
MAKE_TYPE_INFO( float )
MAKE_TYPE_INFO( short )


Servo horizontalServo;
Servo verticalServo;
Servo sideTiltServo;

const int MPU_addr=0x68;  // I2C address of the MPU-6050
float GyX,GyY,GyZ,AcX,AcY,AcZ,Tmp;

int i = 0;

long X = 0;
long Y = 0;
long Z = 0;

unsigned long previousMillis = 0;
unsigned long currentMillis = millis();
const long interval = 10;

unsigned long TIMERpreviousMillis = 0;
unsigned long TIMERcurrentMillis = millis();
const long TIMERinterval = 1;

int resetButtonPin = 12;

byte data = 0;
byte datav = 0;
byte datah = 0;

int horizontalPosition = 90;
int verticalPosition = 90;
int sideTiltPosition = 90;

int buttonCurrent = 0;
int buttonPrevious = 0;
unsigned char buttonHistory = 0;

int test = 0;

void setup()
{
  Serial.begin(9600);

  Serial.println("Initializing Gyroscope...");
  
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  horizontalServo.attach(5);
  horizontalServo.write(90);
  verticalServo.attach(6);
  verticalServo.write(90);
  

  Serial.println("Initialization Complete");
}

void loop()
{
  /*
  TIMERcurrentMillis = millis();
  if (TIMERcurrentMillis - TIMERpreviousMillis >= TIMERinterval)
  {
    //reset timer
    TIMERpreviousMillis = TIMERcurrentMillis;

    buttonPrevious = buttonCurrent;
    if(digitalRead(resetButtonPin) == HIGH)
    {
      buttonCurrent = 1;
    }
    else
    {
      buttonCurrent = 0;
    }
    buttonHistory = (buttonHistory << 1) | buttonCurrent;

    if((buttonPrevious == 0)&&(buttonCurrent == 1))
    {
      test = 0;
    }
  }

  if(test == 0)
  {
    if(buttonHistory == 0x0)
    {
      test = 1;
      
      sideTiltPosition = 90;
      Serial2.write(0x6);
      Serial2.write(sideTiltPosition);
      verticalPosition = 90;
      Serial2.write(0x7);
      Serial2.write(verticalPosition);
      horizontalPosition = 90;
      Serial2.write(0x8);
      Serial2.write(horizontalPosition);
    }
  }
  */
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);  // starting with register 0x43 (GYRO_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 6 registers
  
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  GyX=GyX/131;
  GyY=GyY/131;
  GyZ=GyZ/131;
  //Serial.println( TYPE_NAME(Wire.read()<<8|Wire.read()) );

  X = X + GyX;
  Y = Y + GyY;
  Z = Z + GyZ;

  i = i + 1;
  
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {    
    //take average
    X = X/i;
    Y = Y/i;
    Z = Z/i;

    //multiply by 0.01 (ten milliseconds)
    X = X*0.035;
    Y = Y*0.035;
    Z = Z*0.035;

    //reset i
    i = 0;

    //reset timer
    previousMillis = currentMillis;



    //-------------------------------------------------------------------------------------
    sideTiltPosition = sideTiltPosition + X;

    if((sideTiltPosition <= 180) && (sideTiltPosition >= 0))
    {
      //do nothing
    }
    else if(sideTiltPosition > 180)
    {
      sideTiltPosition = 180;
    }
    else if(horizontalPosition < 0)
    {
      sideTiltPosition = 0;
    }
    datav = sideTiltPosition & 0xFF;
    //Serial.println("0x6");
    //Serial.println(data);
    //delay(5);
    //-------------------------------------------------------------------------------------


    //-------------------------------------------------------------------------------------
    verticalPosition = verticalPosition + Y;

    if((verticalPosition <= 180) && (verticalPosition >= 0))
    {
      //do nothing
    }
    else if(verticalPosition > 180)
    {
      verticalPosition = 180;
    }
    else if(verticalPosition < 0)
    {
      verticalPosition = 0;
    }
    data = verticalPosition & 0xFF;
    //Serial.println("0x7");
    //Serial.println(data);
    //delay(5);
    //-------------------------------------------------------------------------------------
    
    
    //-------------------------------------------------------------------------------------
    horizontalPosition = horizontalPosition + Z;

    if((horizontalPosition <= 180) && (horizontalPosition >= 0))
    {
      //do nothing
    }
    else if(horizontalPosition > 180)
    {
      horizontalPosition = 180;
    }
    else if(horizontalPosition < 0)
    {
      horizontalPosition = 0;
    }
    datah = horizontalPosition & 0xFF;
    Serial.println(Z);
    Serial.println(datah);
    //Serial.println("0x8");
    //Serial.println(data);
    //delay(5);
    //-------------------------------------------------------------------------------------

    horizontalServo.write(datah);
    verticalServo.write(datav);
    //reset averages
    X = 0;
    Y = 0;
    Z = 0;
  }
}
