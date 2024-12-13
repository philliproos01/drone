#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ESP32Servo.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo panServo;
Servo tiltServo;

const int panPin = 26;
const int tiltPin = 27;

void setup() {
  Serial.begin(9600);
  
  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring or I2C address!");
    while (1);
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);

  panServo.attach(panPin, 500, 2500);
  tiltServo.attach(tiltPin, 500, 2500);
}

void loop() {
  imu::Quaternion quat = bno.getQuat();
  
  double w = quat.w();
  double x = quat.x();
  double y = quat.y();
  double z = quat.z();
  
  double sinr_cosp = 2 * (w * x + y * z);
  double cosr_cosp = 1 - 2 * (x * x + y * y);
  double roll = atan2(sinr_cosp, cosr_cosp);

  double sinp = 2 * (w * y - z * x);
  double pitch;
  if (abs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp);
  else
    pitch = asin(sinp);

  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  double yaw = atan2(siny_cosp, cosy_cosp);

  roll *= 180.0 / M_PI;
  pitch *= 180.0 / M_PI;
  yaw *= 180.0 / M_PI;

  int panAngle = map(yaw, -180, 180, 0, 180);
  int tiltAngle = map(pitch, -90, 90, 0, 180);

  panServo.write(panAngle);
  tiltServo.write(tiltAngle);

  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);

  delay(100);
}
