#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <RadioLib.h>


Adafruit_BNO055 bno = Adafruit_BNO055(55);
SX1278 radio = new Module(33, 32, 25);

void setFlag(void);
void print_data();
// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;

void setup() {
  Serial.begin(9600);
  
  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring or I2C address!");
    while (1);
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);


  //init radio
  Serial.print(F("[SX1278] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(1); }
  }
  // set the function that will be called
  // when packet transmission is finished
  radio.setPacketSentAction(setFlag);

  // start transmitting the first packet
  Serial.print(F("[SX1278] Sending first packet ... "));

  // you can transmit C-string or Arduino string up to
  // 255 characters long
  transmissionState = radio.startTransmit("accel data: ");
}


// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;

// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  // we sent a packet, set the flag
  transmittedFlag = true;
}

int count = 0;

void loop() {
  imu::Quaternion quat = bno.getQuat();
  
  // Convert quaternion to Euler angles
  double w = quat.w();
  double x = quat.x();
  double y = quat.y();
  double z = quat.z();
  
  // Roll (x-axis rotation)
  double sinr_cosp = 2 * (w * x + y * z);
  double cosr_cosp = 1 - 2 * (x * x + y * y);
  double roll = atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  double sinp = 2 * (w * y - z * x);
  double pitch;
  if (abs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // Yaw (z-axis rotation)
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  double yaw = atan2(siny_cosp, cosy_cosp);

  // Convert radians to degrees
  roll *= 180.0 / M_PI;
  pitch *= 180.0 / M_PI;
  yaw *= 180.0 / M_PI;

  // Print roll, pitch, and yaw
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);

  delay(100); // Adjust delay as needed

  //send over radio
  // check if the previous transmission finished
  if(transmittedFlag) {
    // reset flag
    transmittedFlag = false;

    if (transmissionState == RADIOLIB_ERR_NONE) {
      // packet was successfully sent
      Serial.println(F("transmission finished!"));

      // NOTE: when using interrupt-driven transmit method,
      //       it is not possible to automatically measure
      //       transmission data rate using getDataRate()

    } else {
      Serial.print(F("failed, code "));
      Serial.println(transmissionState);

    }

    // clean up after transmission is finished
    // this will ensure transmitter is disabled,
    // RF switch is powered down etc.
    radio.finishTransmit();


    String data = "Data: " + String(roll) + " " + String(pitch) + " " + String(yaw);

    transmissionState = radio.startTransmit(data);
  
  }
}
