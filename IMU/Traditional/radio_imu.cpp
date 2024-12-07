#include <Arduino_BMI270_BMM150.h>
#include <math.h>
#include <RadioLib.h>
//#include <Servo.h>

// Magnetometer calibration parameters
float b[3] = {48.648271, 1.461257, -44.551660};
float A_inv[3][3] = {
  {1.125686, -0.003850, -0.011228},
  {-0.003850, 1.081973, -0.015144},
  {-0.011228, -0.015144, 1.001810}
};

float roll, pitch, yaw;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;
float dt = 0.01; // 10ms sample rate



SX1278 radio = new Module(4, 3, 2);
int transmissionState = RADIOLIB_ERR_NONE;

// Variables to store previous angles for servo control
float prevYaw = 0;
float prevPitch = 0;

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

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print(F("[SX1278] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(1); }
  }

  radio.setPacketSentAction(setFlag);

  // start transmitting the first packet
  Serial.print(F("[SX1278] Sending first packet ... "));

  // you can transmit C-string or Arduino string up to
  // 255 characters long
  transmissionState = radio.startTransmit("accel data: ");


  // Set initial position to center (90 degrees)
  
}

void loop() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    IMU.readMagneticField(magX, magY, magZ);

    gyroX = gyroX + 0.0007;
    gyroY = gyroY - 0.001;
    gyroZ = gyroZ + 0.0007;
    // Convert magnetometer data from µT to Gauss
    magX /= 100.0;
    magY /= -100.0; // Invert Y
    magZ /= -100.0; // Invert Z

    // Apply magnetometer calibration
    float magCorrected[3];
    for (int i = 0; i < 3; i++) {
      magCorrected[i] = A_inv[i][0] * (magX - b[0]) + A_inv[i][1] * (magY - b[1]) + A_inv[i][2] * (magZ - b[2]);
    }

    // Calculate roll and pitch from accelerometer
    roll = atan2(accY, accZ);
    pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ));

    // Calculate yaw from magnetometer
    float sinRoll = sin(roll);
    float cosRoll = cos(roll);
    float sinPitch = sin(pitch);
    float cosPitch = cos(pitch);

    float Bfy = magCorrected[1] * cosRoll - magCorrected[2] * sinRoll;
    float Bfz = magCorrected[1] * sinRoll + magCorrected[2] * cosRoll;

    yaw = atan2(Bfy, magCorrected[0] * cosPitch + Bfz * sinPitch);

    // Convert radians to degrees
    roll *= 180.0 / M_PI;
    pitch *= 180.0 / M_PI;
    yaw *= 180.0 / M_PI;

    // Control servos based on yaw and pitch changes
    /*float yawChange = yaw - prevYaw;
    float pitchChange = pitch - prevPitch;*/

    // Update servo positions
    
   


    // Print the results
    Serial.print(roll);
    Serial.print("/");
    Serial.print(pitch);
    Serial.print("/");
    Serial.println(yaw);



    //transmit across Radio
    if(transmittedFlag) {
      // reset flag
      transmittedFlag = false;

      if (transmissionState == RADIOLIB_ERR_NONE) {
        // packet was successfully sent
        //Serial.println(F("transmission finished!"));

        // NOTE: when using interrupt-driven transmit method,
        //       it is not possible to automatically measure
        //       transmission data rate using getDataRate()

      } else {
        //Serial.print(F("failed, code "));
        //Serial.println(transmissionState);

      }

      // clean up after transmission is finished
      // this will ensure transmitter is disabled,
      // RF switch is powered down etc.
      radio.finishTransmit();

      //Serial.print(F("[SX1278] Sending another packet ... "));
      String data = "Data: " + String(roll) + " " + String(pitch) + " " + String(yaw);
      //reset averages
     

      transmissionState = radio.startTransmit(data);
    }

  }

  delay(10); // 10ms delay for 100Hz update rate
}
