
#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>
#include <math.h>

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

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
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

    // Print the results
    Serial.print(roll);
    Serial.print("/");
    Serial.print(pitch);
    Serial.print("/");
    Serial.println(yaw);
  }

  delay(10); // 10ms delay for 100Hz update rate
}
