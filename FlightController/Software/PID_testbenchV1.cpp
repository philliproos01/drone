#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

// Create MPU6050 object
MPU6050 mpu;

// PID variables
double pitchInput, pitchOutput, pitchSetpoint;
double rollInput, rollOutput, rollSetpoint;
double yawInput, yawOutput, yawSetpoint;

// PID tunings
double Kp = 1.0, Ki = 0.1, Kd = 0.01;

// Create PID controllers
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT);
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, Kp, Ki, Kd, DIRECT);
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, Kp, Ki, Kd, DIRECT);

// Motor PWM outputs (for example, for 4 motors)
int motor1, motor2, motor3, motor4;

// Calibration constants (based on your specific sensor)
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

void setup() {
  Serial.begin(115200);
  
  Wire.begin();
  mpu.initialize();

  // Check if MPU6050 is connected
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  // Initialize PID controllers
  pitchSetpoint = 0; // Set to desired angle
  rollSetpoint = 0;
  yawSetpoint = 0;
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  yawPID.SetMode(AUTOMATIC);
}

void loop() {
  // Get sensor data (accelerometer and gyroscope)
  mpu.getAcceleration(&accelX, &accelY, &accelZ);
  mpu.getRotation(&gyroX, &gyroY, &gyroZ);

  // Convert raw accelerometer and gyro data to meaningful angles (simplified)
  // Assuming you have a good sensor calibration for more accurate results.
  pitchInput = atan2(accelY, accelZ) * 180.0 / PI;  // Pitch calculation (degrees)
  rollInput = atan2(accelX, accelZ) * 180.0 / PI;   // Roll calculation (degrees)
  yawInput = gyroZ / 131.0;  // Gyroscope gives angular velocity in degrees/sec

  // Compute PID outputs
  pitchPID.Compute();
  rollPID.Compute();
  yawPID.Compute();

  // Adjust motor speeds based on PID outputs
  motor1 = 1500 + pitchOutput - rollOutput + yawOutput;
  motor2 = 1500 + pitchOutput + rollOutput - yawOutput;
  motor3 = 1500 - pitchOutput - rollOutput - yawOutput;
  motor4 = 1500 - pitchOutput + rollOutput + yawOutput;

  // Limit PWM values (assuming PWM range is from 1000 to 2000)
  motor1 = constrain(motor1, 1000, 2000);
  motor2 = constrain(motor2, 1000, 2000);
  motor3 = constrain(motor3, 1000, 2000);
  motor4 = constrain(motor4, 1000, 2000);

  // Output the PWM values to the motors (assuming you use PWM pins)
  analogWrite(32, motor1);  // Motor 1 PWM pin (replace with actual pin)
  analogWrite(33, motor2);  // Motor 2 PWM pin
  analogWrite(34, motor3);  // Motor 3 PWM pin
  analogWrite(35, motor4);  // Motor 4 PWM pin

  // Print data for debugging
  Serial.print("Pitch: "); Serial.print(pitchInput); 
  Serial.print(" Roll: "); Serial.print(rollInput);
  Serial.print(" Yaw: "); Serial.print(yawInput);
  Serial.print(" PWM1: "); Serial.print(motor1);
  Serial.print(" PWM2: "); Serial.print(motor2);
  Serial.print(" PWM3: "); Serial.print(motor3);
  Serial.print(" PWM4: "); Serial.println(motor4);

  delay(20);  // Adjust loop frequency
}
