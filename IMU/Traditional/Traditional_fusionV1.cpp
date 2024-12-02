#include <Arduino.h>
#include "Arduino_BMI270_BMM150.h"

float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float pitch, roll, yaw;
float deltat;
void calculate_IMU_error();
int c = 0;

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;

float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;

void print_data();
float b[3] = {48.378279, -2.921956, -41.759007};  // Combined bias vector
float A_inv[3][3] = {    // Inverse of combined scale factors, misalignments, and soft iron effects
  {1.282660, -0.001886, -0.028071},
  {-0.001886, 1.219533, -0.018117},
  {-0.028071, -0.018117, 1.123966}
};

void calibrateMagnetometer(float h[3], float H_calibrated[3]);
void setCalibrationParameters(float bias[3], float A_inverse[3][3]);

class MyBoschSensor: public BoschSensorClass {

  public:
    MyBoschSensor(TwoWire& wire = Wire) : BoschSensorClass(wire) {};

  protected:
    virtual int8_t configure_sensor(struct bmi2_dev *dev)
    {
      int8_t rslt;
      uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

      struct bmi2_int_pin_config int_pin_cfg;
      int_pin_cfg.pin_type = BMI2_INT1;
      int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
      int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
      int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
      int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
      int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

      struct bmi2_sens_config sens_cfg[2];
      sens_cfg[0].type = BMI2_ACCEL;
      sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
      sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_25HZ;
      sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
      sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
      sens_cfg[1].type = BMI2_GYRO;
      sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
      sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
      sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_25HZ;
      sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
      sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

      rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
      if (rslt != BMI2_OK)
        return rslt;

      rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
      if (rslt != BMI2_OK)
        return rslt;

      rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
      if (rslt != BMI2_OK)
        return rslt;

      rslt = bmi2_sensor_enable(sens_list, 2, dev);
      if (rslt != BMI2_OK)
        return rslt;

      return rslt;
    }
};

MyBoschSensor myIMU(Wire1);

void print_data() {
  // we can also read accelerometer / gyro data here!
  //Serial.println("Got new data!");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);
  //myIMU.debug(Serial);
  //myIMU.onInterrupt(print_data);
  myIMU.begin();

  Serial.print("Accelerometer sample rate = ");
  Serial.println(myIMU.accelerationSampleRate());

  //calculate_IMU_error();

  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  

  
  if (myIMU.accelerationAvailable()) {
    myIMU.readAcceleration(AccX, AccY, AccZ);

  }
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 5.19; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 0.96; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  
  if (myIMU.gyroscopeAvailable()) {
    myIMU.readGyroscope(GyroX, GyroY, GyroZ);
     
  }
  GyroX = GyroX + 0.0007; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 0.001; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.0007; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
  if (myIMU.magneticFieldAvailable()) {

    myIMU.readMagneticField(mx, my, mz);
    
  }
  float h[3] = {mx, my, mz};
  float H_calibrated[3];
  calibrateMagnetometer(h, H_calibrated);
  
  //magnnetometer debugging
  /*Serial.print(H_calibrated[0]);
  Serial.print("/");
  Serial.print(H_calibrated[1]);
  Serial.print("/");
  Serial.println(H_calibrated[2]);*/


  //roll, yaw, pitch debugging

  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);*/
  //delay(10);
}

void calibrateMagnetometer(float h[3], float H_calibrated[3]) {
  // Subtract bias: (h - b)
  float temp[3];
  for (int i = 0; i < 3; i++) {
    temp[i] = h[i] - b[i];
  }
  
  // Multiply by A_inv: A^-1 * (h - b)
  for (int i = 0; i < 3; i++) {
    H_calibrated[i] = 0;
    for (int j = 0; j < 3; j++) {
      H_calibrated[i] += A_inv[i][j] * temp[j];
    }
  }
}

void setCalibrationParameters(float bias[3], float A_inverse[3][3]) {
  // Set bias vector
  for (int i = 0; i < 3; i++) {
    b[i] = bias[i];
  }
  
  // Set A_inv matrix
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      A_inv[i][j] = A_inverse[i][j];
    }
  }
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  /*while (!myIMU.accelerationAvailable()) {
    if (myIMU.accelerationAvailable()) {
      myIMU.readAcceleration(ax, ay, az);
    }
  }
  if (myIMU.accelerationAvailable()) {
    myIMU.readAcceleration(ax, ay, az);
     Serial.print("Starup accel: \t");
      Serial.print(ax);
      Serial.print('\t');
      Serial.print(ay);
      Serial.print('\t');
      Serial.print(az);
      Serial.println();
      Serial.print("Startup accel complete\t");
  }*/

  while (c < 200) {
    
    if (myIMU.accelerationAvailable()) {
      myIMU.readAcceleration(ax, ay, az);
    }
    
    AccX = (ax);
    AccY = (ay);
    AccZ = (az);
    Serial.println(ax);
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    delay(50);
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    /*Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);*/
    
    
    if (myIMU.gyroscopeAvailable()) {
      myIMU.readGyroscope(gx, gy, gz);
    }
    
    GyroX = gx;
    GyroY = gy;
    GyroZ = gz;
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}
