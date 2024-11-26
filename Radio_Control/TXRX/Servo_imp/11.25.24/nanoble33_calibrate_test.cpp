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

  calculate_IMU_error();

  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  

  
  if (myIMU.accelerationAvailable()) {
    myIMU.readAcceleration(ax, ay, az);
     Serial.print("accel: \t");
      Serial.print(ax);
      Serial.print('\t');
      Serial.print(ay);
      Serial.print('\t');
      Serial.print(az);
      Serial.println();
  }

  
  if (myIMU.gyroscopeAvailable()) {
    myIMU.readGyroscope(gx, gy, gz);
     Serial.print("gyro: \t");
    Serial.print(gx);
    Serial.print('\t');
    Serial.print(gy);
    Serial.print('\t');
    Serial.print(gz);
    Serial.println();
  }
  
  
  if (myIMU.magneticFieldAvailable()) {

    myIMU.readMagneticField(mx, my, mz);
     Serial.print("mag: \t");
    Serial.print(mx);
    Serial.print('\t');
    Serial.print(my);
    Serial.print('\t');
    Serial.print(mz);
    Serial.println();
  }
  delay(10);
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
