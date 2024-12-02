#include <Arduino.h>
#include "Arduino_BMI270_BMM150.h"
#include "SensorFusion.h"

SF fusion;

float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float pitch, roll, yaw;
float deltat;

float b[3] = {48.378279, -2.921956, -41.759007}; // Combined bias vector
float A_inv[3][3] = { // Inverse of combined scale factors, misalignments, and soft iron effects
  {1.282660, -0.001886, -0.028071},
  {-0.001886, 1.219533, -0.018117},
  {-0.028071, -0.018117, 1.123966}
};

void calibrateMagnetometer(float h[3], float H_calibrated[3]);

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

void setup() {
  Serial.begin(115200);
  while (!Serial);
  myIMU.begin();
  Serial.print("Accelerometer sample rate = ");
  Serial.println(myIMU.accelerationSampleRate());
  delay(500);
}

void loop() {
  //static uint32_t prev_ms = millis();
  //uint32_t now = millis();
  //float dt = (now - prev_ms) / 1000.0f;
  //prev_ms = now;

  if (myIMU.accelerationAvailable()) {
    myIMU.readAcceleration(ax, ay, az);
  }

  if (myIMU.gyroscopeAvailable()) {
    myIMU.readGyroscope(gx, gy, gz);
  }

  if (myIMU.magneticFieldAvailable()) {
    myIMU.readMagneticField(mx, my, mz);
  }

  float h[3] = {mx, my, mz};
  float H_calibrated[3];
  calibrateMagnetometer(h, H_calibrated);

  // Convert to proper units
  ax *= 9.81f;
  ay *= 9.81f;
  az *= 9.81f;
  gx *= (PI / 180.0f);
  gy *= (PI / 180.0f);
  gz *= (PI / 180.0f);

  // Update sensor fusion
  //fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, H_calibrated[0], H_calibrated[1], H_calibrated[2], dt);
  //deltat = fusion.deltatUpdate();
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, H_calibrated[0], -H_calibrated[1], -H_calibrated[2], fusion.deltatUpdate());
  // Get Euler angles
  roll = fusion.getRoll();
  pitch = fusion.getPitch();
  yaw = fusion.getYaw();

  // Print results
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);

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
