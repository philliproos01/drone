//#include "MPU9250.h"		// https://github.com/bolderflight/MPU9250 	
#include <RadioLib.h>
#include "Streaming.h" 		// needed for the Serial output https://github.com/geneReeves/ArduinoStreaming
#include "SensorFusion.h"
#include "Arduino_BMI270_BMM150.h"

SF fusion;

float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float pitch, roll, yaw;
float deltat;

//#define SS_PIN PB12
//SPIClass mySPI (2);
//MPU9250 IMU(mySPI, SS_PIN);
int status;

//#define EULER_DATA
//#define RAW_DATA
#define PROCESSING
//#define SERIAL_PLOTER



//BLE 9DOF sensor
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
//sensor setup ends here
MyBoschSensor myIMU(Wire1);
void setFlag(void);
void print_data();


void setup() {
  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}

  // start communication with IMU
  myIMU.debug(Serial);
  myIMU.onInterrupt(print_data);
  myIMU.begin();
  

  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  Serial.println(myIMU.accelerationSampleRate());
  



}

void loop() {

  
  if (myIMU.accelerationAvailable()) {
    myIMU.readAcceleration(ax, ay, az);
  }

  if (myIMU.gyroscopeAvailable()) {
    myIMU.readGyroscope(gx, gy, gz);
  }
  
  if (myIMU.magneticFieldAvailable()) {

    myIMU.readMagneticField(mx, my, mz);
  }
  
  /*ax = IMU.getAccelX_mss();
  ay = IMU.getAccelY_mss();
  az = IMU.getAccelZ_mss();
  gx = IMU.getGyroX_rads();
  gy = IMU.getGyroY_rads();
  gz = IMU.getGyroZ_rads();
  mx = IMU.getMagX_uT();
  my = IMU.getMagY_uT();
  mz = IMU.getMagZ_uT();*/
  temp = 11;

#ifdef RAW_DATA
  Serial << "From last Update:\t"; Serial.println(deltat, 6);
  Serial << "GYRO:\tx:" << gx << "\t\ty:" << gy << "\t\tz:" << gz << newl;
  Serial << "ACC:\tx:" << ax << "\t\ty:" << ay << "\t\tz:" << az << newl;
  Serial << "MAG:\tx:" << mx << "\t\ty:" << my << "\t\tz:" << mz << newl;
  Serial << "TEMP:\t" << temp << newl << newl;
#endif

  deltat = fusion.deltatUpdate();
  //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //mahony is suggested if there isn't the mag
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick

  roll = fusion.getRoll();
  pitch = fusion.getPitch();
  yaw = fusion.getYaw();

#ifdef EULER_DATA
  Serial << "Pitch:\t" << pitch << "\t\tRoll:\t" << roll << "\t\tYaw:\t" << yaw << /*"\t\t" <<*/ "\t";
#endif

#ifdef PROCESSING
  roll = fusion.getRollRadians();
  pitch = fusion.getPitchRadians();
  yaw = fusion.getYawRadians();
  Serial << pitch << ":" << roll << ":" << yaw << "\n";
#endif

#ifdef SERIAL_PLOTER
  Serial << pitch << " " << roll << " " << yaw << endl;
#endif

  //delay(200); //for readability

}

void print_data() {
  // we can also read accelerometer / gyro data here!
  //Serial.println("Got new data!");
}
