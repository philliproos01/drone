
#include <RadioLib.h>
#include "Arduino_BMI270_BMM150.h"

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
// SX1278 has the following connections:
// NSS pin:   10
// DIO0 pin:  2
// RESET pin: 9
// DIO1 pin:  3
//old default: SX1278 radio = new Module(10, 2, 9, 3);
SX1278 radio = new Module(4, 3, 2);
// or detect the pinout automatically using RadioBoards
// https://github.com/radiolib-org/RadioBoards
/*
#define RADIO_BOARD_AUTO
#include <RadioBoards.h>
Radio radio = new RadioModule();
*/
void setFlag(void);
void print_data();
// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;
MyBoschSensor myIMU(Wire1);


void setup() {


  Serial.begin(9600);
    //Serial.begin(115200);
  while (!Serial);
  myIMU.debug(Serial);
  myIMU.onInterrupt(print_data);
  myIMU.begin();

  Serial.print("Accelerometer sample rate = ");
  Serial.println(myIMU.accelerationSampleRate());

  // initialize SX1278 with default settings
  Serial.print(F("[SX1278] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // set the function that will be called
  // when packet transmission is finished
  radio.setPacketSentAction(setFlag);

  // start transmitting the first packet
  Serial.print(F("[SX1278] Sending first packet ... "));

  // you can transmit C-string or Arduino string up to
  // 255 characters long
  transmissionState = radio.startTransmit("Hello World!");

  // you can also transmit byte array up to 255 bytes long
  /*
    byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                      0x89, 0xAB, 0xCD, 0xEF};
    transmissionState = radio.startTransmit(byteArr, 8);
  */
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

// counter to keep track of transmitted packets
int count = 0;

void loop() {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;

  if (myIMU.accelerationAvailable()) {
    myIMU.readAcceleration(accel_x, accel_y, accel_z);

    Serial.print("accel: \t");
    Serial.print(accel_x);
    Serial.print('\t');
    Serial.print(accel_y);
    Serial.print('\t');
    Serial.print(accel_z);
    Serial.println();
  }

  if (myIMU.gyroscopeAvailable()) {

    myIMU.readGyroscope(gyro_x, gyro_y, gyro_z);

    Serial.print("gyro: \t");
    Serial.print(gyro_x);
    Serial.print('\t');
    Serial.print(gyro_y);
    Serial.print('\t');
    Serial.print(gyro_z);
    Serial.println();
  }

  if (myIMU.magneticFieldAvailable()) {

    myIMU.readMagneticField(mag_x, mag_y, mag_z);

    Serial.print("mag: \t");
    Serial.print(mag_x);
    Serial.print('\t');
    Serial.print(mag_y);
    Serial.print('\t');
    Serial.print(mag_z);
    Serial.println();
  }
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

    // wait a second before transmitting again
    delay(1000);

    // send another one
    Serial.print(F("[SX1278] Sending another packet ... "));

    // you can transmit C-string or Arduino string up to
    // 255 characters long
    String str = "Hello World! #" + String(accel_x) + String(accel_y) + String(accel_z);
    
    
    //String(gyro_x) + String(gyro_y) + String(gyro_z)
    //String(mag_x) + String(mag_y) + String(mag_z)
   
    //String str = "Hello World! #" + String(count++);
    transmissionState = radio.startTransmit(str);

    // you can also transmit byte array up to 255 bytes long
    /*
      byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                        0x89, 0xAB, 0xCD, 0xEF};
      transmissionState = radio.startTransmit(byteArr, 8);
    */
  }

  
}

void print_data() {
  // we can also read accelerometer / gyro data here!
  //Serial.println("Got new data!");
}
