#include <Arduino.h>
#include <ESP32Servo.h>
#include <RadioLib.h>

Servo panServo;
Servo tiltServo;

const int panPin = 26;
const int tiltPin = 27;
SX1278 radio = new Module(33, 32, 25);
int panAngle;
int tiltAngle;


volatile bool receivedFlag = false;

void setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}

void setFlag(void);
float stringToFloat(const char* str, int start, int end);
int parseData(const char* input, float* output, int maxSize);


void setup() {
  Serial.begin(115200);
  

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);

  panServo.attach(panPin, 500, 2500);
  tiltServo.attach(tiltPin, 500, 2500);
  
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
  // when new packet is received
  radio.setPacketReceivedAction(setFlag);

  // start listening for LoRa packets
  Serial.print(F("[SX1278] Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }


}

void loop() {
  float dataArray[3];
  const char* dataString = "Data: 0.00 1.11 2.22 3.33 4.44 5.55 6.66 7.77 8.88";
  
  // check if the flag is set
  if(receivedFlag) {
    // reset flag
    receivedFlag = false;

    // you can read received data as an Arduino String
    String str;
    int state = radio.readData(str);

    // you can also read received data as byte array
    /*
      byte byteArr[8];
      int numBytes = radio.getPacketLength();
      int state = radio.readData(byteArr, numBytes);
    */

    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      Serial.println(F("[SX1278] Received packet!"));

      // print data of the packet
      Serial.print(F("[SX1278] Data:\t\t"));
      Serial.println(str);

      // print RSSI (Received Signal Strength Indicator)
      Serial.print(F("[SX1278] RSSI:\t\t"));
      Serial.print(radio.getRSSI());
      Serial.println(F(" dBm"));

      // print SNR (Signal-to-Noise Ratio)
      Serial.print(F("[SX1278] SNR:\t\t"));
      Serial.print(radio.getSNR());
      Serial.println(F(" dB"));

      // print frequency error
      Serial.print(F("[SX1278] Frequency error:\t"));
      Serial.print(radio.getFrequencyError());
      Serial.println(F(" Hz"));

      int dataCount = parseData(str.c_str(), dataArray, 3);
      Serial.print("Parsed ");
      Serial.print(dataCount);
      Serial.println(" values:");
      
    // for (int i = 0; i < dataCount; i++) {
      //    Serial.println(dataArray[i], 2);  // Print with 2 decimal places
    // }
      double yaw = dataArray[1];
      double pitch = dataArray[2];

      Serial.println(dataArray[1]);
      Serial.println(dataArray[2]);
      
      int panAngle = map(yaw, -180, 180, 0, 180);
      int tiltAngle = map(pitch, -90, 90, 0, 180);
      


      panServo.write(panAngle);
      tiltServo.write(tiltAngle);


    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("[SX1278] CRC error!"));

    } else {
      // some other error occurred
      Serial.print(F("[SX1278] Failed, code "));
      Serial.println(state);

    }
  
  }
  

  
}

float stringToFloat(const char* str, int start, int end) {
    float result = 0.0f;
    float fraction = 0.1f;
    bool decimal = false;
    bool negative = false;
    
    for (int i = start; i < end; i++) {
        if (str[i] == '-') {
            negative = true;
        } else if (str[i] == '.') {
            decimal = true;
        } else if (str[i] >= '0' && str[i] <= '9') {
            if (!decimal) {
                result = result * 10.0f + (str[i] - '0');
            } else {
                result += (str[i] - '0') * fraction;
                fraction *= 0.1f;
            }
        }
    }
    
    return negative ? -result : result;
}

// Function to parse the string and fill the float array
int parseData(const char* input, float* output, int maxSize) {
    int count = 0;
    int start = 0;
    bool dataStarted = false;
    
    for (int i = 0; input[i] != '\0' && count < maxSize; i++) {
        if (!dataStarted) {
            if (input[i] == ':') {
                dataStarted = true;
                start = i + 1;
            }
        } else {
            if (input[i] == ' ' || input[i + 1] == '\0') {
                int end = (input[i + 1] == '\0') ? i + 1 : i;
                output[count++] = stringToFloat(input, start, end);
                start = i + 1;
            }
        }
    }
    
    return count;
}
