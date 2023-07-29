/**
 * @brief   IoT Train with MaBeee firmware for Seeed XIAO nRF52840 BLE Sense
 * @file        iot_train.ino
 * @author  TANAHASHI, Jiro (aka jtFuruhata) <jt@do-johodai.ac.jp>
 * @version 0.0.1
 * @date    2023-03-06
 * 
 * @copyright   Copyright (c) 2023 jtLab, Hokkaido Information University,
 *              Release under the MIT License.
 *              See LICENSE.
 */

#include "iot_train.h"   // IoT Train definitions
#include "Preferences.h" // Preferences on onboard flash memory

// require LSM6DS3 Library by Seeed Studio
#include "LSM6DS3.h"
static LSM6DS3 imu(I2C_MODE, 0x6A);     // LSM6DS3(IMU) object

// load modules
#include "LED.h"    // LED singleton class definition (LED &led)

#define SERIAL_BAUDRATE 9600    // bps
#define SERIAL_TIMEOUT  5000    // ms

static STATE_PERIPHERAL statePeripheral = P_INIT;   // state machine for peripheral
static STATE_CENTRAL    stateCentral    = C_INIT;   // state machine for central
static STATE_COMMAND    stateCommand    = CMD_INIT; // state machine for command dispatcher

static commandPacket commandRequest;   // command packet from central
static commandPacket commandResponse;  // command packet to central

// XIAO BLE device properties and GATT profile:
static BLEDevice central;
String xiaoName;    // device name of XIAO
String xiaoAddress; // BLE address of XIAO
BLEService xiaoService(XIAO_XIAO_SERV_UUID);                     // GATT: XIAO service
BLECharacteristic commandCharacteristic(XIAO_COMMAND_CHAR_UUID, XIAO_COMMAND_CHAR_PROP, XIAO_COMMAND_CHAR_LEN); // GATT: command characteristic
BLECharacteristic accelCharacteristic(XIAO_ACCEL_CHAR_UUID, XIAO_ACCEL_CHAR_PROP, XIAO_ACCEL_CHAR_LEN);         // GATT: accelerometer characteristic
BLECharacteristic gyroCharacteristic(XIAO_GYRO_CHAR_UUID, XIAO_GYRO_CHAR_PROP, XIAO_GYRO_CHAR_LEN);             // GATT: gyroscope characteristic
BLECharacteristic tempCharacteristic(XIAO_TEMP_CHAR_UUID, XIAO_TEMP_CHAR_PROP, XIAO_TEMP_CHAR_LEN);             // GATT: temperature characteristic
BLECharacteristic ledCharacteristic(XIAO_LED_CHAR_UUID, XIAO_LED_CHAR_PROP, XIAO_LED_CHAR_LEN);                 // GATT: LED characteristic
BLECharacteristic pwmCharacteristic(XIAO_PWM_CHAR_UUID, XIAO_PWM_CHAR_PROP, XIAO_PWM_CHAR_LEN);                 // GATT: PWM characteristic
BLECharacteristic voltCharacteristic(XIAO_VOLT_CHAR_UUID, XIAO_VOLT_CHAR_PROP, XIAO_VOLT_CHAR_LEN);             // GATT: voltage characteristic

// handler for writable characteristics
void onCommandWritten(BLEDevice, BLECharacteristic);
void onLedWritten(BLEDevice, BLECharacteristic);
void onPwmWritten(BLEDevice, BLECharacteristic);

// handler for notifiable characteristics
void updateAccel();
void updateGyro();
void updateTemp();
void updateVolt();

// MaBeee BLE device properties and GATT profile:
BLEDevice mabeee;   // MaBeee BLE device
String mabeeeName;  // device name of MaBeee
String mabeeeAddress;   // BLE address of MaBeee
BLEService mabeeeCtrlService;           // GATT: control service
BLECharacteristic mabeeePwmCharacteristic;    // GATT: PWM characteristic
BLECharacteristic mabeeeVoltCharacteristic;   // GATT: voltage characteristic
BLEService mabeeeDataService;            // GATT: data service
BLECharacteristic mabeeeNameCharacteristic;   // GATT: name characteristic   
BLECharacteristic mabeeeIdCharacteristic;     // GATT: ID characteristic
BLECharacteristic mabeeeVersionCharacteristic;// GATT: version characteristic

void setup() {
    // start serial port
    led.setColor(LED::WHITE);
    Serial.begin(SERIAL_BAUDRATE, SERIAL_8N1);
    while (!Serial && millis() < SERIAL_TIMEOUT);
    led.setColor(LED::BLACK);
    Serial.print("ETrobocon IoT Train controller on ");
    Serial.println(BOARD_NAME);

    // start IMU
    if (imu.begin()) {
        Serial.println("starting IMU failed!");
        led.setR();
        while(1);
    } else {
        led.flashG(100,3);
        Serial.println("IMU health: OK");
    }

    //start LittleFS
    LittleFS = new FileSystem_MBED();
    if (!LittleFS->init()) {
        Serial.println("LittleFS failed!");
        while (1) {
            led.flashR();
            delay(800);
        }
    } else {
        readSettings();
        led.setG();
        Serial.println("LittleFS health: OK");
    }

    // start BLE Peripheral
    if (!BLE.begin()) {
        Serial.println("starting BLE failed!");
        while (1) {
            led.flashR(100, 2);
            delay(600);
        }
    }
    xiaoAddress = BLE.address();
    xiaoAddress.toLowerCase();
    xiaoName = "XIAO" + xiaoAddress.substring(9,11)
                      + xiaoAddress.substring(12,14)
                      + xiaoAddress.substring(15,17);
    xiaoName.toUpperCase();        
    led.setB();
    Serial.print("BLE health: OK - ");
    Serial.println(xiaoName);

    // prepare advertisement
    BLE.setLocalName(xiaoName.c_str());
    BLE.setDeviceName(xiaoName.c_str());
    commandCharacteristic.setEventHandler(BLEWritten, onCommandWritten);
    ledCharacteristic.setEventHandler(BLEWritten, onLedWritten);
    pwmCharacteristic.setEventHandler(BLEWritten, onPwmWritten);
    xiaoService.addCharacteristic(commandCharacteristic);
    xiaoService.addCharacteristic(accelCharacteristic);
    xiaoService.addCharacteristic(gyroCharacteristic);
    xiaoService.addCharacteristic(tempCharacteristic);
    xiaoService.addCharacteristic(ledCharacteristic);
    xiaoService.addCharacteristic(pwmCharacteristic);
    xiaoService.addCharacteristic(voltCharacteristic);
    BLE.addService(xiaoService);

    // update values
    updateAccel();
    updateGyro();
    updateTemp();
    updateLed();
    updatePwm();
    updateVolt();

    // device ready
    led.resetB();
    statePeripheral = P_ADVERTISING;
}

void doPeripheral() {
    static bool entry = true;
    static byte counter = 50;
    switch (statePeripheral) {
      case P_INIT: {
        setup();
        statePeripheral = P_ADVERTISING;
        entry = true;
      }  break;
      
      case P_ADVERTISING: {
        if (entry) {
            BLE.setAdvertisedService(xiaoService);
            BLE.advertise();
            Serial.println("Peripheral: Now advertising...");
            entry = false;
        }

//        if (host.connect()) {
        if (central = BLE.central()) {
            Serial.print("Peripheral: Connected to ");
//            Serial.println(host.getAddress());
            Serial.println(central.address());
            BLE.stopAdvertise();
            statePeripheral = P_CONNECTED;
            entry = true;
//        } else if (isAllowedHost(getHostAddress()))  {
//            Serial.print("Peripheral: Disconnected from ");
//            Serial.print(getHostAddress());
//            Serial.println(" because it is not allowed.");
        } else {
            counter--;
            if (!counter) {
                counter = 50;
                led.flipG();
            }
        }
      } break;

      case P_CONNECTED: {
//        if (host.isConnected()) {
        if (central.connected()) {
            led.setG();
        } else {
            Serial.println("Peripheral: Disconnected.");
            statePeripheral = P_ADVERTISING;
            entry = true;
        }
      } break;
    }
}

void doCentral() {
    static bool entry = true;
    static byte counter = 50;
    switch (stateCentral) {
      case C_INIT: {
        stateCentral = C_SCANNING;
        entry = true;
      }  break;

      case C_SCANNING: {
        String result = "";
        if (entry) {
            BLE.scan();
            Serial.println("Central: Now scanning...");
            entry = false;
        }
        if (mabeee = BLE.available()) {
            if (mabeee.localName().startsWith("MaBeee")) {
                result = mabeee.localName();
            }
        }
        if (result.length()) {
            Serial.print("Central: Found ");
            Serial.print(result);
            if (isPairedMaBeee(result)
              || connectFirstMaBeee) {
                Serial.println(" which is the target device.");
                BLE.stopScan();
                stateCentral = C_CONNECTING;
                entry = true;
            } else {
                Serial.println(".");
            }
        } else {
            counter--;
            if (!counter) {
                counter = 50;
                led.flipB();
            }
        }
      } break;

      case C_CONNECTING: {
        if (entry) {
            Serial.println("Central: Connecting...");
            if (mabeee) {
                setPairedMaBeeeName(mabeee.localName());
                connectFirstMaBeee = false;
                mabeee.connect();
            }
            counter = 25;
            entry = false;
        }
        if (mabeee.connected()) {
            Serial.println("Central: Connected.");
            stateCentral = C_CONNECTED;
            entry = true;
        } else {
            counter--;
            if (!counter) {
                counter = 25;
                led.flipB();
            }
        }
      } break;

      case C_CONNECTED: {
        bool result = true;
        if (entry) {
            entry = false;
            // discover attributes
            if (!mabeee.discoverAttributes()) {
                Serial.println("Central: Attribute discovery failed!");
                result = false;
            }
            // get services
            mabeeeCtrlService = mabeee.service(MABEEE_CTRL_SERV_UUID);
            mabeeeDataService = mabeee.service(MABEEE_DATA_SERV_UUID);
            if (!mabeeeCtrlService || !mabeeeDataService) {
                Serial.println("Central: Service discovery failed!");
                result = false;
            }
            // get characteristics
            mabeeePwmCharacteristic = mabeeeCtrlService.characteristic(MABEEE_PWM_CHAR_UUID);
            mabeeeVoltCharacteristic = mabeeeCtrlService.characteristic(MABEEE_VOLT_CHAR_UUID);
            mabeeeNameCharacteristic = mabeeeDataService.characteristic(MABEEE_NAME_CHAR_UUID);
            mabeeeIdCharacteristic = mabeeeDataService.characteristic(MABEEE_ID_CHAR_UUID);
            mabeeeVersionCharacteristic = mabeeeDataService.characteristic(MABEEE_VER_CHAR_UUID);
            if (!mabeeePwmCharacteristic
            || !mabeeeVoltCharacteristic
            || !mabeeeNameCharacteristic
            || !mabeeeIdCharacteristic
            || !mabeeeVersionCharacteristic) {
                Serial.println("Central: Characteristic discovery failed!");
                result = false;
            }
            if (!result) {
                mabeee.disconnect();
                Serial.println("Central: Failed to prepare services.");
                stateCentral = C_SCANNING;
                entry = true;
            }
        }
        if (mabeee.connect()) {
            led.setB();
        } else {
            Serial.println("Central: Disconnected.");
            stateCentral = C_SCANNING;
            entry = true;
        }
      } break;
    }
}

void doNotify(){
    static uint16_t counter[] = {0, 0, 0, 0};
    for (int idx=0; idx<4; idx++) {
        if (counter[idx] >= 10) {
        counter[idx] -= 10;
        }
        if (counter[idx] <= 0) {
            switch (idx) {
              case 0: {
                updateAccel();
                counter[0] = getAccelPeriod();
              } break;
              case 1: {
                updateGyro();
                counter[1] = getGyroPeriod();
              } break;
              case 2: {
                updateTemp();
                counter[2] = getTempPeriod();
              } break;
              case 3: {
                updateVolt();
                counter[3] = getVoltPeriod();
              } break;
            }
        }
    }
}

void doCommand(){
    static bool entry = true;
    static byte counter = 50;
    switch (stateCommand) {
      case CMD_INIT: {
        stateCommand = CMD_WAITING;
        entry = true;
      }  break;

      case CMD_WAITING: {
        if (entry) {
            led.resetR();
            entry = false;
        }
        // state will change by command dispacher
      } break;

      case CMD_EXECUTING: {
        if (entry) {
            Serial.print("Command: start executing ");
            Serial.println(commandResponse.command);
            entry = false;
        }
        if (true) {
            Serial.println("Command: Processed.");
            stateCommand = CMD_WAITING;
            entry = true;
        } else {
            counter--;
            if (!counter) {
                counter = 50;
                led.flipR();
            }
        }
      } break;
    }
}

void loop() {
    static unsigned long beginTime = 0;
    static unsigned long elapsedTime = 0;
    beginTime = micros();

    doNotify();
    doCommand();
    doPeripheral();
    doCentral();

    // each loop runs about every 10ms
    elapsedTime = micros() - beginTime;
    delayMicroseconds(10000 - (elapsedTime % 10000));
}

void updateAccel() {
    float3Packet accel;
    accel.timestamp = millis();
    accel.float1 = imu.readFloatAccelX();
    accel.float2 = imu.readFloatAccelY();
    accel.float3 = imu.readFloatAccelZ();
    accelCharacteristic.writeValue(accel.byteArray, sizeof(accel.byteArray));
}

void updateGyro() {
    float3Packet gyro;
    gyro.timestamp = millis();
    gyro.float1 = imu.readFloatGyroX();
    gyro.float2 = imu.readFloatGyroY();
    gyro.float3 = imu.readFloatGyroZ();
    gyroCharacteristic.writeValue(gyro.byteArray, sizeof(gyro.byteArray));
}

void updateTemp() {
    float1Packet temp;
    temp.timestamp = millis();
    temp.float1 = imu.readTempC();
    tempCharacteristic.writeValue(temp.byteArray, sizeof(temp.byteArray));//
}

void updateLed() {
    unsigned char color = (unsigned char)led.getColor();
    ledCharacteristic.writeValue(color);
}

void updatePwm() {
    // ToDo: move to MaBeee class?
    unsigned char pwm = 0;
    pwmCharacteristic.writeValue(pwm);
}

void updateVolt() {
    // ToDo: move to MaBeee class?
    float1Packet volt;
    volt.timestamp = millis();
    volt.float1 = 0.0F;
    voltCharacteristic.writeValue(volt.byteArray, sizeof(volt.byteArray));
}

void setAccelPeriod(uint16_t period) {
    accelPeriod = period;
}

void setGyroPeriod(uint16_t period) {
    gyroPeriod = period;
}

void setTempPeriod(uint16_t period) {
    tempPeriod = period;
}

void setVoltPeriod(uint16_t period) {
    voltPeriod = period;
}

uint16_t getAccelPeriod() {
    return accelPeriod;
}

uint16_t getGyroPeriod() {
    return gyroPeriod;    
}

uint16_t getTempPeriod() {
    return tempPeriod;
}

uint16_t getVoltPeriod() {
    return voltPeriod;
}

/**
 * @brief command dispacher
 * 
 * @param central BLEDevice set by callback
 * @param characteristic BLECharacteristic set by callback
 */
void onCommandWritten(BLEDevice central, BLECharacteristic characteristic) {
    Serial.println("Debug: onCommandWritten");
    // prepare packets
    memcpy(commandRequest.byteArray, characteristic.value(), sizeof(commandRequest.byteArray));
    commandRequest.payload[16] = '\0';

    Serial.print("Command: Received -> ");
    Serial.print(commandRequest.commandID);
    Serial.print(":");
    Serial.println(commandRequest.command);

    if (stateCommand == CMD_WAITING) {
        memcpy(commandResponse.byteArray, commandRequest.byteArray, sizeof(commandResponse.byteArray));
        commandResponse.response = CMD_RESP_ILLEG_CMD;
        
        switch (commandRequest.command) {
          case CMD_NOP: {
            commandResponse.response = CMD_RESP_OK;
          } break;

          case CMD_TRAIN_STOP:
          case CMD_TRAIN_FORWARD:
          case CMD_TRAIN_SET_PWM:
          case CMD_TRAIN_GET_PWM: {
            commandResponse.response = CMD_RESP_EXECUTING;
          } break;

          case CMD_TRAIN_SET_LED: {
            led.setColor((LED::COLOR)commandRequest.payload[0]);
            commandResponse.response = CMD_RESP_OK;
          }

          case CMD_TRAIN_GET_LED: {
            commandResponse.payload[0] = (unsigned char)led.getColor();
            commandResponse.response = CMD_RESP_OK;
          } break;

          case CMD_MABEEE_GET_NAME:
          case CMD_MABEEE_GET_ID:
          case CMD_MABEEE_GET_VER:
          case CMD_MABEEE_GET_BDADDR:
          case CMD_MABEEE_GET_RSSI: {
            commandResponse.response = CMD_RESP_EXECUTING;
          } break;

          case CMD_XIAO_GET_STATE: {
            commandResponse.payload[0] = (unsigned char)stateCentral;
            commandResponse.response = CMD_RESP_OK;
          } break;
          
          case CMD_XIAO_SET_MABEEE: {
            connectFirstMaBeee = false;
            if (setPairedMaBeeeName(String((char *)commandRequest.payload))) {
                commandResponse.response = CMD_RESP_OK;
            } else {
                Serial.println("Command: error: failed to write file.");
                commandResponse.response = CMD_RESP_ERROR;
            }
          }

          case CMD_XIAO_GET_MABEEE: {
            memcpy(commandResponse.payload, pairedMaBeeeName.c_str(), pairedMaBeeeName.length());
            commandResponse.response = CMD_RESP_OK;
          } break;

          case CMD_XIAO_SCAN_MABEEE:
          case CMD_XIAO_CONNECT_AUTO: {
            commandResponse.response = CMD_RESP_EXECUTING;
          } break;
        }
    } else if (commandRequest.command == CMD_CANCEL) {
        Serial.println("Command: Cancel requested.");
        commandRequest.response = CMD_RESP_EXECUTING;
    } else {
        Serial.println("Command: error: command dispatcher is busy.");
        commandResponse.response = CMD_RESP_BUSY;
    }
}

void onLedWritten(BLEDevice central, BLECharacteristic characteristic) {
    Serial.println("Debug: onLedWritten");
    led.setColor((LED::COLOR)characteristic.value()[0]);
}

void onPwmWritten(BLEDevice central, BLECharacteristic characteristic) {
    Serial.println("Debug: onPwmWritten");
    pwmPacket pwm;
    memcpy(pwm.byteArray, characteristic.value(), sizeof(pwm.byteArray));
}