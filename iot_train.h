/**
 * @brief   BLE definitions of IoT Train with MaBeee using from both central and peripheral
 * @file        iot_train.h
 * @author  TANAHASHI, Jiro (aka jtFuruhata) <jt@do-johodai.ac.jp>
 * @version 1.0.1
 * @date    2023-03-18
 * 
 * @copyright   Copyright (c) 2023 jtLab, Hokkaido Information University,
 *              Release under the MIT License.
 *              See LICENSE.
 */

#pragma once
#ifndef _IOT_TRAIN_H_
#define _IOT_TRAIN_H_

#include <Arduino.h>     // Arduino core
#include "ArduinoBLE.h"  // BLE library

// convert values to byte array
typedef union {
    struct {
        uint32_t  timestamp;
        float float1;
    };
    unsigned char byteArray[8];
} float1Packet;

typedef union {
    struct {
        uint32_t  timestamp;
        float float1;
        float float2;
        float float3;
    };
    unsigned char byteArray[16];
} float3Packet;

typedef union {
    struct {
        uint32_t  commandID;
        unsigned char command;
        unsigned char response;
        unsigned char payload[17];
    };
    unsigned char byteArray[23];
} commandPacket;

typedef union {
    struct {
        unsigned char command;
        uint32_t  pwm;
    };
    unsigned char byteArray[5];
} pwmPacket;

typedef union {
    struct {
        uint16_t major;
        uint16_t minor;
    };
    unsigned char byteArray[4];
} versionPacket;

// BLE peripheral GATT profile: XIAO side
#define XIAO_XIAO_SERV_UUID "AD0C1000-64E9-48B0-9088-6F9E9FE4972E"      // UUID:XIAO service
#define XIAO_XIAO_SERV_TYPE BLEService                                  // type:XIAO service
#define XIAO_COMMAND_CHAR_UUID "AD0C1001-64E9-48B0-9088-6F9E9FE4972E"   // UUID:command characteristic
#define XIAO_COMMAND_CHAR_PROP (BLEWrite|BLENotify)                     // property:command characteristic, writable/notifiable
#define XIAO_COMMAND_CHAR_TYPE BLECharacteristic                        // type:command characteristic, byte array
#define XIAO_COMMAND_CHAR_LEN 21                                        // length:command characteristic, 21 bytes
#define XIAO_ACCEL_CHAR_UUID "AD0C1002-64E9-48B0-9088-6F9E9FE4972E"     // UUID:accelerometer characteristic
#define XIAO_ACCEL_CHAR_PROP (BLERead|BLENotify)                        // property:accelerometer characteristic, readable/notifiable
#define XIAO_ACCEL_CHAR_TYPE BLECharacteristic                          // type:accelerometer characteristic, byte array
#define XIAO_ACCEL_CHAR_LEN 16                                          // length:accelerometer characteristic, 16 bytes
#define XIAO_GYRO_CHAR_UUID "AD0C1003-64E9-48B0-9088-6F9E9FE4972E"      // UUID:gyroscope characteristic
#define XIAO_GYRO_CHAR_PROP (BLERead|BLENotify)                         // property:gyroscope characteristic, readable/notifiable
#define XIAO_GYRO_CHAR_TYPE BLECharacteristic                           // type:gyroscope characteristic, byte array
#define XIAO_GYRO_CHAR_LEN 16                                           // length:gyroscope characteristic, 16 bytes
#define XIAO_TEMP_CHAR_UUID "AD0C1004-64E9-48B0-9088-6F9E9FE4972E"      // UUID:temperature characteristic
#define XIAO_TEMP_CHAR_PROP (BLERead|BLENotify)                         // property:temperature characteristic, readable/notifiable
#define XIAO_TEMP_CHAR_TYPE BLECharacteristic                           // type:temperature characteristic, byte array
#define XIAO_TEMP_CHAR_LEN 8                                            // length:temperature characteristic, 8 bytes
#define XIAO_LED_CHAR_UUID "AD0C1005-64E9-48B0-9088-6F9E9FE4972E"       // UUID:LED characteristic
#define XIAO_LED_CHAR_PROP (BLERead|BLEWrite)                           // property:LED characteristic, readable/writable
#define XIAO_LED_CHAR_TYPE BLEUnsignedCharCharacteristic                // type:LED characteristic, unsigned char
#define XIAO_LED_CHAR_LEN 1                                             // length:LED characteristic, 1 byte
#define XIAO_PWM_CHAR_UUID "AD0C2001-64E9-48B0-9088-6F9E9FE4972E"       // UUID:PWM characteristic
#define XIAO_PWM_CHAR_PROP (BLERead|BLEWrite)                           // property:PWM characteristic, readable/writable
#define XIAO_PWM_CHAR_TYPE BLEUnsignedCharCharacteristic                // type:PWM characteristic, unsigned char
#define XIAO_PWM_CHAR_LEN 1                                             // length:PWM characteristic, 1 byte
#define XIAO_VOLT_CHAR_UUID "AD0C2002-64E9-48B0-9088-6F9E9FE4972E"      // UUID:voltage characteristic 
#define XIAO_VOLT_CHAR_PROP (BLERead|BLENotify)                         // property:voltage characteristic, readable/notifiable
#define XIAO_VOLT_CHAR_TYPE BLECharacteristic                           // type:voltage characteristic, byte array
#define XIAO_VOLT_CHAR_LEN 8                                            // length:voltage characteristic, 8 bytes

// BLE peripheral GATT profile: MaBeee side
#define MABEEE_CTRL_SERV_UUID "B9F5FF00-D813-46C6-8B61-B453EE2C74D9"    // UUID:MaBeee control service
#define MABEEE_CTRL_SERV_TYPE BLEService                                // type:MaBeee control service
#define MABEEE_1000_CHAR_UUID "B9F51000-D813-46C6-8B61-B453EE2C74D9"    // UUID:unknown 1000 characteristic
#define MABEEE_1000_CHAR_PROP (BLERead|BLEWrite)                        // property:unknown 1000 characteristic, readable/writable
#define MABEEE_1000_CHAR_TYPE BLELongCharacteristic                     // type:unknown 1000 characteristic, unsigned int 32
#define MABEEE_1000_CHAR_LEN 4                                          // length:unknown 1000 characteristic, 4 bytes
#define MABEEE_VOLT_CHAR_UUID "B9F51001-D813-46C6-8B61-B453EE2C74D9"    // UUID:voltage characteristic
#define MABEEE_VOLT_CHAR_PROP (BLEWrite|BLENotify)                      // property:voltage characteristic, writable/notifiable
#define MABEEE_VOLT_CHAR_TYPE BLEUnsignedCharCharacteristic             // type:voltage characteristic, unsigned char
#define MABEEE_VOLT_CHAR_LEN 1                                          // length:voltage characteristic, 1 byte
#define MABEEE_1002_CHAR_UUID "B9F51002-D813-46C6-8B61-B453EE2C74D9"    // UUID:unknown 1002 characteristic
#define MABEEE_1002_CHAR_PROP (BLENotify)                               // property:unknown 1002 characteristic, notifiable
#define MABEEE_1002_CHAR_TYPE BLEUnsignedCharCharacteristic             // type:unknown 1002 characteristic, unsigned char
#define MABEEE_1002_CHAR_LEN 1                                          // length:unknown 1002 characteristic, 1 byte
#define MABEEE_3005_CHAR_UUID "B9F53005-D813-46C6-8B61-B453EE2C74D9"    // UUID:unknown 3005 characteristic
#define MABEEE_3005_CHAR_PROP (BLERead|BLEWrite)                        // property:unknown 3005 characteristic, readable/writable
#define MABEEE_3005_CHAR_TYPE BLECharacteristic                         // type:unknown 3005 characteristic, byte array
#define MABEEE_3005_CHAR_LEN 5                                          // length:unknown 3005 characteristic, 5 bytes
#define MABEEE_PWM_CHAR_UUID "B9F53006-D813-46C6-8B61-B453EE2C74D9"     // UUID:PWM characteristic
#define MABEEE_PWM_CHAR_PROP (BLERead|BLEWrite)                         // property:PWM characteristic, readable/writable
#define MABEEE_PWM_CHAR_TYPE BLECharacteristic                          // type:PWM characteristic, byte array
#define MABEEE_PWM_CHAR_LEN 5                                           // length:PWM characteristic, 5 bytes
#define MABEEE_DATA_SERV_UUID "B9F54000-D813-46C6-8B61-B453EE2C74D9"    // UUID:MaBeee data service
#define MABEEE_DATA_SERV_TYPE BLEService                                // type:MaBeee data service
#define MABEEE_NAME_CHAR_UUID "B9F54001-D813-46C6-8B61-B453EE2C74D9"    // UUID:device name characteristic
#define MABEEE_NAME_CHAR_PROP (BLERead|BLEWrite)                        // property:device name characteristic, readable/writable
#define MABEEE_NAME_CHAR_TYPE BLECharacteristic                         // type:device name characteristic, byte array(string)
#define MABEEE_NAME_CHAR_LEN 12                                         // length:device name characteristic, 12 bytes
#define MABEEE_ID_CHAR_UUID "B9F54002-D813-46C6-8B61-B453EE2C74D9"      // UUID:device ID characteristic
#define MABEEE_ID_CHAR_PROP (BLERead)                                   // property:device ID characteristic, readable
#define MABEEE_ID_CHAR_TYPE BLECharacteristic                           // type:device ID characteristic, byte array(uusigned int 64)
#define MABEEE_ID_CHAR_LEN 8                                            // length:device ID characteristic, 8 bytes
#define MABEEE_VER_CHAR_UUID "B9F54003-D813-46C6-8B61-B453EE2C74D9"     // UUID:firmware version characteristic
#define MABEEE_VER_CHAR_PROP (BLERead)                                  // property:firmware version characteristic, readable
#define MABEEE_VER_CHAR_TYPE BLECharacteristic                          // type:firmware version characteristic, byte array(2 * unsigned int 16)
#define MABEEE_VER_CHAR_LEN 4                                           // length:firmware version characteristic, 4 bytes
#define MABEEE_4004_CHAR_UUID "B9F54004-D813-46C6-8B61-B453EE2C74D9"    // UUID:unknown 4004 characteristic
#define MABEEE_4004_CHAR_PROP (BLEWrite)                                // property:unknown 4004 characteristic, writable
#define MABEEE_4004_CHAR_TYPE BLECharacteristic                         // type:unknown 4004 characteristic, byte array
#define MABEEE_4004_CHAR_LEN 1                                          // length:unknown 4004 characteristic, 1 byte

// control commands from host as central to XIAO as peripheral
#define CMD_NOP                 0 // no operation, use like pinging. payload = none, return = none
#define CMD_CANCEL            255 // cancel operation. payload = none, return = none
#define CMD_TRAIN_STOP          1 // stop train. payload = none, return = none
#define CMD_TRAIN_FORWARD       2 // move train forward by duty 100%. payload = none, return = none
#define CMD_TRAIN_SET_PWM       3 // move train forward by duty 0-100%. payload = duty:unsigned char, return = none
#define CMD_TRAIN_GET_PWM       4 // get current duty. payload = none, return = duty:unsigned char
#define CMD_TRAIN_SET_LED       5 // set LED color. payload = color:unsigned char(LED::COLOR), return = none
#define CMD_TRAIN_GET_LED       6 // get LED color. payload = none, return = color:unsigned char(LED::COLOR)
#define CMD_MABEEE_GET_NAME    11 // get device name. payload = none, return = name:unsigned char(string)
#define CMD_MABEEE_GET_ID      12 // get device ID. payload = none, return = ID:unsigned int 64
#define CMD_MABEEE_GET_VER     13 // get firmware version. payload = none, return = version:versionPacket
#define CMD_MABEEE_GET_BDADDR  14 // get MaBeee's bluetooth address. payload = none, return = address:unsigned char(string)
#define CMD_MABEEE_GET_RSSI    15 // get bluetooth RSSI from MaBeee to XIAO. payload = none, return = RSSI:signed char
#define CMD_XIAO_GET_STATE     21 // get connection status with MaBeee. payload = none, return = state:unsigned char(STATE_CENTRAL)
#define CMD_XIAO_SET_MABEEE    22 // set device name of MaBeee to connect. payload = name:unsigned char(string), return = none
#define CMD_XIAO_GET_MABEEE    23 // get device name of MaBeee to connect. payload = none, return = name:unsigned char(string)
#define CMD_XIAO_SCAN_MABEEE   24 // scan until the first MaBeee is found. payload = none, return = name:unsigned char(string)
#define CMD_XIAO_CONNECT_AUTO  25 // connect to the first MaBeee found. payload = none, return = none
#define CMD_XIAO_ALLOW_HOST    26 // register currently connected host (central) as allowed one. payload = none, return = none
#define CMD_SET_PERIOD_ACCEL   31 // set notification period of accelerometer. payload = period:unsigned int 16(0=do not notify), return = none
#define CMD_GET_PERIOD_ACCEL   32 // get notification period of accelerometer. payload = none, return = period:unsigned int 16
#define CMD_SET_PERIOD_GYRO    33 // set notification period of gyroscope. payload = period:unsigned int 16(0=do not notify), return = none
#define CMD_GET_PERIOD_GYRO    34 // get notification period of gyroscope. payload = none, return = period:unsigned int 16
#define CMD_SET_PERIOD_TEMP    35 // set notification period of temperature. payload = period:unsigned int 16(0=do not notify), return = none
#define CMD_GET_PERIOD_TEMP    36 // get notification period of temperature. payload = none, return = period:unsigned int 16
#define CMD_SET_PERIOD_VOLT    37 // set notification period of voltage. payload = period:unsigned int 16(0=do not notify), return = none
#define CMD_GET_PERIOD_VOLT    38 // get notification period of voltage. payload = none, return = period:unsigned int 16

// command response from XIAO as peripheral to host as central
#define CMD_RESP_OK           200 // command executed successfully
#define CMD_RESP_EXECUTING    201 // command is executing
#define CMD_RESP_ERROR        100 // command executed with unknown error
#define CMD_RESP_BUSY         101 // error: command dispatcher is busy
#define CMD_RESP_ILLEG_CMD    102 // error: illegal command
#define CMD_RESP_ILLEG_PARAM  103 // error: illegal payload parameter
#define CMD_RESP_CONN_DENIED  111 // error: connection denied; host address not allowed
#define CMD_RESP_NO_MABEEE    121 // error: MaBeee not found
#define CMD_RESP_DISC_MABEEE  122 // error: MaBeee disconnected

// state machine as peripheral
#define STATE_PERIPHERAL_INIT           0 // initial state
#define STATE_PERIPHERAL_ADVERTISING    3 // advertising state
#define STATE_PERIPHERAL_CONNECTED      4 // connected state

typedef enum {
    P_INIT = STATE_PERIPHERAL_INIT,
    P_ADVERTISING = STATE_PERIPHERAL_ADVERTISING,
    P_CONNECTED = STATE_PERIPHERAL_CONNECTED
} STATE_PERIPHERAL;

// state machine as central
#define STATE_CENTRAL_INIT          0 // initial state
#define STATE_CENTRAL_SCANNING      1 // scanning state
#define STATE_CENTRAL_CONNECTING    2 // connecting state
#define STATE_CENTRAL_CONNECTED     4 // connected state
#define STATE_CENTRAL_DISCONNECTED  5 // disconnected state

typedef enum {
    C_INIT = STATE_CENTRAL_INIT,
    C_SCANNING = STATE_CENTRAL_SCANNING,
    C_CONNECTING = STATE_CENTRAL_CONNECTING,
    C_CONNECTED = STATE_CENTRAL_CONNECTED,
    C_DISCONNECTED = STATE_CENTRAL_DISCONNECTED
} STATE_CENTRAL;

// state machine as command dispatcher
#define STATE_CMD_INIT          0 // initial state
#define STATE_CMD_WAITING       6 // waiting for command
#define STATE_CMD_EXECUTING     7 // executing command

typedef enum {
    CMD_INIT = STATE_CMD_INIT,
    CMD_WAITING = STATE_CMD_WAITING,
    CMD_EXECUTING = STATE_CMD_EXECUTING
} STATE_COMMAND;

/**
 * @brief update acceleration value
 */
void updateAccel();

/**
 * @brief update gyroscope value
 */
void updateGyro();

/**
 * @brief update temperature value
 */
void updateTemp();

/**
 * @brief update LED value
 */
void updateLed();

/**
 * @brief update PWM value
 */
void updatePwm();

/**
 * @brief update voltage value
 */
void updateVolt();

/**
 * @brief Set the Accel Period
 * 
 * @param period of notification (ms)
 */
void setAccelPeriod(uint16_t period);

/**
 * @brief Set the Gyro Period
 * 
 * @param period of notification (ms)
 */
void setGyroPeriod(uint16_t period);

/**
 * @brief Set the Temp Period
 * 
 * @param period of notification (ms)
 */
void setTempPeriod(uint16_t period);

/**
 * @brief Set the Voltage Period
 * 
 * @param period of notification (ms)
 */
void setVoltPeriod(uint16_t period);

/**
 * @brief Get the Accel Period
 * 
 * @return period of notification (ms)
 */
uint16_t getAccelPeriod();

/**
 * @brief Get the Gyro Period
 * 
 * @return period of notification (ms)
 */
uint16_t getGyroPeriod();

/**
 * @brief Get the Temp Period
 * 
 * @return period of notification (ms)
 */
uint16_t getTempPeriod();

/**
 * @brief Get the Voltage Period
 * 
 * @return period of notification (ms)
 */
uint16_t getVoltPeriod();


#endif // _IOT_TRAIN_H_
