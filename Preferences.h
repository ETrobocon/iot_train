/**
 * @brief   IoT Train preferences on onboard flash memory for Seeed XIAO nRF52840 BLE (Sense)
 * @file        Preferences.h
 * @author  TANAHASHI, Jiro (aka jtFuruhata) <jt@do-johodai.ac.jp>
 * @version 0.0.1
 * @date    2023-03-21
 * 
 * @copyright   Copyright (c) 2023 jtLab, Hokkaido Information University,
 *              Release under the MIT License.
 *              See LICENSE.
 */

#pragma once
#ifndef _PREFERENCES_H_
#define _PREFERENCES_H_

#include "iot_train.h"      // IoT Train definitions

// require nRF52 LittleFS Library by Khoi Hoaung
#define _FS_LOGLEVEL_               1
#define NANO33BLE_FS_SIZE_KB        256
#define FORCE_REFORMAT              false
#include "FS_Nano33BLE.h"

// file sysytem on onboard flash memory
static FileSystem_MBED *LittleFS;      // LittleFS object

// files of persistent paramater written on LittleFS
const char fileMaBeee[] = MBED_FS_FILE_PREFIX "/mabeee.txt";
const char fileHost[] = MBED_FS_FILE_PREFIX "/host.txt";

static String pairedMaBeeeName = "";    // device name of the paired MaBeee (MaBeeeA00000)
static String allowedHostAddress = "";  // BLE address of the allowed host (00:00:00:00:00:00)
static bool connectFirstMaBeee = true;  // true: connect first MaBeee automatically
static uint16_t accelPeriod = 50;       // period of accelerometer notification (ms)
static uint16_t gyroPeriod = 50;        // period of gyroscope notification (ms)
static uint16_t tempPeriod = 50;        // period of temperature notification (ms)
static uint16_t voltPeriod = 50;        // period of voltage notification (ms)

/**
 * @brief  Read string from LittleFS
 * 
 * @param filename file name
 * @return file contents (max length: 32), empty string if error
 */
String readFile(const char *filename);

/**
 * @brief Write string to LittleFS
 * 
 * @param filename file name
 * @param data string to write (max length: 32)
 * @return >0 length sent if success, -1 if error
 */
int writeFile(const char *filename, String data);

/**
 * @brief the paired MaBeee is specified
 * 
 * @return true if the paired MaBeee is specified in onboard flash memory
 */
bool hasPairedMaBeee();

/**
 * @brief check if the specified name is the paired MaBeee
 * 
 * @param name device name
 * @return true if the specified name is the paired MaBeee
 */
bool isPairedMaBeee(String name);

/**
 * @brief Get the paired MaBeee name
 * 
 * @return the paired MaBeee name 
 */
String getPairedMaBeeeName();

/**
 * @brief Set the paired MaBeee name
 * 
 * @param name the paired MaBeee name
 * @return true if the paired Mabeee name is set successfully
 */
bool setPairedMaBeeeName(String name);

/**
 * @brief the allowed host is specified
 * 
 * @return true if the allowed host is specified in onboard flash memory
 */
bool hasAllowedHost();

/**
 * @brief check if the specified address is the allowed host
 * 
 * @param address BLE address
 * @return true if the specified address is the allowed host
 */
bool isAllowedHost(String address);

/**
 * @brief Get the allowed host address
 * 
 * @return the allowed host address
 */
String getAllowedHostAddress();

/**
 * @brief Set the allowed host address
 * 
 * @param address the allowed host address
 * @return true if the allowed host address is set successfully
 */
bool setAllowedHostAddress(String address);

/**
 * @brief read settings from files on flash memory
 */
void readSettings();

#endif // _PREFERENCES_H_
