/**
 * @brief   IoT Train preferences on onboard flash memory for Seeed XIAO nRF52840 BLE (Sense)
 * @file        Preferences.cpp
 * @author  TANAHASHI, Jiro (aka jtFuruhata) <jt@do-johodai.ac.jp>
 * @version 0.0.1
 * @date    2023-03-21
 * 
 * @copyright   Copyright (c) 2023 jtLab, Hokkaido Information University,
 *              Release under the MIT License.
 *              See LICENSE.
 */

#include "Preferences.h"

String readFile(const char *filename){
    String result = "";
    Serial.println(filename);
    FILE *file = fopen(filename, "r");
    if (file) {
        char buf[33];
        int len = fread(buf, 1, sizeof(buf), file);
        fclose(file);
        if (len > 0) {
            buf[len] = '\0';
            result = String(buf);
        } else {
            remove(fileMaBeee);
        }
    }
    return result;
}

int writeFile(const char *filename, String data) {
    int result = -1;
    FILE *file = fopen(filename, "w");
    if (file) {
        char buf[33];
        strcpy(buf, data.c_str());
        result = fwrite(buf, 1, sizeof(buf), file);
        fclose(file);
    }
    return result;
}

bool hasPairedMaBeee() {
    return pairedMaBeeeName.length() > 0;
}

bool isPairedMaBeee(String name) {
    return pairedMaBeeeName.equals(name);
}

String getPairedMaBeeeName(){
    return pairedMaBeeeName;
}

bool setPairedMaBeeeName(String name){
    bool result = false;
    if (name.startsWith("MaBeee") && name.length() == 12) {
        pairedMaBeeeName = name;
        writeFile(fileMaBeee, pairedMaBeeeName);
        result = true;
    }
    return result;
}

bool hasAllowedHost() {
    return allowedHostAddress.length() > 0;
}

bool isAllowedHost(String address) {
    return !hasAllowedHost() || allowedHostAddress.equals(address);
}

String getAllowedHostAddress(){
    return allowedHostAddress;
}

bool setAllowedHostAddress(String address){
    bool result = false;
    if (address.length() == 17) {
        allowedHostAddress = address;
        if (writeFile(fileHost, address)) {
            result = true;
        }
    }
    return result;
}

void readSettings() {
    setPairedMaBeeeName(readFile(fileMaBeee));
    setAllowedHostAddress(readFile(fileHost));
}
