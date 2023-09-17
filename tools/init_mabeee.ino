// load modules
#include "LED.h"    // LED singleton class definition (LED &led)

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

#define SERIAL_BAUDRATE 9600    // bps
#define SERIAL_TIMEOUT  5000    // ms

void removeFile(const char *filename){
    String result = "";
    Serial.print("Removing ");
    Serial.print(filename);
    FILE *file = fopen(filename, "r");
    if (file) {
        char buf[33];
        int len = fread(buf, 1, sizeof(buf), file);
        fclose(file);
        if (len > 0) {
            buf[len] = '\0';
            result = String(buf);
            Serial.print(": '");
            Serial.print(result);
            Serial.print("' (");
            Serial.print(len);
            Serial.println(")");
        }
        remove(fileMaBeee);
    } else {
        Serial.println(": file not found.");
    }
}

void setup() {
    // start serial port
    led.setColor(LED::WHITE);
    Serial.begin(SERIAL_BAUDRATE, SERIAL_8N1);
    while (!Serial && millis() < SERIAL_TIMEOUT);
    led.setColor(LED::BLACK);
    Serial.print("Initialising ETrobocon IoT Train controller on ");
    Serial.println(BOARD_NAME);

    //start LittleFS
    LittleFS = new FileSystem_MBED();
    if (!LittleFS->init()) {
        Serial.println("LittleFS failed!");
        while (1) {
            led.flashR();
            delay(800);
        }
    } else {
        Serial.print("Removing ");
        Serial.println(fileMaBeee);
        removeFile(fileMaBeee);
        Serial.print("Removing ");
        Serial.println(fileHost);
        removeFile(fileHost);
        led.setG();
        Serial.println("Done.");
        while (1);
    }
}

void loop() {
}
