/**
 * @brief   LED controller singleton for Seeed XIAO nRF52840 BLE (Sense)
 * @file        LED.h
 * @author  TANAHASHI, Jiro (aka jtFuruhata) <jt@do-johodai.ac.jp>
 * @version 1.2.0
 * @date    2023-03-22
 * 
 * @copyright   Copyright (c) 2023 jtLab, Hokkaido Information University,
 *              Release under the MIT License.
 *              See LICENSE.
 */

#pragma once
#ifndef LED_H_INCLUDE
#define LED_H_INCLUDE

#include <Arduino.h>    // Arduino core

/**
 * @brief 8 colors LED class
 */
class LED {
  public:
    /**
     * @brief LED color code (8 colors, old BRG style)
     */
    typedef enum {
        BLACK   = 0,
        BLUE    = 1,
        RED     = 2,
        MAGENTA = 3,
        GREEN   = 4,
        CYAN    = 5,
        YELLOW  = 6,
        WHITE   = 7
    } COLOR;

    /**
     * @brief LED color mask
     */
    typedef enum {
        B = 0x01,
        R = 0x02,
        G = 0x04
    } BRG;

  private:
    LED::COLOR color;     // LED color code
    static LED* instance; // singleton instance

    /**
     * @brief Construct a new LED singleton object
     */
    LED();

    /**
     * @brief Write status to the LED
     */
    void writeLED();

  public:
    /**
     * @brief Get the singleton instance of LED
     * 
     * @return LED* singleton instance
     */
    static LED* getInstance();

    /**
     * @brief Set the color of the LED
     * 
     * @param color specify the color to set
     */
    void setColor(LED::COLOR color);

    /**
     * @brief Turn on the specified LED
     * 
     * @param brg specify the LED to turn on
     */
    void setLED(LED::BRG brg);

    /**
     * @brief Turn on the Red LED
     */
    void setR();

    /**
     * @brief Turn on the Green LED
     */
    void setG();

    /**
     * @brief Turn on the Blue LED
     */
    void setB();

    /**
     * @brief Flash the LED
     * 
     * @param color color to flash
     * @param delayTime delay time between each flash
     * @param times flash times
     */
    void flashLED(LED::COLOR color, int delayTime=100, int times=1);

    /**
     * @brief Flash the specified LED
     * 
     * @param delayTime delay time between each flash
     * @param times flash times
     */
    void flashLED(LED::BRG brg, int delayTime=100, int times=1);

    /**
     * @brief Flash the Red LED
     * 
     * @param delayTime delay time between each flash
     * @param times flash times
     */
    void flashR(int delayTime=100, int times=1);

    /**
     * @brief Flash the Green LED
     * 
     * @param delayTime delay time between each flash
     * @param times flash times
     */
    void flashG(int delayTime=100, int times=1);

    /**
     * @brief Flash the Blue LED
     * 
     * @param delayTime delay time between each flash
     * @param times flash times
     */
    void flashB(int delayTime=100, int times=1);

    /**
     * @brief Turn off the specified LED
     * 
     * @param brg specify the LED to turn off
     */
    void resetLED(LED::BRG brg);

    /**
     * @brief Turn off the Red LED
     */
    void resetR();

    /**
     * @brief Turn off the Green LED
     */
    void resetG();

    /**
     * @brief Turn off the Blue LED
     */
    void resetB();

    /**
     * @brief Flip the specified LED
     * 
     * @param brg specify the LED to flip
     */
    void flipLED(LED::BRG brg);

    /**
     * @brief Flip the Red LED
     */
    void flipR();

    /**
     * @brief Flip the Green LED
     */
    void flipG();

    /**
     * @brief Flip the Blue LED
     */
    void flipB();

    /**
     * @brief Get the color of the LED
     * 
     * @return LED::COLOR the color of the LED
     */
    LED::COLOR getColor();

    /**
     * @brief Get the LEDs state
     * 
     * @param brg specify the LED to get the state
     * @return true if the specyfied LED is on, false if the specyfied LED is off
     */
    bool getLED(LED::BRG brg);

    /**
     * @brief Get the Red LED state
     * 
     * @return true if the Red LED is on, false if the Red LED is off
     */
    bool getR();

    /**
     * @brief Get the Green LED state
     * 
     * @return true if the Green LED is on, false if the Green LED is off
     */
    bool getG();

    /**
     * @brief Get the Blue LED state
     * 
     * @return true if the Blue LED is on, false if the Blue LED is off
     */
    bool getB();
};

extern LED& led; 

#endif  // _LED_H_
