/**
 * @brief   LED controller singleton for Seeed XIAO nRF52840 BLE (Sense)
 * @file        led.cpp
 * @author  TANAHASHI, Jiro (aka jtFuruhata) <jt@do-johodai.ac.jp>
 * @version 1.2.0
 * @date    2023-03-22
 * 
 * @copyright   Copyright (c) 2023 jtLab, Hokkaido Information University,
 *              Release under the MIT License.
 *              See LICENSE.
 */

#include "LED.h"

LED::LED() {
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    this->setColor(LED::BLACK);
}

void LED::writeLED() {
    digitalWrite(LEDR, this->getR() ? LOW : HIGH);
    digitalWrite(LEDG, this->getG() ? LOW : HIGH);
    digitalWrite(LEDB, this->getB() ? LOW : HIGH);
}

LED* LED::getInstance() {
    if (instance == nullptr) {
        instance = new LED();
    }
    return instance;
}

void LED::setColor(LED::COLOR color) {
    this->color = color;
    this->writeLED();
}

void LED::setLED(LED::BRG brg) {
    this->color = (LED::COLOR)((byte)this->color | (byte)brg);
    this->writeLED();
}

void LED::setR() {
    this->setLED(LED::R);
}

void LED::setG() {
    this->setLED(LED::G);
}

void LED::setB() {
    this->setLED(LED::B);
}

void LED::flashLED(LED::COLOR color, int delayTime, int times) {
    for (int i = 0; i < times; i++) {
        this->setColor(color);
        delay(delayTime);
        this->setColor(LED::BLACK);
        delay(delayTime);
    }
}

void LED::flashLED(LED::BRG brg, int delayTime, int times) {
    for (int i = 0; i < times; i++) {
        this->setLED(brg);
        delay(delayTime);
        this->resetLED(brg);
        delay(delayTime);
    }
}

void LED::flashR(int delayTime, int times) {
    this->flashLED(LED::R, delayTime, times);
}

void LED::flashG(int delayTime, int times) {
    this->flashLED(LED::G, delayTime, times);
}

void LED::flashB(int delayTime, int times) {
    this->flashLED(LED::B, delayTime, times);
}

void LED::resetLED(LED::BRG brg) {
    this->color = (LED::COLOR)((byte)this->color & ~(byte)brg);
    this->writeLED();
}

void LED::resetR() {
    this->resetLED(LED::R);
}

void LED::resetG() {
    this->resetLED(LED::G);
}

void LED::resetB() {
    this->resetLED(LED::B);
}

void LED::flipLED(LED::BRG brg) {
    this->color = (LED::COLOR)((byte)this->color ^ (byte)brg);
    this->writeLED();
}

void LED::flipR() {
    this->flipLED(LED::R);
}

void LED::flipG() {
    this->flipLED(LED::G);
}

void LED::flipB() {
    this->flipLED(LED::B);
}

LED::COLOR LED::getColor() {
    return this->color;
}

bool LED::getLED(LED::BRG brg) {
    return (this->color & (byte)brg) ? true : false;
}

bool LED::getR() {
    return this->getLED(LED::R);
}

bool LED::getG() {
    return this->getLED(LED::G);
}

bool LED::getB() {
    return this->getLED(LED::B);
}

LED* LED::instance = nullptr;

LED* LEDobject = LED::getInstance();
LED& led = *LEDobject;
