#include "device.h"

void Device::begin() {
    // Initialize Sensor System.
    sensors.initAllSensors();
    sensors.attachAllInterrupts();
    sensors.beginAllTasks();

    // Initialize drive system.
    driver.init();
    driver.setSpeed(DEFAULT_SPEED);
}

void Device::sleep_mode_1() {

}

void Device::sleep_mode_2() {

}

void Device::shutdown() {

}

DRV8833 Device::getDriver() { return driver; }