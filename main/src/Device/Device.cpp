#include "device.h"

void Device::begin() {

    // Initialize Communication System.
    _communication_system.begin();

    // Initialize Sensor System.
    _sensor_system.initAllSensors();
    _sensor_system.attachAllInterrupts();
    _sensor_system.beginAllTasks();

    // Initialize Drive system.
    _drive_system.init();
    _drive_system.setSpeed(DEFAULT_SPEED);
}

void Device::sleep_mode_1() {

}

void Device::sleep_mode_2() {

}

void Device::shutdown() {

}

DRV8833 Device::get_drive_system() { return _drive_system; }

void Device::testComms() {
    // Initialize Communication System.
    Serial.println("Beginning Comms Test.");
    vTaskDelay(pdMS_TO_TICKS(10000));
    _communication_system.begin();
    Serial.println("Ending Comms Test.");
}