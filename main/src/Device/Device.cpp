#include "device.h"

void Device::begin() {
    
    // Initialize Sentry Subsystems.
    initCommunicationSystem();
    initSensorSystem();
    initDriveSystem();
}

void Device::sleep_mode_1() {

}

void Device::sleep_mode_2() {

}

void Device::shutdown() {

}

TB9051FTG Device::get_drive_system() { return _drive_system; }

void Device::testComms() {
    // Initialize Communication System.
    Serial.println("Beginning Comms Test.");
    for(int i = 0; i < 10; i++) {
        Serial.println(".");
        delay(500);
    }
    _communication_system.begin();
    Serial.println("Ending Comms Test.");
}

void Device::test_data_to_firebase() {
    Serial.println("Beginning Transmission of Sensor Data to Firebase test");
    for(int i = 0; i < 10; i++) {
        Serial.println(".");
        delay(500);
    }
    initCommunicationSystem();
    initSensorSystem();
}

void Device::test_US() {
    _sensor_system.initAllSensors();
}

void Device::initSensorSystem() {
    _sensor_system.initAllSensors();
    //_sensor_system.attachAllInterrupts();
    _sensor_system.createSemaphores();
    _sensor_system.beginAllTasks();
}

void Device::initDriveSystem() {
    _drive_system.init();
    _drive_system.setSpeed(DEFAULT_SPEED);
}


void Device::initCommunicationSystem() {
    _communication_system.begin();
}

void Device::loop() {
    // Loop the FIREBASEAPP for auth purposes.
    _communication_system.getFbApp().loop();
}