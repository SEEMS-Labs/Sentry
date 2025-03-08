#include "TB9051FTG.h"

// Define task handles.
TaskHandle_t move_sentry_handle = NULL;    
TaskHandle_t walk_algorithm_handle = NULL;   

void move_sentry_task(void *pvParameters) {

    // Initialize w/ Drive system pointer.
    TB9051FTG *driver = static_cast<TB9051FTG *>(pvParameters);

    // Begin task loop.
    for(;;) {

        // Receive notification to begin movement.

        // Receive notification to stop
    }
}

void walk_algorithm_task(void *pvParameters) {

    for(;;) {
        // Receive notification of UC sensor readings. 
    }
}

void TB9051FTG::beginMovementTask() {

}

void TB9051FTG::init() {
    Serial.println("Motors Initializing!");
    leftMotor.init();
    rightMotor.init();
    Serial.println("Motors Initialized!");
}

void TB9051FTG::setSpeed(int speed) {
    Serial.println("Setting Motor Speed!");
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);
    Serial.println("Motor Speed Set!");
}
 
void TB9051FTG::moveForward() {
    Serial.println("Moving Forward!");
    leftMotor.spinCCW();
    rightMotor.spinCW();
}

void TB9051FTG::moveBackward() {
    Serial.println("Moving backward!");
    leftMotor.spinCW();
    rightMotor.spinCCW();
}

void TB9051FTG::arcLeft() {

}

void TB9051FTG::arcRight() {

}

void TB9051FTG::stop(stopType sType) {
    Serial.println("Stopping!");
    leftMotor.stop(sType);
    rightMotor.stop(sType);
}
