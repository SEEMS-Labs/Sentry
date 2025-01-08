#include "DRV8833.h"

// Define task handles.
TaskHandle_t move_sentry_handle = NULL;    
TaskHandle_t walk_algorithm_handle = NULL;   

void move_sentry_task(void *pvParameters) {

    // Initialize w/ Drive system pointer.
    DRV8833 *driver = static_cast<DRV8833 *>(pvParameters);

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

void DRV8833::beginMovementTask() {

}

void DRV8833::init() {
    leftMotor.init();
    rightMotor.init();
}

void DRV8833::setSpeed(int speed) {
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);
}
 
void DRV8833::moveForward() {
    leftMotor.spinCCW();
    rightMotor.spinCW();
}

void DRV8833::moveBackward() {
    leftMotor.spinCW();
    rightMotor.spinCCW();
}

void DRV8833::arcLeft() {

}

void DRV8833::arcRight() {

}

void DRV8833::stop(stopType sType) {
    leftMotor.stop(sType);
    rightMotor.stop(sType);
}
