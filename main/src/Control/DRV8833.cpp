#include "DRV8833.h"

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
