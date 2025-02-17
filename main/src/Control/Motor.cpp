#include "Motor.h"

void Motor::init() {
    // Set pins as PWM outputs.
    ledcAttach(posTerm, PWM_FREQ, PWM_RES);
    ledcAttach(negTerm, PWM_FREQ, PWM_RES);
    
}

void Motor::spinCW() {
    ledcWrite(posTerm, LED_C_LOW);
    ledcWrite(negTerm, speed);
}

void Motor::spinCCW() {
    ledcWrite(posTerm, speed);
    ledcWrite(negTerm, LED_C_LOW);
}

void Motor::setSpeed(int speed) {
    this->speed = speed;
}

void Motor::stop(stopType sType) {

    // Select the stop type.
    switch (sType) {
        // To brake, write all high.
        case BRAKE: 
            ledcWrite(posTerm, LED_C_HIGH);
            ledcWrite(negTerm, LED_C_HIGH);
            break;
        
        // To coast write all low.
        case COAST:
            ledcWrite(posTerm, LED_C_LOW);
            ledcWrite(negTerm, LED_C_LOW);
            break;
    }
}