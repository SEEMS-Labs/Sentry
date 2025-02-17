
// Include gaurd.
#ifndef MOTOR_H
#define MOTOR_H

#include "Sentry/main/src/sentryConfigInfo.h"
#include <esp32-hal-ledc.h>
#include "driver/mcpwm_prelude.h"
#include <Arduino.h>

#define PWM_FREQ 20000
#define PWM_RES 8
#define LED_C_LOW 0
#define LED_C_HIGH 255

enum stopType {
    COAST, 
    BRAKE
};

class Motor {
    
    private:
        const int posTerm;      // PWM 1 terminal.
        const int negTerm;      // PWM 2 terminal
        int speed;              // Speed motor should be driving in.

    public:
        Motor(int posTerm, int negTerm) : posTerm(posTerm), negTerm(negTerm) {};
        void init();
        void spinCW();
        void spinCCW();
        void setSpeed(int speed);
        void stop(stopType sType);
};

// End include gaurd.
#endif /* Motor.h */