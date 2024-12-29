
// Include gaurd.
#ifndef MOTOR_H
#define MOTOR_H

#include "Sentry/main/src/sentryConfigInfo.h"
#include <esp32-hal-ledc.h>
#include "driver/mcpwm.h"
#include <Arduino.h>

#define DRV8833_R_MOT_1 4
#define DRV8833_R_MOT_2 5
#define DRV8833_L_MOT_1 6
#define DRV8833_L_MOT_2 7
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
        const int posTerm;
        const int negTerm;
        int speed;

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