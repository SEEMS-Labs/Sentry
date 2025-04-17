
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
        const int posTerm;          // PWM 1 terminal.
        const int negTerm;          // PWM 2 terminal
        const int enablePin;        // Enable pin.
        const int encoderA;         // Encoder A output.
        const int encoderB;         // Encoder B output.
        const int diagnostic;       // Diagnostic Pin.
        const int currentMonitor;   // Current Monitoring Pin.
        
        int speed = 0;              // Speed motor should be driving in.
        float currLim = 200.0;     // Current limit through motor in milli-Amps.
        bool enabled = false;       // Motor enable pin status.
        bool isSwitching = false;   // Motor direction switching status for cases where the motor stops but experiences overcurrent.
        ulong onTime = 0;           // Time (in ms) that the motor turned on from rest.

    public:
        Motor(int posTerm, int negTerm, int enablePin, int encoderA, int encoderB, int diagnostic, int currentMonitor) : 
            posTerm(posTerm), 
            negTerm(negTerm),
            enablePin(enablePin),
            encoderA(encoderA),
            encoderB(encoderB),
            diagnostic(diagnostic),
            currentMonitor(currentMonitor) {};

        void init();
        void enable();  // Turn on the motor.
        void disable(); // Turn off the motor.
        void spinCW();
        void spinCCW();
        void setSpeed(int speed);
        int getSpeed();
        bool monitorOverCurrentConditions();
        bool monitorDiagnosticConditions();
        bool getSwitchingDirectionStatus();
        ulong getOnTime();
        void stop(stopType sType);

        bool getEnableStatus();
        //bool getRotationStatus();
};

// End include gaurd.
#endif /* Motor.h */