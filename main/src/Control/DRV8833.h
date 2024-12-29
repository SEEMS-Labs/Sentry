
// Include gaurd.
#ifndef DRV8833_H
#define DRV8833_H

#include "Motor.h"

/**
 * Represents the DRV8833 Dual Motor Driver that controls the drive system.
 */
class DRV8833 {

    private:
        Motor leftMotor;    // Left Motor of the Sentry.
        Motor rightMotor;   // Right Motor of the Sentry.
    public:
        DRV8833() : 
            leftMotor(DRV8833_L_MOT_1, DRV8833_L_MOT_2), 
            rightMotor(DRV8833_R_MOT_1, DRV8833_R_MOT_2) {};

        // Initializes the drive system.
        void init();                

        // Alters the speed of the Sentry.
        void setSpeed(int speed);   

        // Advances the Sentry forwards.
        void moveForward();         

        // Reverses the Sentry.
        void moveBackward();        

        // Unimplemented. Arcs the Sentry leftwards.
        void arcLeft();             

        // Unimplemented. Arcs the Sentry rightwards.
        void arcRight();            

        /**
         * Stops the Sentry's movement by either braking or coasting. 
         * @param sType Type of Sentry Stop (BRAKE or COAST).
         */
        void stop(stopType sType);  
};

// End include gaurd.
#endif /* DRV8833.h */