
// Include gaurd.
#ifndef TB9051FTG_H       
#define TB9051FTG_H

#include "Motor.h"

void move_sentry_task(void *pvParameters);      // Movment task.
void walk_algorithm_task(void *pvParameters);   // RW task.

/**
 * Represents the TB9051FTG Motor Driver that drives a single
 * motor within the drive system.
 */
class TB9051FTG{

    private:
        Motor leftMotor;    // Left Motor of the Sentry.
        Motor rightMotor;   // Right Motor of the Sentry.
        
    public:
        TB9051FTG() : 
            leftMotor(L_MOT_PWM1, L_MOT_PWM2), 
            rightMotor(R_MOT_PWM1, R_MOT_PWM2) {};

        // Start movement task.
        void beginMovementTask();

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
#endif /* TB9051FTG.h */