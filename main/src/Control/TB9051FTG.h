
// Include gaurd.
#ifndef TB9051FTG_H       
#define TB9051FTG_H

#include "Motor.h"
#include "Sentry/main/src/StateManager.h"

void user_ctrld_mvmt_task(void *pvTB9051FTG);   // User controlled movement task.
void erw_mvmt_task(void *pvTB9051FTG);          // Enhanced Random Walk movement task.

/**
 * Represents the TB9051FTG Motor Driver that drives a single
 * motor within the drive system.
 */
class TB9051FTG {

    private:
        Motor leftMotor;                    // Left Motor of the Sentry.
        Motor rightMotor;                   // Right Motor of the Sentry.
        UserDriveCommands *driveCommands;   // Pointer to global instance of user drive command data packet.

        void initTasks();
        BaseType_t beginUserControlledMovementTask();
        BaseType_t beginEnhancedRandomWalkMovementTask();
        
    public:
        TB9051FTG(UserDriveCommands *driveCommands) : 
            leftMotor(L_MOT_PWM1, L_MOT_PWM2), 
            rightMotor(R_MOT_PWM2, R_MOT_PWM1),
            driveCommands(driveCommands) {};


        // Initializes the drive system.
        void init();                

        // Alters the speed of the Sentry.
        void setSpeed(int speed);   

        // Advances the Sentry forwards.
        void moveForward();         

        // Reverses the Sentry.
        void moveBackward();        

        // Rotate the Sentry leftwards.
        void rotateLeft();             

        // Rotate the Sentry rightwards.
        void rotateRight();            

        /**
         * Stops the Sentry's movement by either braking or coasting. 
         * @param sType Type of Sentry Stop (BRAKE or COAST).
         */
        void stop(stopType sType);  
};

// End include gaurd.
#endif /* TB9051FTG.h */