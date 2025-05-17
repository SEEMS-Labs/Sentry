
// Include gaurd.
#ifndef TB9051FTG_H       
#define TB9051FTG_H

#include "esp_random.h"
#include "Motor.h"
#include "Sentry/main/src/StateManager.h"

enum _step : uint8_t { 
    Left,
    Right,
    Front,
    Back
};
typedef enum _step Step;

void user_ctrld_mvmt_task(void *pvTB9051FTG);       // User controlled movement task.
void erw_mvmt_task(void *pvTB9051FTG);              // Enhanced Random Walk movement task.
void monitor_ocm_task(void *pvTB9051FTG);           // Current Monitoring output task.
void obs_detect_stop_task(void *pvTB9051FTG);       // Motor emergency obstacle detected stop task.
void mvmt_manager_task(void *pvTB9051FTG);

/**
 * Represents the TB9051FTG Motor Driver that drives a single
 * motor within the drive system.
 */
class TB9051FTG {

    private:
        Motor leftMotor;                    // Left Motor of the Sentry.
        Motor rightMotor;                   // Right Motor of the Sentry.
        UserDriveCommands *driveCommands;   // Pointer to global instance of user drive command data packet.
        ObstacleData *obstacleInfo;         // Pointer to global instance of obstacle data for movement choices.

        BaseType_t beginUserControlledMovementTask();
        BaseType_t beginEnhancedRandomWalkMovementTask();
        BaseType_t beginMotorCurrentMonitoringTask();
        BaseType_t beginObstacleDetecetedStopTask();
        BaseType_t beginMovementManager();
        
    public:
        TB9051FTG(UserDriveCommands *driveCommands, ObstacleData *obstacleInfo) : 
            leftMotor(
                L_MOT_PWM1, 
                L_MOT_PWM2, 
                L_MOT_EN, 
                L_ENC_A, 
                L_ENC_B, 
                L_MOT_DIAG,
                L_MOT_OCM
            ), 
            rightMotor(
                R_MOT_PWM1, 
                R_MOT_PWM2, 
                R_MOT_EN, 
                R_ENC_A, 
                R_ENC_B, 
                R_MOT_DIAG,
                R_MOT_OCM
            ),
            driveCommands(driveCommands), obstacleInfo(obstacleInfo) {};


        // Initializes the drive system.
        void init();                
        
        // Starts the sentry's drive tasks.
        void initTasks();

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

        Motor *getLeftMotor();
        Motor *getRightMotor();
        UserDriveCommands *getUserDriveCommands();
        ObstacleData *getObstacleInfo();

        Step randomStep();
        Step enhancedRandomStep(); 
        void takeStep(Step s);
};

// End include gaurd.
#endif /* TB9051FTG.h */