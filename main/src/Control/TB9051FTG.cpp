#include "TB9051FTG.h"

// Define task handles.
TaskHandle_t user_ctrld_mvmt_handle = NULL;    
TaskHandle_t erw_mvmt_handle = NULL;  
TaskHandle_t monitor_ocm_handle = NULL;
TaskHandle_t obs_detect_stop_handle = NULL;
TaskHandle_t mvmt_manager_handle = NULL;

void user_ctrld_mvmt_task(void *pvTB9051FTG) {

    // Initialize w/ Drive system pointer.
    TB9051FTG *driver = static_cast<TB9051FTG *>(pvTB9051FTG);
    UserDriveCommands* commands = driver->getUserDriveCommands();

    // Begin task loop.
    for(;;) {

        // Block indefinitely until user movement notifcation is received from the rx_user_data task.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 


        // Do nothing when commands aren't proper. 
        if(commands->active == false && LAZY_CONTROLLER_MODE == false) continue;

        // Deal with using the dpad.
        if(commands->usingDpad) {
            if(commands->dpad_Forward) {
                driver->moveForward();
                log_e("Command: Move Fowrwards.");
            }
            else if(commands->dpad_Backward) {
                driver->moveBackward();
                log_e("Command. Move Backwards.");
            }
            else if(commands->dpad_Left) {
                driver->rotateLeft();
                log_e("Command. Move Left.");
            }
            else if(commands->dpad_Right) {
                driver->rotateRight();
                log_e("Command. Move Right.");
            }
            else {
                driver->stop(stopType::BRAKE);
                log_e("Dpad fall through. Errant State. Stopping Motors.");
            }
        }

        // Deal with using the joystick.
        else if(commands->usingJoystick) {
            driver->stop(stopType::BRAKE);
            log_e("Joystick Unimplmented. Errant State. Stopping Motors.");
        }
        
        // Default to turning the controller off.
        else {
            driver->stop(stopType::BRAKE);
            log_e("Controller state off. Stopping Motors.");
        }
    }
}

void erw_mvmt_task(void *pvTB9051FTG) {

    // Initialize w/ Drive system pointer.
    TickType_t xLastWakeTime =  xTaskGetTickCount();
    TB9051FTG *driver = static_cast<TB9051FTG *>(pvTB9051FTG);
    ObstacleData *obstacles = driver->getObstacleInfo();
    uint32_t randNumber = esp_random();
    bool stepsCalculated = false;
    int numOfSteps = 5;
    int min_steps = 5, max_steps = 10;
    Step res;

    for(;;) {

        // Wait to be removed from Sentinel mode to perfom steps.
        if(StateManager::getManager()->getSentryMovementState() == MovementState::ms_IDLE) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            numOfSteps = 5;
            stepsCalculated = false;
            StateManager::getManager()->setSentryMovementState(MovementState::ms_AUTONOMOUS);
        }

        // Perform 5-10 random walk steps. Choose by way of coin flip.
        if(!stepsCalculated) {
            while(randNumber % 2 == 0 && numOfSteps <= max_steps) numOfSteps++;
            stepsCalculated = true;
        } 

        // Deal w/ obstacles case.
        if(StateManager::getManager()->getSentryMovementState() == MovementState::ms_EMERGENCY_STOP) {
            StateManager::getManager()->setSentryMovementState(MovementState::ms_AUTONOMOUS);
            res = driver->enhancedRandomStep();
        }

        // Deal w/ normal random walk.
        else {
            res = driver->randomStep();
        }

        // Go back to idle mode.
        if(--numOfSteps == 0) {
            StateManager::getManager()->setSentryMovementState(MovementState::ms_IDLE);
            // Restart notif timer.
        }

        // Each step is 10s in length.
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ERW_STEP_LENGTH * 1000));
    }
}

void monitor_ocm_task(void *pvTB9051FTG) {

    // Initialize w/ Drive system pointer.
    TB9051FTG *driver = static_cast<TB9051FTG *>(pvTB9051FTG);
    Motor *leftMotor = driver->getLeftMotor();
    Motor *rightMotor = driver->getRightMotor();

    // Begin task loop.
    for(;;) {

        // Read pins.
        bool leftMotorCooked =leftMotor->monitorOverCurrentConditions() && leftMotor->getEnableStatus();
        bool rightMotorCooked = rightMotor->monitorOverCurrentConditions() && rightMotor->getEnableStatus();

        ulong delta;
        ulong currTime = millis();
        if(leftMotorCooked) {
            delta = currTime - leftMotor->getOnTime();
            if(delta >= 2000)  {
                leftMotor->stop(stopType::BRAKE);
                rightMotor->stop(stopType::BRAKE);
                Serial.println("Left Motor Went Over Current While Not Switching Directions. Stop Made.");
                Serial.printf("CurrTime = %u, onTime = %u, diff = %u\n", currTime, leftMotor->getOnTime(), delta);
            } 
            else Serial.println("Left Motor Went Over Current While Switching Directions. Stop Not Made.");
        }
        
        if(rightMotorCooked) {
            delta = currTime - rightMotor->getOnTime();
            if(delta > 2000) {
                rightMotor->stop(stopType::BRAKE);
                leftMotor->stop(stopType::BRAKE);
                Serial.println("Right Motor Went Over Current While Not Switching Directions. Stop Made.");
                Serial.printf("CurrTime = %u, onTime = %u, diff = %u\n", currTime, rightMotor->getOnTime(), delta);
            } 
            else Serial.println("Right Motor Went Over Current While Switching Directions. Stop Not Made.");
        }

        // Check Motors every 125 ms.
        vTaskDelay(pdMS_TO_TICKS(125));
    }
}

void obs_detect_stop_task(void *pvTB9051FTG) {
    // Initialize.
    TB9051FTG *driver = static_cast<TB9051FTG *>(pvTB9051FTG);

    // Task loop.
    for(;;) {
        // Receive notification from US sensor readings. 
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 

        // Stop both motors and declare an emergency stop.
        driver->stop(stopType::BRAKE);
        StateManager::getManager()->setSentryMovementState(MovementState::ms_EMERGENCY_STOP);
      
    }
}

void mvmt_manager_task(void *pvTB9051FTG) {
    // Setup.
    TickType_t xLastWakeTime =  xTaskGetTickCount();
    ulong startTime = millis();
    ulong currTime;

    // Loop.
    for(;;) {

        currTime = millis();
        if(StateManager::getManager()->getSentryMovementState() == MovementState::ms_MANUAL) startTime = millis();
        if(currTime - startTime >= 5*60*1000 && StateManager::getManager()->getSentryMovementState() == MovementState::ms_IDLE) {
            xTaskNotify(erw_mvmt_handle, -1, eNoAction);
            startTime = currTime;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ERW_STEP_LENGTH * 1000));
    }
}

void TB9051FTG::initTasks() {
    BaseType_t taskCreated;

    taskCreated = beginUserControlledMovementTask();
    if(taskCreated != pdPASS) Serial.printf("User Controlled Movement task not created. Fail Code: %d\n", taskCreated);
    else Serial.println("User Controlled Movement task created.");

    taskCreated = beginEnhancedRandomWalkMovementTask();
    if(taskCreated != pdPASS) Serial.printf("Enhanced Random Walk Movement task not created. Fail Code: %d\n", taskCreated);
    else Serial.println("Enhanced Random Walk Movement task created.");

    taskCreated = beginMotorCurrentMonitoringTask();
    if(taskCreated != pdPASS) Serial.printf("Motor Current Monitoring task not created. Fail Code: %d\n", taskCreated);
    else Serial.println("Motor Current Monitoring task created.");

    taskCreated = beginObstacleDetecetedStopTask();
    if(taskCreated != pdPASS) Serial.printf("Motor obstacle detection stop task not created. Fail Code: %d\n", taskCreated);
    else Serial.println("Motor obstacle detection stop task created.");

    taskCreated = beginMovementManager();
    if(taskCreated != pdPASS) Serial.printf("Motor Movement manager task not created. Fail Code: %d\n", taskCreated);
    else Serial.println("Motor movemenet manager task created.");
}

BaseType_t TB9051FTG::beginUserControlledMovementTask() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &user_ctrld_mvmt_task,          // Pointer to task function.
        "user_ctrld_mvmt_task",         // Task name.
        TaskStackDepth::tsd_DRIVE,      // Size of stack allocated to the task (in bytes).
        this,                           // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,    // Task priority level.
        &user_ctrld_mvmt_handle,        // Pointer to task handle.
        1                               // Core that the task will run on.
    );
    return res;
}

BaseType_t TB9051FTG::beginEnhancedRandomWalkMovementTask() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &erw_mvmt_task,                 // Pointer to task function.
        "erw_mvmt_task",                // Task name.
        TaskStackDepth::tsd_DRIVE,      // Size of stack allocated to the task (in bytes).
        this,                           // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,    // Task priority level.
        &erw_mvmt_handle,               // Pointer to task handle.
        1                               // Core that the task will run on.
    );
    return res;
}

BaseType_t TB9051FTG::beginMotorCurrentMonitoringTask() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &monitor_ocm_task,              // Pointer to task function.
        "monitor_ocm_task",             // Task name.
        TaskStackDepth::tsd_DRIVE,      // Size of stack allocated to the task (in bytes).
        this,                           // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,    // Task priority level.
        &monitor_ocm_handle,            // Pointer to task handle.
        1                               // Core that the task will run on.
    );
    return res;
}

BaseType_t TB9051FTG::beginObstacleDetecetedStopTask() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &obs_detect_stop_task,              // Pointer to task function.
        "obs_detect_stop_task",             // Task name.
        TaskStackDepth::tsd_DRIVE,          // Size of stack allocated to the task (in bytes).
        this,                               // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,        // Task priority level.
        &obs_detect_stop_handle,            // Pointer to task handle.
        1                                   // Core that the task will run on.
    );
    return res;
}

BaseType_t TB9051FTG::beginMovementManager() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &mvmt_manager_task,                 // Pointer to task function.
        "mvmt?",                            // Task name.
        TaskStackDepth::tsd_DRIVE,          // Size of stack allocated to the task (in bytes).
        this,                               // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,        // Task priority level.
        &mvmt_manager_handle,               // Pointer to task handle.
        1                                   // Core that the task will run on.
    );
    return res;
}

void TB9051FTG::init() {
    Serial.println("Motors Initializing!");
    leftMotor.init();
    rightMotor.init();
    Serial.println("Motors Initialized!");
}

void TB9051FTG::setSpeed(int speed) {
    Serial.println("Setting Motor Speed!");
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);
    Serial.println("Motor Speed Set!");
}
 
void TB9051FTG::moveForward() {
    Serial.println("Moving Forward!");
    leftMotor.spinCCW();
    rightMotor.spinCW();
}

void TB9051FTG::moveBackward() {
    Serial.println("Moving Backward!");
    leftMotor.spinCW();
    rightMotor.spinCCW();
}

void TB9051FTG::rotateLeft() {
    //leftMotor.stop(stopType::BRAKE);
    setSpeed(TURNING_SPEED);
    leftMotor.spinCW();
    rightMotor.spinCW();
    setSpeed(DEFAULT_SPEED);
}

void TB9051FTG::rotateRight() {
    //rightMotor.stop(stopType::BRAKE);
    setSpeed(TURNING_SPEED);
    rightMotor.spinCCW();
    leftMotor.spinCCW();
    setSpeed(DEFAULT_SPEED);
}

void TB9051FTG::stop(stopType sType) {
    leftMotor.stop(sType);
    rightMotor.stop(sType);
}

Motor *TB9051FTG::getLeftMotor() {
    return &leftMotor;
}

Motor *TB9051FTG::getRightMotor() {
    return &rightMotor;
}

UserDriveCommands *TB9051FTG::getUserDriveCommands() {
    return driveCommands;
}

ObstacleData *TB9051FTG::getObstacleInfo() {
    return obstacleInfo;
}

Step TB9051FTG::randomStep() {
    Step res;
    uint32_t randNum1 = esp_random();
    uint32_t randNum2 = esp_random();

    // Choose left or right
    if(randNum1 % 2 == 0) {
        if(randNum2 % 2 == 0) res = Step::Left;
        else res = Step::Right;
    }

    // Choose up or down.
    else {
        if(randNum2 % 2 == 0) res = Step::Front;
        else res = Step::Back;
    }

    // return res.
    return res;
}

Step TB9051FTG::enhancedRandomStep() {
    Step res = Step::Front;
    uint32_t randNum = esp_random();
    
    if(obstacleInfo->frontObstacleDetected) {
        // check if left and right side free.
        if(!obstacleInfo->leftObstacleDetected && !obstacleInfo->rightObstacleDetected) {
            res = (randNum % 2 == 0) ? Step::Left : Step::Right;
        } 

        // Check if left free.
        else if (obstacleInfo->leftObstacleDetected == false) {
            res = Step::Left;
        }

        // Check if right was free.
        else {
            res = Step::Right;
        }
    }

    return res;
} 

void TB9051FTG::takeStep(Step s) {
    switch (s) {
        case Step::Front :
            moveForward();
            break;
        
        case Step::Back :
            moveBackward();
            break;

        case Step::Left :
            rotateLeft();
            break;

        case Step::Right :
            rotateRight(); 
            break;

        default:
            stop(stopType::BRAKE);
            break;
    }

    vTaskDelay(pdMS_TO_TICKS(ERW_STEP_LENGTH * 1000));
    stop(stopType::BRAKE);

}