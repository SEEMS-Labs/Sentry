#include "TB9051FTG.h"

// Define task handles.
TaskHandle_t user_ctrld_mvmt_handle = NULL;    
TaskHandle_t erw_mvmt_handle = NULL;  
TaskHandle_t monitor_ocm_handle = NULL;
TaskHandle_t monitor_diag_handle = NULL;

void user_ctrld_mvmt_task(void *pvTB9051FTG) {

    // Initialize w/ Drive system pointer.
    TB9051FTG *driver = static_cast<TB9051FTG *>(pvTB9051FTG);

    // Begin task loop.
    for(;;) {

        // Block indefinitely until user movement notifcation is received from the rx_user_data task.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 

        // Receive notification to stop
    }
}

void erw_mvmt_task(void *pvTB9051FTG) {

    // Initialize w/ Drive system pointer.
    TB9051FTG *driver = static_cast<TB9051FTG *>(pvTB9051FTG);

    for(;;) {
        // Receive notification of UC sensor readings. 
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 
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
        bool leftMotorCooked =leftMotor->monitorOverCurrentConditions() &&leftMotor->getEnableStatus();
        bool rightMotorCooked = rightMotor->monitorOverCurrentConditions() && rightMotor->getEnableStatus();

        ulong delta;
        ulong currTime = millis();
        if(leftMotorCooked) {
            delta = currTime - leftMotor->getOnTime();
            if(delta >= 2000)  {
                leftMotor->stop(stopType::BRAKE);
                Serial.println("Left Motor Went Over Current While Not Switching Directions. Stop Made.");
                Serial.printf("CurrTime = %u, onTime = %u, diff = %u\n", currTime, leftMotor->getOnTime(), delta);
            } 
            else Serial.println("Left Motor Went Over Current While Switching Directions. Stop Not Made.");
        }
        
        if(rightMotorCooked) {
            delta = currTime - rightMotor->getOnTime();
            if(delta > 2000) {
                rightMotor->stop(stopType::BRAKE);
                Serial.println("Right Motor Went Over Current While Not Switching Directions. Stop Made.");
                Serial.printf("CurrTime = %u, onTime = %u, diff = %u\n", currTime, rightMotor->getOnTime(), delta);
            } 
            else Serial.println("Right Motor Went Over Current While Switching Directions. Stop Not Made.");
        }

        // Check Motors every 125 ms.
        vTaskDelay(pdMS_TO_TICKS(125));
    }
}

void monitor_diag_task(void *pvTB9051FTG) {
    // Initialize.
    TB9051FTG *driver = static_cast<TB9051FTG *>(pvTB9051FTG);
    Motor *leftMotor = driver->getLeftMotor();
    Motor *rightMotor = driver->getRightMotor();

    // Task loop.
    for(;;) {
        
        vTaskDelay(pdMS_TO_TICKS(1000));
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

void TB9051FTG::init() {
    Serial.println("Motors Initializing!");
    leftMotor.init();
    rightMotor.init();
    initTasks();
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
    leftMotor.stop(stopType::BRAKE);
    rightMotor.spinCW();
}

void TB9051FTG::rotateRight() {
    rightMotor.stop(stopType::BRAKE);
    leftMotor.spinCCW();
}

Motor *TB9051FTG::getLeftMotor() {
    return &leftMotor;
}

Motor *TB9051FTG::getRightMotor() {
    return &rightMotor;
}

void TB9051FTG::stop(stopType sType) {
    Serial.println("Stopping!");
    leftMotor.stop(sType);
    rightMotor.stop(sType);
}
