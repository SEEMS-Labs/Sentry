#include "SensorManager.h"
#include <Arduino.h>

// Define task handles.
TaskHandle_t update_thresholds_handle = NULL;
TaskHandle_t poll_US_handle = NULL;                
TaskHandle_t poll_mic_handle = NULL;               
TaskHandle_t poll_bme_handle = NULL;    

/**
 * This task updates the thresholds of all sensors based on custom thresholds provided
 * by the user through firebase. 
 */
void update_thresholds_task(void *pvSensorManager) {
    // Initialize task.
    SensorManager *manager = static_cast<SensorManager *>(pvSensorManager);

    // Task loop.
    for(;;) {

        // Block indefinitely until notifcation is received from the rx_user_data task.
        // Time to wait for notification set as the maximum. Functionally means no seperate delay is needed.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   
        Serial.println("Time to update sensor thresholds(maybe)");
        manager->updateSensorThresholds();  
    }
}

/**
 * This task reads and provides the distance values measured from the ultrasonic sensors.
 * Order of reading: Front->Back->Left->Right. This order minimizes potential echoes and interference between 
 * sensors. Based on the way the sensors will be pulsed, the task will wait to be notified in the same order
 * from the interrupts: "on_xxxxx_us_echo_changed()" [xxxxx = front, back, etc.] and will act on the notifcation 
 * value that corresponds to the interrupt that just notified it. These values are defined in the header for this file.
 * @param *pvSensorManager a pointer to the Sensor Manager instance running from which the sensors will be pulsed.
 * each sensor will be pulsed using the HCSR04 "pulseTrigger" function. 
 */
void poll_US_task(void *pvSensorManager) {
    // Initialize task.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    SensorManager *manager = static_cast<SensorManager *>(pvSensorManager);

    // Grab ultrasonic sensors.
    HCSR04 *frontSensor = manager->fetchUS(F_US_ID);
    HCSR04 *backSensor = manager->fetchUS(B_US_ID);
    HCSR04 *leftSensor = manager->fetchUS(L_US_ID);
    HCSR04 *rightSensor = manager->fetchUS(R_US_ID);

    // Grab data packets.
    ObstacleData *obsDetectionPacket = manager->getObstaclesPacket();
    Alerts *alertInfoPacket = manager->getAlertsPacket();

    // Create reading success monitoring variables.
    bool frontGood, backGood, leftGood, rightGood;

    // Create threshold monitoring variables
    char frontBreached, backBreached, leftBreached, rightBreached;
    bool breachDetected;
    
    // Begin task loop.
    for(;;) {
        //Serial.println("Begin Reading Ultrasonic Sensors");
        
        // Read all 4 sensors on the specified order. At most ~40 ms to read 1 -> ~160ms to read all 4.
        frontGood = frontSensor->readSensor(US_READ_TIME);
        backGood = backSensor->readSensor(US_READ_TIME);
        leftGood = leftSensor->readSensor(US_READ_TIME);
        rightGood = rightSensor->readSensor(US_READ_TIME);

        //Serial.printf("FrontGood: %d, BackGood: %d, LeftGood: %d, RightGood: %d\n", frontGood, backGood, leftGood, rightGood);

        /*
        if(frontSensor->isActive()) Serial.printf("Front: %.2f in.\n", frontSensor->getDistanceReading());
        if(backSensor->isActive()) Serial.printf("Back: %.2f in.\n", frontSensor->getDistanceReading());
        if(leftSensor->isActive()) Serial.printf("Left: %.2f in.\n", frontSensor->getDistanceReading());
        if(rightSensor->isActive()) Serial.printf("Right: %.2f in.\n", frontSensor->getDistanceReading());
        */

        // Get thresholds breached for each sensor.
        frontBreached = frontSensor->passedThreshold();
        backBreached = backSensor->passedThreshold();
        leftBreached = leftSensor->passedThreshold();
        rightBreached = rightSensor->passedThreshold();
        
        // Determine threshold breaches to act on based on movement mode.
        MovementState currMvmtState = StateManager::getManager()->getSentryMovementState();
        
        // Deal with Sentinel Mode (aka Idle movement mode). Ultrasonics used for human presence estimation.
        if(currMvmtState == MovementState::ms_IDLE) {
            // Get human presence threshold breached flag for each sensor.
            frontBreached &= PRESENCE_THRESHOLD_BREACHED;
            backBreached &= PRESENCE_THRESHOLD_BREACHED;
            leftBreached &= PRESENCE_THRESHOLD_BREACHED;
            rightBreached &= PRESENCE_THRESHOLD_BREACHED;

            // Update alerts packet if needed.
            breachDetected = frontBreached || backBreached || leftBreached || rightBreached;
            alertInfoPacket->motion = (breachDetected) ? UNSAFE : SAFE;

            // Notify alert transmission task. 
            if(breachDetected) {
                Serial.println("✨Presence Detected, Finna Transmit.");
                if(frontBreached) Serial.printf("Front: %.2f in. < %.2f in.\n", frontSensor->getDistanceReading(), frontSensor->getHpeThreshold());
                if(backBreached) Serial.printf("Back: %.2f in. < %.2f in.\n", backSensor->getDistanceReading(), backSensor->getHpeThreshold());
                if(leftBreached) Serial.printf("Left: %.2f in. < %.2f in.\n", leftSensor->getDistanceReading(), leftSensor->getHpeThreshold());
                if(rightBreached) Serial.printf("Right: %.2f in. < %.2f in.\n", rightSensor->getDistanceReading(), rightSensor->getHpeThreshold());
                xTaskNotify(tx_alerts_handle, PRESENCE_THRESHOLD_BREACHED, eSetBits);
            }
        }

        // Deal with Sentry Modes (AKA Autonomous and Manual movement modes). Ultrasonics used for obstacle estimation.
        else if(currMvmtState != MovementState::ms_EMERGENCY_STOP) {
            // Get obstacle detection breached flag for each sensor.
            frontBreached &= OBSTACLE_THRESHOLD_BREACHED;
            backBreached &= OBSTACLE_THRESHOLD_BREACHED;
            leftBreached &= OBSTACLE_THRESHOLD_BREACHED;
            rightBreached &= OBSTACLE_THRESHOLD_BREACHED;            

            // Update Obstacle Detection data packet if needed.
            breachDetected = frontBreached || backBreached || leftBreached || rightBreached;
            obsDetectionPacket->frontObstacleDetected = frontBreached;
            obsDetectionPacket->backObstacleDetected = backBreached;
            obsDetectionPacket->leftObstacleDetected = leftBreached;
            obsDetectionPacket->rightObstacleDetected = rightBreached;

            // Notify tasks.
            if(breachDetected) {
                // Notify random walk task.
                if(currMvmtState == MovementState::ms_AUTONOMOUS) xTaskNotify(erw_mvmt_handle, OBSTACLE_THRESHOLD_BREACHED, eSetBits);

                // Notify user controlled movement task.
                else if(currMvmtState == MovementState::ms_MANUAL) xTaskNotify(user_ctrld_mvmt_handle, OBSTACLE_THRESHOLD_BREACHED, eSetBits);
            }
        }

        // This case shouldn't be reached.
        else {
            Serial.println("Hmm. How did we get here?");
        }

        // Poll sensors periodically.
        vTaskDelayUntil(&xLastWakeTime, MAX_US_POLL_TIME + 250);
    }
}

void poll_mic_task(void *pvSensorManager) {

    // Initialize task.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    SensorManager *manager = static_cast<SensorManager *>(pvSensorManager);

    // Grab microphone sensor and environmental data packets.
    Microphone *mic = manager->fetchMic();
    SensorData *sensorDataPacket = manager->getEnvironmentalDataPacket();
    Alerts *alertInfoPacket = manager->getAlertsPacket();

    // Take 10 readings before averaging.
    int count = 0;

    // Main task loop.
    for(;;) {

        bool success = mic->readSensor(-1);
        if(success) {
            //Serial.printf("Succesful mic poll: %d\n", count);
            if(count > 10) {
                //Serial.println("Enough samples. Mic output now!");
                // Reset counter and take average of readings.
                count = 0;
                //float soundLevel = mic->averageBuffer();
                float soundLevel = mic->getLastSoundLevelReading();
                //Serial.printf("Data Value: %f\n", soundLevel);

                // Check to see if alerts ready.
                //*
                char thresholdCheck = mic->passedThreshold();
                if(thresholdCheck == THD_ALERT) {
                    // Update alerts packet.
                    alertInfoPacket->noiseStatus = UNSAFE;
                
                    // Notify alert transmission task.
                    xTaskNotify(tx_alerts_handle, -1, eNoAction);
                } 
                //*/

                // Update data buffer. Notify transmit task.
                //Serial.printf("DATA PACKET PRE UPDATE: %f\n", sensorDataPacket->noiseLevel);
                sensorDataPacket->noiseLevel = soundLevel;
                //Serial.printf("DATA PACKET POST UPDATE: %f\n", sensorDataPacket->noiseLevel);
                xTaskNotify(tx_mic_data_handle, MIC_DATA_READY, eNoAction);
            }
            else count++;
        }

        // Delay.
        //Serial.println("Delaying from pollMic");
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
        
    }
} 

void poll_bme_task(void *pvSensorManager) {

    // Initialize task.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    SensorManager *manager = static_cast<SensorManager *>(pvSensorManager);

    // Grab BME688 sensor and alert data packet.
    BME688 *bme = manager->fetchBME();
    SensorData *sensorDataPacket = manager->getEnvironmentalDataPacket();
    Alerts *alertInfoPacket = manager->getAlertsPacket();

    // Begin task loop.
    for(;;) {

        // Read BME.
        //Serial.printf("\n--Begin polling BME.\n");
        bool success = bme->readSensor(BME_READ_TIME);
        //Serial.printf("--Done polling BME.\n");

        // Grab readings if succesful.
        if(success) {
            //Serial.println("--Reading was successful.");

            // Check to see if alerts ready.
            char thresholdCheck = bme->passedThreshold();
            if(thresholdCheck != !THD_ALERT) {
                // Update alerts packet.
                alertInfoPacket->airQualityStatus = (thresholdCheck & AQI_BREACHED_MASK) ? UNSAFE : SAFE;
                alertInfoPacket->humidityStatus = (thresholdCheck & HUMIDITY_BREACHED_MASK) ? UNSAFE : SAFE;
                alertInfoPacket->pressureStatus = (thresholdCheck & PRESSURE_BREACHED_MASK) ? UNSAFE : SAFE;
                alertInfoPacket->temperatureStatus = (thresholdCheck & TEMPERATURE_BREACHED_MASK) ? UNSAFE : SAFE;
                alertInfoPacket->co2Status = (thresholdCheck & CO2_BREACHED_MASK) ? UNSAFE : SAFE;
                alertInfoPacket->bVOCStatus = (thresholdCheck & VOC_BREACHED_MASK) ? UNSAFE : SAFE;

                // Notify alert transmission task.
                if(!SERIAL_ONLY_MODE) xTaskNotify(tx_alerts_handle, -1, eNoAction);
            }
            
            /*
            Serial.printf("Besc AQI: %f\n", bme->get_IAQ_reading());
            Serial.printf("Bsec Humidity: %f\n", bme->get_humidity_reading());
            Serial.printf("Bsec Pressure: %f\n", bme->get_pressure_reading());
            Serial.printf("Bsec Temperature: %f\n", bme->get_temp_reading());

            // Preupdate
            Serial.printf("Preupdate AQI: %f\n", sensorDataPacket->airQualityIndex);
            Serial.printf("Preupdate Humidity: %f\n", sensorDataPacket->humidityLevel);
            Serial.printf("Preupdate Pressure: %f\n", sensorDataPacket->pressureLevel);
            Serial.printf("Preupdate Temperature: %f\n", sensorDataPacket->temperatureLevel);
            */

            // Update data packet.
            sensorDataPacket->airQualityIndex = bme->get_IAQ_reading();
            sensorDataPacket->humidityLevel = bme->get_humidity_reading();
            sensorDataPacket->pressureLevel = bme->get_pressure_reading();
            sensorDataPacket->temperatureLevel = bme->get_temp_reading();
            sensorDataPacket->bVOClevel = bme->get_VOC_reading();
            sensorDataPacket->CO2Level = bme->get_CO2_reading();
            
            // Preupdate
            /*
            Serial.printf("Postupdate AQI: %f\n", sensorDataPacket->airQualityIndex);
            Serial.printf("Postupdate Humidity: %f\n", sensorDataPacket->humidityLevel);
            Serial.printf("Postupdate Pressure: %f\n", sensorDataPacket->pressureLevel);
            Serial.printf("Postupdate Temperature: %f\n", sensorDataPacket->temperatureLevel);
            */

            // Notify general data transmission task.
            //Serial.printf("--Notifying TX_BME_DATA_TASK\n");
            if(!SERIAL_ONLY_MODE) xTaskNotify(tx_bme_data_handle, BME_DATA_READY, eNoAction);
        }
        else Serial.printf("--Reading was unsuccessful. You're cooked. BME Active: %d\n", bme->checkSensorStatus());

        // Poll BME periodically.
        vTaskDelayUntil(&xLastWakeTime, MAX_BME_POLL_TIME);
        
    }
}   

void on_front_us_echo_changed(void *arg) {
    ulong currTime = micros();
    int pinState = digitalRead(ECHO_F);
    HCSR04 *frontSensor = static_cast<HCSR04 *>(arg);
    if(pinState == HIGH) frontSensor->setISRStartPulse(currTime);
    else {
        frontSensor->setIRSEndPulse(currTime);
        BaseType_t higherPriorityWasAwoken = pdFALSE;
        xTaskNotifyFromISR(poll_US_handle, F_US_READY, eSetBits, &higherPriorityWasAwoken);
        portYIELD_FROM_ISR(higherPriorityWasAwoken);
    }
}

void on_back_us_echo_changed(void *arg) {
    ulong currTime = micros();
    int pinState = digitalRead(ECHO_B);
    HCSR04 *backSensor = static_cast<HCSR04 *>(arg);
    if(pinState == HIGH) backSensor->setISRStartPulse(currTime);
    else {
        backSensor->setIRSEndPulse(currTime);
        BaseType_t higherPriorityWasAwoken = pdFALSE;
        xTaskNotifyFromISR(poll_US_handle, B_US_READY, eSetBits, &higherPriorityWasAwoken);
        portYIELD_FROM_ISR(higherPriorityWasAwoken);
    }
} 

void on_left_us_echo_changed(void *arg) {
    ulong currTime = micros();
    int pinState = digitalRead(ECHO_L);
    HCSR04 *leftSensor = static_cast<HCSR04 *>(arg);
    if(pinState == HIGH) leftSensor->setISRStartPulse(currTime);
    else {
        leftSensor->setIRSEndPulse(currTime);
        BaseType_t higherPriorityWasAwoken = pdFALSE;
        xTaskNotifyFromISR(poll_US_handle, L_US_READY, eSetBits, &higherPriorityWasAwoken);
        portYIELD_FROM_ISR(higherPriorityWasAwoken);
    } 
} 

void on_right_us_echo_changed(void *arg) {
    ulong currTime = micros();
    int pinState = digitalRead(ECHO_R);
    HCSR04 *rightSensor = static_cast<HCSR04 *>(arg);
    if(pinState == HIGH) rightSensor->setISRStartPulse(currTime);
    else {
        rightSensor->setIRSEndPulse(currTime);
        BaseType_t higherPriorityWasAwoken = pdFALSE;
        xTaskNotifyFromISR(poll_US_handle, R_US_READY, eSetBits, &higherPriorityWasAwoken);
        portYIELD_FROM_ISR(higherPriorityWasAwoken);
    }
}

/**
 * Initialize the 4 US sensors, Mic, and BME.
 */
void SensorManager::initAllSensors(){
    initUS();
    initMic();
    initBME();
}

/**
 * Initialze the Sentry's 4 Ultrasonic Sensors.
 */
void SensorManager::initUS() {
    // Construct the ultrasonic sensors.
    frontUS = new HCSR04(TRIG_F, ECHO_F, F_US_ID, DEF_F_OBS_LIM, this);
    backUS = new HCSR04(TRIG_B, ECHO_B, B_US_ID, DEF_B_OBS_LIM, this);
    leftUS = new HCSR04(TRIG_L, ECHO_L, L_US_ID, DEF_L_OBS_LIM, this);
    rightUS = new HCSR04(TRIG_R, ECHO_R, R_US_ID, DEF_R_OBS_LIM, this);

    // Initialize the ultrasonic sensors.
    frontUS->init();
    leftUS->init();
    rightUS->init();
    backUS->init();

    // Set the human presence estimation thresholds of the ultrasonic sensors.
    frontUS->setHpeThreshold(preferences);
    leftUS->setHpeThreshold(preferences);
    rightUS->setHpeThreshold(preferences);
    backUS->setHpeThreshold(preferences);
}

/**
 * Initialize the Sentry's Microphone.
 */
void SensorManager::initMic() {
    mic->init();
    mic->setThreshold(preferences);
}

/**
 * Initialize the Sentry's environmental sensor.
 */
void SensorManager::initBME() {
   bme688->init();  
   bme688->setThresholds(preferences);
}

void SensorManager::attachAllInterrupts(){
    // Attach ultrasonic interrupts.
    attachInterruptArg(ECHO_F, on_front_us_echo_changed, frontUS, CHANGE);
    attachInterruptArg(ECHO_B, on_back_us_echo_changed, backUS, CHANGE);
    attachInterruptArg(ECHO_L, on_left_us_echo_changed, leftUS, CHANGE);
    attachInterruptArg(ECHO_R, on_right_us_echo_changed, rightUS, CHANGE);
}

// Per name.
void SensorManager::beginAllTasks() {

    // Create and begin all sensor based tasks.
    BaseType_t taskCreated;

    taskCreated = beginReadUltrasonicTask();
    if(taskCreated != pdPASS) Serial.printf("Read Ultrasonic Sensor task not created. Fail Code: %d\n", taskCreated);
    else Serial.println("Read Ultrasonic Sensor task created.");

    taskCreated = beginReadMicrophoneTask();
    if(taskCreated != pdPASS) Serial.printf("Read Microphone task not created. Fail Code: %d\n", taskCreated);
    else Serial.println("Read Microphone task created.");

    taskCreated = beginReadBMETask();
    if(taskCreated != pdPASS) Serial.printf("Read BME task not created. Fail Code: %d\n", taskCreated);
    else Serial.println("Read BME task created.");

    taskCreated = beginUpdateThresholdTask();
    if(taskCreated != pdPASS) Serial.printf("Update Sensor Thresholds task not created. Fail Code: %d\n", taskCreated);
    else Serial.println("Update Sensor Thresholds task created.");
    
}

HCSR04* SensorManager::fetchUS(SensorID id) {
    HCSR04 *res = NULL;
    switch (id) {
        case F_US_ID: 
            res = frontUS;
            break;
        case B_US_ID: 
            res = backUS;
            break;
        case L_US_ID: 
            res = leftUS;
            break;
        case R_US_ID: 
            res = rightUS;
            break;
    }
    return res;
}

BME688* SensorManager::fetchBME() {
    return bme688;
}

Microphone* SensorManager::fetchMic() {
    return mic;
}

Alerts* SensorManager::getAlertsPacket() {
    return alertInfo;
}

ObstacleData* SensorManager::getObstaclesPacket() {
    return obstacleInfo;
}

SensorData* SensorManager::getEnvironmentalDataPacket() {
    return environmentalData;
}

UserSentryConfig *SensorManager::getUserSentryConfigDataPacket() {
    return userConfig;
}

void SensorManager::updateSensorThresholds() {
    frontUS->setHpeThreshold(preferences);
    backUS->setHpeThreshold(preferences);
    leftUS->setHpeThreshold(preferences);
    rightUS->setHpeThreshold(preferences);
    mic->setThreshold(preferences);
    bme688->setThresholds(preferences);
}

// Create the task to poll the 4 ultrasonic sensors.
BaseType_t SensorManager::beginReadUltrasonicTask() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &poll_US_task,                  // Pointer to task function.
        "poll_US_Task",                 // Task name.
        TaskStackDepth::tsd_POLL,       // Size of stack allocated to the task (in bytes).
        this,                           // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,    // Task priority level.
        &poll_US_handle,                // Pointer to task handle.
        1                               // Core that the task will run on.
    );
    return res;
}

// Create the task to poll the Microphone.
BaseType_t SensorManager::beginReadMicrophoneTask() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &poll_mic_task, 
        "poll_mic_Task", 
        TaskStackDepth::tsd_POLL, 
        this, 
        TaskPriorityLevel::tpl_MEDIUM, 
        &poll_mic_handle, 
        1
    ); 
    return res; 
}

// Create the task to poll thebme688->
BaseType_t SensorManager::beginReadBMETask() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &poll_bme_task,                 // Pointer to task function.
        "poll_bme_Task",                // Task name.
        TaskStackDepth::tsd_POLL,       // Size of stack allocated to the task (in bytes).
        this,                           // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_MEDIUM,  // Task priority level.
        &poll_bme_handle,               // Pointer to task handle.
        1                               // Core that the task will run on.
    );
    return res;
}

// Create the task to update sensor thresholds.
BaseType_t SensorManager::beginUpdateThresholdTask() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &update_thresholds_task,            // Pointer to task function.
        "update_sensor_thresholds_task",    // Task name.
        TaskStackDepth::tsd_SET,            // Size of stack allocated to the task (in bytes).
        this,                               // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_MEDIUM_LOW,  // Task priority level.
        &update_thresholds_handle,          // Pointer to task handle.
        1                                   // Core that the task will run on.
    );
    return res;
}
void SensorManager::constructAllSensors() {
    frontUS = new HCSR04(TRIG_F, ECHO_F, F_US_ID, DEF_F_OBS_LIM, this);
    backUS = new HCSR04(TRIG_B, ECHO_B, B_US_ID, DEF_B_OBS_LIM, this);
    leftUS = new HCSR04(TRIG_L, ECHO_L, L_US_ID, DEF_L_OBS_LIM, this);
    rightUS = new HCSR04(TRIG_R, ECHO_R, R_US_ID, DEF_R_OBS_LIM, this);
    bme688 = new BME688(BME_SDI, BME_SCK, this);
    mic = new Microphone(MIC_ANALOG_OUT, this);
}