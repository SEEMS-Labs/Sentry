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
    bool readingGood[4];

    // Create threshold monitoring variables.
    char currSensorBreached[4], lastSensorBreached[4];
    bool obsBreachDetected = false;
    bool hpeChangeDetected = false;
    
    // Initialize variables for proper resetting of fields in firebase (if needed).
    for(int i = 0; i < 4; i++) {
        readingGood[i] = false;
        currSensorBreached[i] = 0x00;
        lastSensorBreached[i] = 0xFF;
    }

    // Begin task loop.
    for(;;) {
        if(PRINT_US_ERR) Serial.println("Begin Reading Ultrasonic Sensors");
        
        // Read all 4 sensors on the specified order. At most ~40 ms to read 1 -> ~160ms to read all 4.
        readingGood[F_US_ID] = frontSensor->readSensor(US_READ_TIME);
        readingGood[B_US_ID] = backSensor->readSensor(US_READ_TIME);
        readingGood[L_US_ID] = leftSensor->readSensor(US_READ_TIME);
        readingGood[R_US_ID] = rightSensor->readSensor(US_READ_TIME);

        if(PRINT_US_TEST || PRINT_US_ERR) Serial.println("---------------------------------------------------------");
        if(PRINT_US_TEST) {
            if(frontSensor->isActive() && readingGood[F_US_ID]) {
                Serial.printf("Front\t Current reading: %.2f, LBA: %f, CBA:%f\n", 
                    frontSensor->getDistanceReading(), 
                    frontSensor->getLastBufferAverage(), 
                    frontSensor->averageBuffer());
            }
            //*
            if(backSensor->isActive() && readingGood[B_US_ID]) {
                Serial.printf("Back\t Current reading: %.2f, LBA: %f, CBA:%f\n", 
                    backSensor->getDistanceReading(), 
                    backSensor->getLastBufferAverage(), 
                    backSensor->averageBuffer());
            }
            if(leftSensor->isActive() && readingGood[L_US_ID]) {
                Serial.printf("Left\t Current reading: %.2f, LBA: %f, CBA:%f\n", 
                    leftSensor->getDistanceReading(), 
                    leftSensor->getLastBufferAverage(), 
                    leftSensor->averageBuffer());
            }
            if(rightSensor->isActive() && readingGood[R_US_ID]) {
            Serial.printf("Right\t Current reading: %.2f, LBA: %f, CBA:%f\n", 
                rightSensor->getDistanceReading(), 
                rightSensor->getLastBufferAverage(), 
                rightSensor->averageBuffer());
            }
            //*/
        }
    
        // Get thresholds breached for each sensor.
        currSensorBreached[F_US_ID] = (frontSensor->isActive()) ? frontSensor->passedThreshold() : 0x00;
        currSensorBreached[B_US_ID] = (backSensor->isActive()) ? backSensor->passedThreshold(): 0x00;
        currSensorBreached[L_US_ID] = (leftSensor->isActive()) ? leftSensor->passedThreshold() : 0x00;
        currSensorBreached[R_US_ID] = (rightSensor->isActive()) ? rightSensor->passedThreshold() : 0x00;
        
        // Determine threshold breaches to act on based on movement mode.
        MovementState currMvmtState = StateManager::getManager()->getSentryMovementState();
        
        // Deal with Sentinel Mode (aka Idle movement mode). Ultrasonics used for human presence estimation.
        if(currMvmtState == MovementState::ms_IDLE) {
            // Get human presence threshold breached flag for each sensor.
            currSensorBreached[F_US_ID] &= (PRESENCE_BREACH_STRENGTH_MASK | PRESENCE_THRESHOLD_BREACHED);
            currSensorBreached[B_US_ID] &= (PRESENCE_BREACH_STRENGTH_MASK | PRESENCE_THRESHOLD_BREACHED);
            currSensorBreached[L_US_ID] &= (PRESENCE_BREACH_STRENGTH_MASK | PRESENCE_THRESHOLD_BREACHED);
            currSensorBreached[R_US_ID] &= (PRESENCE_BREACH_STRENGTH_MASK | PRESENCE_THRESHOLD_BREACHED);

            // Update alerts packet and Notify if needed.
            hpeChangeDetected = (currSensorBreached[F_US_ID] != lastSensorBreached[F_US_ID]) || 
                                (currSensorBreached[B_US_ID] != lastSensorBreached[B_US_ID]) || 
                                (currSensorBreached[L_US_ID] != lastSensorBreached[L_US_ID]) || 
                                (currSensorBreached[R_US_ID] != lastSensorBreached[R_US_ID]);

            if(hpeChangeDetected) {

                if(PRINT_US_ERR) {
                    if(frontSensor->isActive() && readingGood[F_US_ID]) Serial.printf("👋 FrontBreached: curr->0x%x, last->0x%x\n", currSensorBreached[F_US_ID], lastSensorBreached[F_US_ID]);
                    if(backSensor->isActive() && readingGood[B_US_ID]) Serial.printf("👋 BackBreached: curr->0x%x, last->0x%x\n", currSensorBreached[B_US_ID], lastSensorBreached[B_US_ID]);
                    if(leftSensor->isActive() && readingGood[L_US_ID]) Serial.printf("👋 LeftBreached: curr->0x%x, last->0x%x\n", currSensorBreached[L_US_ID], lastSensorBreached[L_US_ID]);
                    if(rightSensor->isActive() && readingGood[R_US_ID]) Serial.printf("👋 RightBreached: curr->0x%x, last->0x%x\n", currSensorBreached[R_US_ID], lastSensorBreached[R_US_ID]);
                }

                // Keep track of sensor breach history.
                lastSensorBreached[F_US_ID] = currSensorBreached[F_US_ID];
                lastSensorBreached[B_US_ID] = currSensorBreached[B_US_ID];
                lastSensorBreached[L_US_ID] = currSensorBreached[L_US_ID];
                lastSensorBreached[R_US_ID] = currSensorBreached[R_US_ID];

                // Set the relevant field in the alerts package.
                uint8_t r_pd_strength = (rightSensor->isActive()) ? decodeHpeThreshold(currSensorBreached[R_US_ID]) : 0;
                uint8_t l_pd_strength = (leftSensor->isActive()) ? decodeHpeThreshold(currSensorBreached[L_US_ID]) : 0;
                uint8_t b_pd_strength = (backSensor->isActive()) ? decodeHpeThreshold(currSensorBreached[B_US_ID]) : 0;
                uint8_t f_pd_strength = (frontSensor->isActive()) ? decodeHpeThreshold(currSensorBreached[F_US_ID]) : 0;
                if(xSemaphoreTake(alert_buffer_mutex, portMAX_DELAY) == pdTRUE) {
                    alertInfoPacket->motion =   (r_pd_strength << R_HPE_LSB) | 
                                                (l_pd_strength << L_HPE_LSB) | 
                                                (b_pd_strength << B_HPE_LSB) | 
                                                (f_pd_strength << F_HPE_LSB);
                    xSemaphoreGive(alert_buffer_mutex);
                }
                if(PRINT_US_ERR) Serial.printf("\t\t[Strength: 0x%x]Front: %.2f in. < %.2f in.\n", f_pd_strength, frontSensor->getDistanceReading(), frontSensor->getHpeThreshold());
                
                if(PRINT_US_ERR) {
                    if(hpeChangeDetected) Serial.printf("\t✨Change in Presence Detected: AlertPacket: (0x%x). DecodedFront: 0x%x\n", alertInfoPacket->motion, f_pd_strength);
                    if(currSensorBreached[F_US_ID]) Serial.printf("\t\tFront: %.2f in. < %.2f in.\n", frontSensor->getDistanceReading(), frontSensor->getHpeThreshold());
                    if(currSensorBreached[B_US_ID]) Serial.printf("\t\tBack: %.2f in. < %.2f in.\n", backSensor->getDistanceReading(), backSensor->getHpeThreshold());
                    if(currSensorBreached[L_US_ID]) Serial.printf("\t\tLeft: %.2f in. < %.2f in.\n", leftSensor->getDistanceReading(), leftSensor->getHpeThreshold());
                    if(currSensorBreached[R_US_ID]) Serial.printf("\t\tRight: %.2f in. < %.2f in.\n", rightSensor->getDistanceReading(), rightSensor->getHpeThreshold());
                }

                // Notify.
                if(!SERIAL_ONLY_MODE) 
                    if(tx_alerts_handle != NULL) xTaskNotify(tx_alerts_handle, -1, eNoAction);
                    else Serial.println("Failed to notify tx_alerts task. Task likely not initialized.");
            }
        }

        // Deal with Sentry Modes (AKA Autonomous and Manual movement modes). Ultrasonics used for obstacle estimation.
        else if(currMvmtState != MovementState::ms_EMERGENCY_STOP) {
            // Get obstacle detection breached flag for each sensor.
            currSensorBreached[F_US_ID] &= OBSTACLE_THRESHOLD_BREACHED;
            currSensorBreached[B_US_ID] &= OBSTACLE_THRESHOLD_BREACHED;
            currSensorBreached[L_US_ID] &= OBSTACLE_THRESHOLD_BREACHED;
            currSensorBreached[R_US_ID] &= OBSTACLE_THRESHOLD_BREACHED;            

            // Update Obstacle Detection data packet if needed.
            obsBreachDetected = currSensorBreached[F_US_ID] | 
                                currSensorBreached[B_US_ID] | 
                                currSensorBreached[L_US_ID] | 
                                currSensorBreached[R_US_ID];

            obsDetectionPacket->frontObstacleDetected = currSensorBreached[F_US_ID];
            obsDetectionPacket->backObstacleDetected = currSensorBreached[B_US_ID];
            obsDetectionPacket->leftObstacleDetected = currSensorBreached[L_US_ID];
            obsDetectionPacket->rightObstacleDetected = currSensorBreached[R_US_ID];

            // Notify task.
            if(obsBreachDetected) {

                // Notify motor emergency stop task.
                if(obs_detect_stop_handle != NULL) xTaskNotify(obs_detect_stop_handle, OBSTACLE_THRESHOLD_BREACHED, eSetBits);
                else Serial.println("Failed to notify obs_detect_stop_handle. Task lkely not initialized.");


            }
        }

        // This case shouldn't be reached.
        else {
            Serial.println("Hmm. How did we get here?");
        }
        if(PRINT_US_TEST || PRINT_US_ERR) Serial.println("---------------------------------------------------------\n");

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

    // Create threhsold monitoring variables.
    char currMicThresholdBreached = THD_ALERT, lastMicThresholdBreached = THD_ALERT;
    bool changeDetected = false;

    // Main task loop.
    for(;;) {

        bool success = mic->readSensor();
        if(success) {
            // Grab current and last sound buffer averages for comparison.
            float currAverageSoundLevel = mic->averageBuffer();
            float lastAverageSoundLevel = mic->getLastAverageSoundLevel();

            // Check to see if alerts ready.
            currMicThresholdBreached = mic->passedThreshold();
            changeDetected = currMicThresholdBreached != lastMicThresholdBreached;
            if(changeDetected) {
                // Update alerts packet and last threshold.
                lastMicThresholdBreached = currMicThresholdBreached;
                alertInfoPacket->noiseStatus = (uint8_t) currMicThresholdBreached;
            
                // Notify alert transmission task.
                if(!SERIAL_ONLY_MODE) 
                    if(tx_alerts_handle != NULL) xTaskNotify(tx_alerts_handle, -1, eNoAction);
                    else Serial.println("Failed to notify tx_alerts_task. Task likely not initialized.");
            } 

            // Update data buffer. Notify transmit task.
            //Serial.printf("DATA PACKET PRE UPDATE: %f\n", sensorDataPacket->noiseLevel);
            sensorDataPacket->noiseLevel = currAverageSoundLevel;
            //Serial.printf("DATA PACKET POST UPDATE: %f\n", sensorDataPacket->noiseLevel);
            if(!SERIAL_ONLY_MODE && tx_mic_data_handle != NULL) xTaskNotify(tx_mic_data_handle, MIC_DATA_READY, eNoAction);
            else Serial.printf("%u: Sound Level: %f\n", millis(), currAverageSoundLevel);
            
        }

        // Delay.
        //Serial.println("Delaying from pollMic");
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(125));
        
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

    // Create threshold monitoring variables.
    char currThresholdsBreached = THD_ALERT, lastThresholdsBreached = THD_ALERT;
    bool changeDetected = false;

    // Begin task loop.
    int count = 0;
    for(;;) {

        // Read BME.
        //Serial.printf("\n--Begin polling BME.\n");
        bool success = bme->readSensor(BME_READ_TIME);
        //Serial.printf("--Done polling BME.\n");

        // Grab readings if succesful.
        if(success) {
            Serial.printf("--Reading was successful: %d\n", ++count);

            // Check to see if alerts ready.
            currThresholdsBreached = bme->passedThreshold();
            changeDetected = (currThresholdsBreached != lastThresholdsBreached);
            if(changeDetected) {
                // Update last threshold.
                lastThresholdsBreached = currThresholdsBreached;

                // Update alerts packet.
                alertInfoPacket->airQualityStatus = (currThresholdsBreached & AQI_BREACHED_MASK) ? UNSAFE : SAFE;
                alertInfoPacket->humidityStatus = (currThresholdsBreached & HUMIDITY_BREACHED_MASK) ? UNSAFE : SAFE;
                alertInfoPacket->pressureStatus = (currThresholdsBreached & PRESSURE_BREACHED_MASK) ? UNSAFE : SAFE;
                alertInfoPacket->temperatureStatus = (currThresholdsBreached & TEMPERATURE_BREACHED_MASK) ? UNSAFE : SAFE;
                alertInfoPacket->co2Status = (currThresholdsBreached & CO2_BREACHED_MASK) ? UNSAFE : SAFE;

                /*
                Serial.println("--------------------------");
                Serial.printf("AQI Breach: %d\n", (currThresholdsBreached & AQI_BREACHED_MASK) ? 1 : 0);
                Serial.printf("Humidity Breach: %d\n", (currThresholdsBreached & HUMIDITY_BREACHED_MASK) ? 1 : 0);
                Serial.printf("Pressure Breach: %d\n", (currThresholdsBreached & PRESSURE_BREACHED_MASK) ? 1 : 0);
                Serial.printf("Temperature Breach: %d\n", (currThresholdsBreached & TEMPERATURE_BREACHED_MASK) ? 1 : 0);
                Serial.printf("CO2 Breach: %d\n", (currThresholdsBreached & CO2_BREACHED_MASK) ? 1 : 0);
                Serial.println("--------------------------");
                //*/

                // Notify alert transmission task.
                if(!SERIAL_ONLY_MODE)  {
                    if(tx_alerts_handle != NULL) xTaskNotify(tx_alerts_handle, -1, eNoAction);
                    else Serial.println("Failed to notify tx_alerts_task. Task likely not initialized.");
                }
            }
            
            // Update data packet.
            sensorDataPacket->airQualityIndex = bme->get_IAQ_reading();
            sensorDataPacket->humidityLevel = bme->get_humidity_reading();
            sensorDataPacket->pressureLevel = bme->get_pressure_reading();
            sensorDataPacket->temperatureLevel = bme->get_temp_reading();
            sensorDataPacket->CO2Level = bme->get_CO2_reading();
            
            // Preupdate
            /*
            Serial.printf("AQI: %.2f\n", sensorDataPacket->airQualityIndex);
            Serial.printf("Humidity: %.2f\n", sensorDataPacket->humidityLevel);
            Serial.printf("Pressure: %.2f\n", sensorDataPacket->pressureLevel);
            Serial.printf("Temperature: %.2f\n\n", sensorDataPacket->temperatureLevel);
            //*/

            // Notify general data transmission task.
            //Serial.printf("--Notifying TX_BME_DATA_TASK\n");
            if(!SERIAL_ONLY_MODE) {
                if(tx_bme_data_handle != NULL) xTaskNotify(tx_bme_data_handle, BME_DATA_READY, eNoAction);
                else Serial.println("Failed to notify tx_bme_data_task. Task likely not initialized.");
            }
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

uint8_t decodeHpeThreshold(char breachField) {
    uint8_t res = 0;
    switch(breachField >> 2) {
        case (STRONG_PRESENCE_BREACH >> 2) :
            res = 3;
            break;
        case (MODERATE_PRESENCE_BREACH >> 2) :
            res = 2;
            break;
        case (WEAK_PRESENCE_BREACH >> 2) :
            res = 1;
            break;
        default:
            res = 0;
            break;
    }
    return res;
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
    if(frontUS->isActive()) frontUS->setHpeThreshold(preferences);
    if(backUS->isActive()) backUS->setHpeThreshold(preferences);
    if(leftUS->isActive()) leftUS->setHpeThreshold(preferences);
    if(rightUS->isActive()) rightUS->setHpeThreshold(preferences);
    if(mic->isActive()) mic->setThreshold(preferences);
    if(bme688->isActive()) bme688->setThresholds(preferences);
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
        TaskPriorityLevel::tpl_HIGH,        // Task priority level.
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