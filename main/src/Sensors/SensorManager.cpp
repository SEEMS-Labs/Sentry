#include "SensorManager.h"
#include <Arduino.h>

// Define task handles.
TaskHandle_t poll_US_handle = NULL;                
TaskHandle_t poll_mic_handle = NULL;               
TaskHandle_t poll_bme_handle = NULL;     

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
    float readings[4];
    
    // Begin task loop.
    for(;;) {
        Serial.println("Begin Reading Sensors.");
        
        // Read the front sensor.
        readings[F_US_ID] = manager->fetchUS(F_US_ID)->readSensor(US_READ_TIME);

        // Read the back sensor.
        readings[B_US_ID] = manager->fetchUS(B_US_ID)->readSensor(US_READ_TIME);

        // Read the left sensor.
        readings[L_US_ID] = manager->fetchUS(L_US_ID)->readSensor(US_READ_TIME);

        // Read the right sensor.
        readings[R_US_ID] = manager->fetchUS(R_US_ID)->readSensor(US_READ_TIME);

        Serial.printf("Front(%d): %.2f in.\n", manager->fetchUS(F_US_ID)->isActive(), readings[F_US_ID]);
        Serial.printf("Back(%d): %.2f in.\n", manager->fetchUS(B_US_ID)->isActive(), readings[B_US_ID]);
        Serial.printf("Left(%d): %.2f in.\n", manager->fetchUS(L_US_ID)->isActive(), readings[L_US_ID]);
        Serial.printf("Right(%d): %.2f in.\n", manager->fetchUS(R_US_ID)->isActive(), readings[R_US_ID]);
        Serial.println("Done Reading Sensors.");

        // Poll sensors periodically.
        vTaskDelayUntil(&xLastWakeTime, MAX_US_POLL_TIME);
    }
}

void poll_mic_task(void *pvParameters) {

    for(;;) {

    }
} 

void poll_bme_task(void *pvParameters) {

    for(;;) {

    }
}   

void IRAM_ATTR on_front_us_echo_changed(void *arg) {
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

void IRAM_ATTR on_back_us_echo_changed(void *arg) {
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

void IRAM_ATTR on_left_us_echo_changed(void *arg) {
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

void IRAM_ATTR on_right_us_echo_changed(void *arg) {
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

void SensorManager::initAllSensors(){
    // Initialize ultrasonic sensors.
    frontUS.init();
    leftUS.init();
    rightUS.init();
    //backUS.init();
    
    // Initilze mic and bme.
    mic.init();
    //bme688.init();
}

void SensorManager::attachAllInterrupts(){
    // Attach ultrasonic interrupts.
    attachInterruptArg(ECHO_F, on_front_us_echo_changed, &frontUS, CHANGE);
    attachInterruptArg(ECHO_B, on_back_us_echo_changed, &backUS, CHANGE);
    attachInterruptArg(ECHO_L, on_left_us_echo_changed, &leftUS, CHANGE);
    attachInterruptArg(ECHO_R, on_right_us_echo_changed, &rightUS, CHANGE);
}

void SensorManager::beginAllTasks(){
    xTaskCreatePinnedToCore(&poll_US_task, "poll_US_Task", TASK_STACK_DEPTH, this, MAX_PRIORITY, &poll_US_handle, 1);
    //xTaskCreatePinnedToCore(&poll_mic_task, "poll_mic_Name", TASK_STACK_DEPTH, this, MAX_PRIORITY, &poll_mic_handle, 1);
    //xTaskCreatePinnedToCore(&poll_bme_task, "poll_bme_Name", TASK_STACK_DEPTH, this, MAX_PRIORITY, &poll_bme_handle, 1);
}

HCSR04* SensorManager::fetchUS(SensorID id) {
    switch (id) {
        case F_US_ID: return &frontUS;
        case B_US_ID: return &backUS;
        case L_US_ID: return &leftUS;
        case R_US_ID: return &rightUS;
    }

    // Return.
    return NULL;
}