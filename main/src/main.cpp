
///*

// External libraries.
#include <Arduino.h>

// Internal headers.
#include "sentryConfigInfo.h"
#include "Device.h"

TaskHandle_t test1 = NULL;
TaskHandle_t test2 = NULL;
void taskFunc(void *pvParams);
void taskFunc2(void *pvParams);
void beginTasks();

// Global data storage.
SensorData outgoingData;
Alerts outgoingAlerts;
UserSentryConfig incomingUserConfig;
UserDriveCommands incomingUserCommands;
ObstacleData obstacleInfo;

// Create Sentry.
Device sentry(&outgoingData, &outgoingAlerts, &incomingUserConfig, &incomingUserCommands, &obstacleInfo);

//char *buffer;
void setup() {
    Serial.begin(BAUD_RATE);    
    vTaskDelay(pdMS_TO_TICKS(3000));           // delay to allow time to pullup serial monitor for debug.
    //buffer = (char *) malloc(sizeof(char) * 20 * 40);
    Serial.println("Entering Device Setup.");
    for(int i = 0; i < 5; i++) {
        Serial.println(".");
        delay(500);
    }
    //beginTasks();
    //sentry.test_mic_data_to_firebase();
    sentry.test_US();
    //sentry.test();
    //sentry.test_mic_data_to_firebase();
} 

void loop() {    
    //vTaskList(buffer);
    //Serial.printf("----------------------------\n");
    //Serial.printf(buffer);
    //Serial.printf("\n----------------------------\n");
    //sentry.showTaskMemoryUsage();
    //delay(3000);
    sentry.loop();
}
//*/

void beginTasks() {
    xTaskCreatePinnedToCore(
        &taskFunc,                  // Pointer to task function.
        "poll_US1_Task",                 // Task name.
        TaskStackDepth::tsd_POLL,       // Size of stack allocated to the task (in bytes).
        NULL,                           // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,    // Task priority level.
        &test1,                // Pointer to task handle.
        1                               // Core that the task will run on.
    );
    xTaskCreatePinnedToCore(
        &taskFunc2,                  // Pointer to task function.
        "poll_US2_Task",                 // Task name.
        TaskStackDepth::tsd_POLL,       // Size of stack allocated to the task (in bytes).
        NULL,                           // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,    // Task priority level.
        &test2,                // Pointer to task handle.
        1                               // Core that the task will run on.
    );
}

void taskFunc(void *pvParams) {
    // Setup.
    
    // Loop.
    for(;;) {
        while(1) Serial.println(" Bonjou ");
        delay(1000);
    }
}

void taskFunc2(void *pvParams) {
    // Setup.
    
    // Loop.
    for(;;) {
        while(1) Serial.println(" Bonswa ");
        delay(1000);
    }
}