// External libraries.
#include <Arduino.h>

///*
// Internal headers.
#include "sentryConfigInfo.h"
#include "Device.h"

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
    sentry.test_US();
    //sentry.test();
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
