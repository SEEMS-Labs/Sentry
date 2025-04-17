
///*

// External libraries.
#include <Arduino.h>

// Internal headers.h
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

void setup() {
    Serial.begin(BAUD_RATE);    
    Serial.println("Entering Device Setup.");
    for(int i = 0; i < 5; i++) {
        Serial.println(".");
        delay(500);
    }
    sentry.begin();
} 

void loop() {    
    sentry.loop();
    vTaskDelay(pdMS_TO_TICKS(1000));
}

//*/
