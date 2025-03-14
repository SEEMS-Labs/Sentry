
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

// Create Sentry.
Device sentry(&outgoingData, &outgoingAlerts, incomingUserConfig, incomingUserCommands);

void setup() {
    Serial.begin(BAUD_RATE);    
    vTaskDelay(pdMS_TO_TICKS(10000));           // 10s delay to allow time to pullup serial monitor for debug.
    Serial.println("Entering Device Setup.");
    for(int i = 0; i < 5; i++) {
        Serial.println(".");
        delay(500);
    }

    //vTaskDelete(NULL);                          // End setup.
} 

bool tested = false;
void loop() {
    if(!tested) {
        sentry.test_motor();
        tested = true;
    }
    else {
        Serial.println("Test Completed! Restart if you want to see it again!");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    //sentry.loop();
    //Serial.println("Looooop");
    //vTaskDelay(pdMS_TO_TICKS(1000));
}

