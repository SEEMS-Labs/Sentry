// External libraries.
#include <Arduino.h>

// Internal headers.
#include "sentryConfigInfo.h"
#include "Device.h"

// Global data storage.
SensorData outgoingData;
Alerts outgoingAlerts;
UserSentryConfig incomingUserConfig;
UserDriveCommands incomingUserCommands;

// Create Sentry.
Device sentry(&outgoingData, &outgoingAlerts, &incomingUserConfig, &incomingUserCommands);

void setup() {
    Serial.begin(BAUD_RATE);    
    vTaskDelay(pdMS_TO_TICKS(10000));           // 10s delay to allow time to pullup serial monitor for debug.
    Serial.println("Entering Device Setup.");
    for(int i = 0; i < 5; i++) {
        Serial.println(".");
        delay(500);
    }
    sentry.test_bme_and_mic_data_to_firebase();
} 

void loop() {}


