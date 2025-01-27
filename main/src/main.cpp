// External libraries.
#include <Arduino.h>
#include <FirebaseClient.h>

// Internal headers.
#include "sentryConfigInfo.h"
#include "Device.h"

// Global data storage.
SensorData outgoingData;
Alerts outgoingAlerts;
UserSentryConfig incomingUserConfig;
UserDriveCommands incomingUserCommands;

// Create Sentry.
Device sentry(outgoingData, outgoingAlerts, incomingUserConfig, incomingUserCommands);
void testDrive(Device sentry);

void setup() {
    Serial.begin(BAUD_RATE);    
    vTaskDelay(pdMS_TO_TICKS(10000));           // 10s delay to allow time to pullup serial monitor for debug.
    Serial.println("Entering Device Setup.");
    sentry.testComms();
    vTaskDelete(NULL);                          // End setup.
} 

void loop() {
    Serial.println("Normal Sentry Operations");
    vTaskDelay(pdMS_TO_TICKS(1000));
}

