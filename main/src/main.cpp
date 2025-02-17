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
Device sentry(outgoingData, outgoingAlerts, incomingUserConfig, incomingUserCommands);

void testDrive(Device Sentry);

void setup() {
    Serial.begin(BAUD_RATE);    
    vTaskDelay(pdMS_TO_TICKS(10000));           // 10s delay to allow time to pullup serial monitor for debug.
    Serial.println("Entering Device Setup.");
    for(int i = 0; i < 10; i++) {
        Serial.println(".");
        delay(500);
    }
    sentry.begin();
    testDrive(sentry);
    vTaskDelete(NULL);                          // End setup.
} 

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void testDrive(Device sentry) {
    // Delay to wait for floor placement.
    delay(pdMS_TO_TICKS(15 * 1000));

    uint32_t rand = esp_random();
    if(rand % 5 == 0) delay(2 * ONE_SECOND);
    else if(rand % 3 == 0) delay(0.1 * ONE_SECOND);
    else if (rand % 2 == 0) delay(0.01 * ONE_SECOND);
    else delay(ONE_SECOND);
    // Move forward for 4 s. Brake.
    sentry.get_drive_system().moveForward();
    delay(pdMS_TO_TICKS(4 * ONE_SECOND));
    sentry.get_drive_system().stop(BRAKE);

    // Move backwards for 2 s. Coast to stop.
    sentry.get_drive_system().moveBackward();
    delay(pdMS_TO_TICKS(ONE_SECOND));
    sentry.get_drive_system().stop(COAST);
}