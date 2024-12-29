#include <Arduino.h>
#include <FirebaseClient.h>

#include "sentryConfigInfo.h"
#include "Device.h"

// Create Sentry.
Device sentry;
void testDrive(Device sentry);

void setup() {
    Serial.begin(BAUD_RATE);    // Serial.
    sentry.begin();             // Start the sentry.
    testDrive(sentry);          // Test the drive system.
    vTaskDelete(NULL);          // End setup.
} 

void loop() {}

void testDrive(Device sentry) {
    // Delay to wait for floor placement.
    delay(pdMS_TO_TICKS(15 * 1000));

    // Move forward for 4 s. Brake.
    sentry.getDriver().moveForward();
    delay(pdMS_TO_TICKS(4 * ONE_SECOND));
    sentry.getDriver().stop(BRAKE);

    // Move backwards for 2 s. Coast to stop.
    sentry.getDriver().moveBackward();
    delay(pdMS_TO_TICKS(ONE_SECOND));
    sentry.getDriver().stop(COAST);
}
