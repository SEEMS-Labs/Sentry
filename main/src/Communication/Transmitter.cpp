#include "Transmitter.h"

void tx_sensor_data_task(void *pvParameters) {
    // Initialize task.

    // Enter task loop.
    for(;;) {
        // Wait for DATA_READY notification from sensor readings.
        // Update firebase w/ Data.
    }
}

void tx_alerts_task(void *pvParameters) {
    // Initialize task.

    // Enter task loop.
    for(;;) {
        
        // Wait for ALERT notification from Sensor readings.
        // Update firebase w/ Alerts that are present.
    }
}

void Transmitter::updateSensorReadingsInFirebase() {

}

void Transmitter::updateAlertsInFirebase() {

}

void Transmitter::initTasks() {

}

void Transmitter::begin() {

}

/**
 * Transmit information from the Sentry BLE Server to any client listening to the characteristic.
 * @param tx_code Transmission code holding information to be transmitted to the Client.
 * @param characteristic Pointer to the characteristic of the BLE server whose value is being updated.
 */
void Transmitter::transmitBLE(BLETransmitCode tx_code, BLECharacteristic *characteristic) {
    int code = (int) tx_code;
    characteristic->setValue(code);
    characteristic->notify();
}

void Transmitter::transmitSensorData() {

}

void Transmitter::transmitAlerts() {

}
