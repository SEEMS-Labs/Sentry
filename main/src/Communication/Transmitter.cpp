#include "Transmitter.h"

// Define task handles.
TaskHandle_t tx_sensor_data_handle = NULL;                
TaskHandle_t tx_alerts_handle = NULL;               

void tx_sensor_data_task(void *pvTransmitter) {
    // Initialize task.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Transmitter *transmitter = static_cast<Transmitter *>(pvTransmitter);
    StateManager *stateManager = StateManager::getManager();

    // Enter task loop.
    for(;;) {
        // Wait for DATA_READY notification from sensor readings.
        // Update firebase w/ Data.

        // Delay.
        
    }
}

void tx_alerts_task(void *pvTransmitter) {
    // Initialize task.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Transmitter *transmitter = static_cast<Transmitter *>(pvTransmitter);

    // Enter task loop.
    for(;;) {
        
        // Wait for ALERT notification from Sensor readings.
        // Update firebase w/ Alerts that are present.

        // Delay.
    }
}

void Transmitter::updateSensorReadingsInFirebase() {

}

void Transmitter::updateAlertsInFirebase() {

}

void Transmitter::initTasks() {
    // Create the task to transmit sensor data.
    xTaskCreatePinnedToCore(
        &tx_sensor_data_task,   // Pointer to task function.
        "tx_sensor_data_Task",  // Task name.
        TASK_STACK_DEPTH,       // Size of stack allocated to the task (in bytes).
        this,                   // Pointer to parameters used for task creation.
        MAX_PRIORITY,           // Task priority level.
        &tx_sensor_data_handle, // Pointer to task handle.
        1                       // Core that the task will run on.
    );

    // Create the task to transmit environmental alerts. 
    xTaskCreatePinnedToCore(
        &tx_alerts_task,        // Pointer to task function.
        "tx_alerts_Task",       // Task name.
        TASK_STACK_DEPTH,       // Size of stack allocated to the task (in bytes).
        this,                   // Pointer to parameters used for task creation.
        MAX_PRIORITY,           // Task priority level.
        &tx_alerts_handle,      // Pointer to task handle.
        1                       // Core that the task will run on.
    );
}

/**
 * Initialize the data tranmission tasks. This should only be called 
 * after the Sentry is fully connected to Wi-Fi.
 */
void Transmitter::begin() { initTasks(); }

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
