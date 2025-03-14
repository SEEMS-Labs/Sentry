#include "Transmitter.h"

// Define task handles.
TaskHandle_t tx_sensor_data_handle = NULL;                
TaskHandle_t tx_alerts_handle = NULL;        
TaskHandle_t tx_bme_data_handle = NULL;
TaskHandle_t tx_mic_data_handle = NULL;       

void tx_sensor_data_task(void *pvTransmitter) {
    // Initialize task.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Transmitter *transmitter = static_cast<Transmitter *>(pvTransmitter);

    // Enter task loop.
    uint32_t notifValue;  // 32-bit task notification holder.
    for(;;) {
        // Wait for DATA_READY notification from sensor readings.
        xTaskNotifyWait(
            0x00,               // Notification bits to clear on entry.
            ULONG_MAX,          // Notification bits to clear on exit.
            &notifValue,        // Pointer to this tasks notificaton values.
            portMAX_DELAY);     // Time to wait for notif to be recieved.
        
        Serial.println("Ready to transmit.");

        // Update Firebase w/ BME Data.
        if((notifValue & BME_DATA_READY) == BME_DATA_READY) {
            Serial.println("Tx BME Data Soon (if Mutex here).");
            if(xSemaphoreTake(_sensor_data_buffer_mutex, portMAX_DELAY) == pdTRUE) {
                Serial.println("<-- Sensor Data Mutex (Tx Task).");
                transmitter->transmitSensorData();
                Serial.println("--> Sensor Data Mutex (Tx Task).");
                xSemaphoreGive(_sensor_data_buffer_mutex);
            }
            else Serial.println("Data unavailible. Waiting. (Tx Task)");
            vTaskDelay(pdMS_TO_TICKS(10));  
        }

        // Update Mic.
        if((notifValue & MIC_DATA_READY) == MIC_DATA_READY) {
            if(xSemaphoreTake(_sensor_data_buffer_mutex, portMAX_DELAY) == pdTRUE) {
                Serial.println("<-- Sensor Data Mutex (Tx Task).");
                transmitter->transmitSensorData();
                Serial.println("--> Sensor Data Mutex (Tx Task).");
                xSemaphoreGive(_sensor_data_buffer_mutex);
            }
            else Serial.println("Data unavailible. Waiting. (Tx Task)");
            vTaskDelay(pdMS_TO_TICKS(10));  
        }
    }
}

/**
 * This task handles transmissions from the Sentry to Firebase for reading 
 * by the partner application. Runs at highest priority for quick updates.
 */
void tx_alerts_task(void *pvTransmitter) {
    // Initialize task.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Transmitter *transmitter = static_cast<Transmitter *>(pvTransmitter);

    // Enter task loop.
    for(;;) {
        
        // Wait indefinitely for ALERT notification from Sensor readings.
        uint32_t pulseFinishedEvent = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        

        // Update firebase w/ Alerts that are present.
        //transmitter->transmitAlerts();
    }
}

void tx_bme_data_task(void *pvTransmitter) {

}

void tx_mic_data_task(void *pvTransmitter) {
    
}

/**
 * Per name.Build up the sensor readings via Json Strings and then send them over to 
 * the proper address in Firebase.
 */
object_t Transmitter::buildSensorReadingsTransmission() {

    Serial.println("About to build environmental data packet for tranmission.");
    Serial.printf("Preupdate AQI: %f\n", envData->airQualityIndex);
    Serial.printf("Preupdate Humidity: %f\n", envData->humidityLevel);
    Serial.printf("Preupdate Pressure: %f\n", envData->pressureLevel);
    Serial.printf("Preupdate Temperature: %f\n", envData->temperatureLevel);

    // Update data packet.
    // Write all sensor readings to Json Strings.
    JsonWriter writer;
    object_t jsonString[8];
    writer.create(jsonString[1], AQ_DATA_KEY, envData->airQualityIndex);
    writer.create(jsonString[2], HUM_DATA_KEY, envData->humidityLevel);
    writer.create(jsonString[3], TMP_DATA_KEY, envData->temperatureLevel);
    writer.create(jsonString[4], DB_SPL_DATA_KEY, envData->noiseLevel);
    writer.create(jsonString[5], PRESSURE_DATA_KEY, envData->pressureLevel);
    writer.create(jsonString[6], VOC_DATA_KEY, envData->bVOClevel);
    writer.create(jsonString[7], CO2_DATA_KEY, envData->CO2Level);
    writer.join(jsonString[0], 7, jsonString[1], jsonString[2], jsonString[3], jsonString[4], jsonString[5], jsonString[6], jsonString[7]);
    return jsonString[0];
}

/**
 * Per name. Build up the alerts via Json Strings and then send them over to 
 * the proper address in Firebase.
 */
object_t Transmitter::buildAlertsTransmission() {

    // Write all alert status' to Json Strings.
    JsonWriter writer;
    object_t jsonString[7];
    writer.create(jsonString[1], AQ_ALERT_KEY, envStatus->airQualityStatus);
    writer.create(jsonString[2], HUM_ALERT_KEY, envStatus->humidityStatus);
    writer.create(jsonString[3], TMP_ALERT_KEY, envStatus->temperatureStatus);
    writer.create(jsonString[4], DB_SPL_ALERT_KEY, envStatus->noiseStatus);
    writer.create(jsonString[5], PRESSURE_ALERT_KEY, envStatus->pressureStatus);
    writer.create(jsonString[6], MOTION_ALERT_KEY, envStatus->motion);
    writer.join(jsonString[0], 6, jsonString[1], jsonString[2], jsonString[3], jsonString[4], jsonString[5], jsonString[6]);
    return jsonString[0];
}

/**
 * Actual Transmission method. Takes the data written to Json and sends it over
 * to Firebase.
 * @param writeAddress Root address of location to write to in Firebase.
 */
void Transmitter::updateFirebase(String writeAddress, object_t jsonString) {
    // Retrieve necessary objects for firebase transmission from the Connectivity Manager.
    RealtimeDatabase firebase = _connManager->getFirebaseDatabase();
    AsyncClientClass aClient = _connManager->getAsyncClient();

    // Transmit.
    bool txGood;
    if(_connManager->getFbApp().ready()) {
        Serial.println("ABOUT TO TRANSMIT DATA TO FIREBASE");
        if(writeAddress == FB_ALERTS_ADDRESS) {
            Serial.println("Transmitting Alerts to Firebase");
            txGood = firebase.set<object_t>(aClient, FB_ALERTS_ADDRESS, jsonString);
        }
        else if(writeAddress == FB_ENV_DATA_ADDRESS) {
            Serial.println("Transmitting sensor readings to Firebase");
            txGood = firebase.set<object_t>(aClient, FB_ENV_DATA_ADDRESS, jsonString);
        }
    }
    else Serial.println("Firebase App is not ready... You're cooked");

    // FUTURE: Check to see if tranmission was succesful. Try sending again if not.
}

/**
 * Initialize the Necessary tasks.
 * Transmit Alerts and Transmit Sensor Data.
 */
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
        MEDIUM_PRIORITY,        // Task priority level.
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

/** 
 * Transmit environmental sensor readings to the appropriate fields in Firebase.
 */
void Transmitter::transmitSensorData() {
    // Check to see if firebase connection is still active.
    if(_stateManager->getSentryConnectionState() != ConnectionState::ns_FB_AND_WF);     // Currently nothing to do if connection isn't active....

    // Later: Check to see if Wi-Fi is still connected.
    // If so, attempt reconnection....

    else {
        object_t jsonString = buildSensorReadingsTransmission();
        updateFirebase(FB_ENV_DATA_ADDRESS, jsonString);
    }
}

/** 
 * Transmit environmental alerts to the appropriate fields in Firebase.
 */
void Transmitter::transmitAlerts() {
    // Check to see if firebase connection is still active.
    if(_stateManager->getSentryConnectionState() != ConnectionState::ns_FB_AND_WF);     // Currently nothing to do if connection isn't active....

    // Later: Check to see if Wi-Fi is still connected.
    // If so, attempt reconnection....

    else {
        object_t jsonString = buildAlertsTransmission();
        updateFirebase(FB_ALERTS_ADDRESS, jsonString);
    }
}

void Transmitter::transmitDistanceData() {
    // Check to see if firebase connection is still active.
    if(_stateManager->getSentryConnectionState() != ConnectionState::ns_FB_AND_WF);     // Currently nothing to do if connection isn't active....

    // Later: Check to see if Wi-Fi is still connected.
    // If so, attempt reconnection....

    else {
        JsonWriter writer;
        object_t jsonString;
        writer.create(jsonString, "distance", ghetto_distance);
        updateFirebase(FB_ENV_DATA_ADDRESS, jsonString);
    }
}