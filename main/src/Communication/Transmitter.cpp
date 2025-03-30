#include "Transmitter.h"

// Define task handles.
TaskHandle_t tx_alerts_handle = NULL;        
TaskHandle_t tx_bme_data_handle = NULL;
TaskHandle_t tx_mic_data_handle = NULL;       

/**
 * This task handles transmissions from the Sentry to Firebase for reading 
 * by the partner application. Runs at highest priority for quick updates.
 */
void tx_alerts_task(void *pvTransmitter) {
    // Initialize task.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Transmitter *transmitter = static_cast<Transmitter *>(pvTransmitter);
    ulong lastTime = millis();

    // Enter task loop.
    for(;;) {
        
        // Wait for Alerts to be given.
        ulTaskNotifyTake(
            pdTRUE,             // Clear notification bits on exit..
            portMAX_DELAY);     // Time to wait for notif to be recieved. Setting this as Max delay 
                                // functionally means a seperate delay isn't required.

        // Take firebase app mutex for uniterrupted transmission.
        if(xSemaphoreTake(firebase_app_mutex, portMAX_DELAY) == pdTRUE) {
            Serial.printf("Time Elapsed since last Alert Transmit: %lu\n", millis() - lastTime);
            Serial.println("<-- FBAPP Mutex (Alert Tx Took).");
            transmitter->transmitAlerts();
            xSemaphoreGive(firebase_app_mutex);
            lastTime = millis();
            Serial.println("--> FBAPP Mutex (Alert Tx Gave).");
        }
        else Serial.println("FBAPP unavailible. Waiting. (Alert Tx Task)");
    }
}

void tx_bme_data_task(void *pvTransmitter) {
    // Initialize task.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Transmitter *transmitter = static_cast<Transmitter *>(pvTransmitter);
    ulong lastTime = millis();

    // Enter Task Loop.
    for(;;) {

        // Wait for BME to be finished reading.
        ulTaskNotifyTake(
            pdTRUE,             // Clear notification bits on exit..
            portMAX_DELAY);     // Time to wait for notif to be recieved. Setting this as Max delay 
                                // functionally means a seperate delay isn't required.

        // Take firebase app mutex for uniterrupted transmission.
        if(xSemaphoreTake(firebase_app_mutex, portMAX_DELAY) == pdTRUE) {
            //Serial.printf("Time Elapsed since last BME Transmit: %lu\n", millis() - lastTime);
            //Serial.println("<-- FBAPP Mutex (BME Tx Took).");
            transmitter->transmitSensorData(DataTransmissionType::tx_BME_DATA);
            xSemaphoreGive(firebase_app_mutex);
            lastTime = millis();
            //Serial.println("--> FBAPP Mutex (BME Tx Gave).");
        }
        else Serial.println("FBAPP unavailible. Waiting. (BME Tx Task)");
        
    }
}

void tx_mic_data_task(void *pvTransmitter) {
    // Initialize task.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Transmitter *transmitter = static_cast<Transmitter *>(pvTransmitter);
    ulong lastTime = millis();

    // Enter Task Loop.
    for(;;) {

        // Wait for MIC to be finished reading.
        ulTaskNotifyTake(
            pdTRUE,             // Clear notification bits on exit..
            portMAX_DELAY);     // Time to wait for notif to be recieved. Setting this as Max delay 
                                // functionally means a seperate dealy isn't required.

        // Take firebase app mutex for uniterrupted transmission.
        if(xSemaphoreTake(firebase_app_mutex, portMAX_DELAY) == pdTRUE) {
            //Serial.printf("Time Elapsed since last MIC Transmit: %lu\n", millis() - lastTime);
            //Serial.println("<-- FBAPP Mutex (MIC Tx Took).");
            transmitter->transmitSensorData(DataTransmissionType::tx_MIC_DATA);
            xSemaphoreGive(firebase_app_mutex);
            lastTime = millis();
            //Serial.println("--> FBAPP Mutex (Mic Tx Gave).");
        }
        else Serial.println("FBAPP unavailible. Waiting. (Mic Tx Task)");
        
    }
}

/**
 * Build up the sensor readings via Json Strings and then send them over to 
 * the proper address in Firebase.
 */
void Transmitter::buildBmeDataTransmission() {

    /*
    Serial.println("About to build environmental data packet for tranmission.");
    Serial.printf("-AQI: %f\n", envData->airQualityIndex);
    Serial.printf("-Humidity: %f\n", envData->humidityLevel);
    Serial.printf("-Pressure: %f\n", envData->pressureLevel);
    Serial.printf("-Temperature: %f\n", envData->temperatureLevel);
    */

    // Update data packet.
    // Write all sensor readings to Json Strings.
    jsonWriter.create(bmeTxBuffer[1], AQ_DATA_KEY, envData->airQualityIndex);
    jsonWriter.create(bmeTxBuffer[2], HUM_DATA_KEY, envData->humidityLevel);
    jsonWriter.create(bmeTxBuffer[3], TMP_DATA_KEY, envData->temperatureLevel);
    jsonWriter.create(bmeTxBuffer[4], DB_SPL_DATA_KEY, envData->noiseLevel);
    jsonWriter.create(bmeTxBuffer[5], PRESSURE_DATA_KEY, envData->pressureLevel);
    jsonWriter.create(bmeTxBuffer[6], VOC_DATA_KEY, envData->bVOClevel);
    jsonWriter.create(bmeTxBuffer[7], CO2_DATA_KEY, envData->CO2Level);
    jsonWriter.join(bmeTxBuffer[0], 7, bmeTxBuffer[1], bmeTxBuffer[2], bmeTxBuffer[3], bmeTxBuffer[4], bmeTxBuffer[5], bmeTxBuffer[6], bmeTxBuffer[7]);
}

void Transmitter::buildMicDataTransmisison() {
   micTxBuffer = envData->noiseLevel; 
}

/**
 * Per name. Build up the alerts via Json Strings and then send them over to 
 * the proper address in Firebase.
 */
void Transmitter::buildAlertsTransmission() {

    // Write all alert status' to Json Strings.
    jsonWriter.create(alertTxBuffer[1], AQ_ALERT_KEY, envStatus->airQualityStatus);
    jsonWriter.create(alertTxBuffer[2], HUM_ALERT_KEY, envStatus->humidityStatus);
    jsonWriter.create(alertTxBuffer[3], TMP_ALERT_KEY, envStatus->temperatureStatus);
    jsonWriter.create(alertTxBuffer[4], DB_SPL_ALERT_KEY, envStatus->noiseStatus);
    jsonWriter.create(alertTxBuffer[5], PRESSURE_ALERT_KEY, envStatus->pressureStatus);
    jsonWriter.create(alertTxBuffer[6], MOTION_ALERT_KEY, envStatus->motion);
    jsonWriter.join(alertTxBuffer[0], 6, alertTxBuffer[1], alertTxBuffer[2], alertTxBuffer[3], alertTxBuffer[4], alertTxBuffer[5], alertTxBuffer[6]);
}

/**
 * Actual Transmission method. Takes the data built and stored within the transmitters buffer 
 * and sends it over to Firebase.
 * @param writeAddress Address of field to write to in Firebase.
 * @param dtt Type of transmisison being written to Firebase.
 */
void Transmitter::updateFirebase(String writeAddress, DataTransmissionType dtt) {
    // Retrieve necessary objects for firebase transmission from the Connectivity Manager.
    RealtimeDatabase *firebase = _connManager->getFirebaseDatabase();
    AsyncClientClass *aClient = _connManager->getAsyncClient();
    FirebaseApp *fbApp = _connManager->getFbApp();

    // Transmit.
    bool txGood;
    fbApp->loop();   // For authentication purposes.
    if(fbApp->ready()) {
    
        // General environmental data transmission.
        if(dtt == DataTransmissionType::tx_BME_DATA || dtt == DataTransmissionType::tx_MIC_DATA) {
            //Serial.println("Transmitting sensor readings to Firebase");
            switch(dtt) {
                // BME Data transmission.
                case DataTransmissionType::tx_BME_DATA :
                    txGood = firebase->set<object_t>(*aClient, writeAddress, bmeTxBuffer[0]);
                    break;
                
                // Mic Data transmission.
                case DataTransmissionType::tx_MIC_DATA :
                    txGood = firebase->set<float>(*aClient, writeAddress, micTxBuffer);
                    break;
            }
        }

        // Sensor alerts.
        else if(dtt == DataTransmissionType::tx_ALERTS) {
            Serial.println("Transmitting Alerts to Firebase");
            txGood = firebase->set<object_t>(*aClient, FB_ALERTS_ADDRESS, alertTxBuffer[0]);
        }
        
        // Test transmission.
        else {
            Serial.println("Transmitting Test to Firebase");
            txGood = firebase->set<number_t>(*aClient, writeAddress, number_t(millis(), 3));
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
    BaseType_t taskCreated;
    taskCreated = beginBmeTxTask();
    if(taskCreated != pdPASS) Serial.printf("Transmit BME Data task not created. Fail Code: %d\n", taskCreated);
    else Serial.println("Transmit BME Data created.");

    taskCreated = beginMicTxTask();
    if(taskCreated != pdPASS) Serial.printf("Transmit Mic Data task not created. Fail Code: %d\n", taskCreated);
    else Serial.println("Transmit Mic Data task created.");

    taskCreated = beginAlertsTxTask();
    if(taskCreated != pdPASS) Serial.printf("Transmit Alert Data task not created. Fail Code: %d\n", taskCreated);
    else Serial.println("Transmit Alert Data task created.");

}

// Create the task to transmit environmental data.
BaseType_t Transmitter::beginBmeTxTask() {  
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &tx_bme_data_task,              // Pointer to task function.
        "tx_bme_data_Task",             // Task name.
        TaskStackDepth::tsd_TRANSMIT,   // Size of stack allocated to the task (in bytes).
        this,                           // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,    // Task priority level.
        &tx_bme_data_handle,            // Pointer to task handle.
        1                               // Core that the task will run on.
    );
    return res;
}

// Create the task to transmit microphone data.
BaseType_t Transmitter::beginMicTxTask() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &tx_mic_data_task,              // Pointer to task function.
        "tx_mic_data_Task",             // Task name.
        TaskStackDepth::tsd_TRANSMIT,   // Size of stack allocated to the task (in bytes).
        this,                           // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,    // Task priority level.
        &tx_mic_data_handle,            // Pointer to task handle.
        1                               // Core that the task will run on.
    );
    return res;
}

// Create the task to transmit environmental alerts. 
BaseType_t Transmitter::beginAlertsTxTask() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &tx_alerts_task,                // Pointer to task function.
        "tx_alerts_Task",               // Task name.
        TaskStackDepth::tsd_TRANSMIT,   // Size of stack allocated to the task (in bytes).
        this,                           // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_MEDIUM,  // Task priority level.
        &tx_alerts_handle,              // Pointer to task handle.
        1                               // Core that the task will run on.
    );
    return res;
}

/**
 * Initialize the data tranmission tasks. This should only be called 
 * after the Sentry is fully connected to Wi-Fi.
 */
void Transmitter::begin() {
    initTasks(); 
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

/** 
 * Transmit environmental sensor readings to the appropriate fields in Firebase.
 * @param dtt Type of transmission to write to Firebase.
 */
void Transmitter::transmitSensorData(DataTransmissionType dtt) {

    // Check to see if firebase connection is still active.
    // Currently nothing to do if connection isn't active....
    // Later: Check to see if Wi-Fi is still connected.
    // If so, attempt reconnection....
    if(_stateManager->getSentryConnectionState() != ConnectionState::ns_FB_AND_WF) return;     
    
    // Determine address to write to in Firebase and build the necessary packet for transmission.
    String writeAddress;
    switch(dtt) {
        case DataTransmissionType::tx_BME_DATA :
            writeAddress = FB_ENV_DATA_ADDRESS;
            buildBmeDataTransmission();
            break;
        
        case DataTransmissionType::tx_MIC_DATA :
            writeAddress = FB_ENV_DATA_ADDRESS + ((String) DB_SPL_DATA_KEY);
            buildMicDataTransmisison();
            break;

        default:
            delay(100); // simulate function call and return.
            writeAddress = "test_uptime";
            delay(100);
            break;
    }
    
    // Update the identified address in firebase.
    updateFirebase(writeAddress, dtt);
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
        buildAlertsTransmission();
        updateFirebase(FB_ALERTS_ADDRESS, DataTransmissionType::tx_ALERTS);
    }
}