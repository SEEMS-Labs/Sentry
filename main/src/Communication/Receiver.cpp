#include "Receiver.h"

// Define Task Handles.
TaskHandle_t rx_user_data_handle = NULL;

/**
 * This task handles reception of data from SentryLink to the Sentry through
 * Firebase. Runs at highest priority for quick updates.
 */
void rx_user_data_task(void *pvReceiver) {
    // Setup.
    Receiver *_rcvr = static_cast<Receiver *>(pvReceiver);

    // Task loop.
    for(;;) {
        _rcvr->receiveSentryLinkUserData();
        vTaskDelay(pdMS_TO_TICKS(125));
    }
}

// Pre processing method to call actual reception processor.
void Receiver::receiveSentryLinkUserData() {
    AsyncResult sentryLinkStreamResult = _connManager->getSentryLinkStreamResult();
    receiveSentryLinkStream(sentryLinkStreamResult);
    sentryLinkStreamResult.clear();
} 

// Stream call back method to process information read from firebase.
void Receiver::receiveSentryLinkStream(AsyncResult &userData) {

    // Do no proccesing if there is nothing to process.
    if(!userData.isResult()) return;

    // Deal with repeat events.
    bool repeatEvent = checkForRepeatedResult(userData);
    if(repeatEvent) return;

    // Deal with non relevant events.
    if(userData.isEvent()) Firebase.printf("Event task: %s, msg: %s, code: %d\n", userData.uid().c_str(), userData.eventLog().message().c_str(), userData.eventLog().code());
    else if(userData.isDebug()) Firebase.printf("Debug task: %s, msg: %s\n", userData.uid().c_str(), userData.debug().c_str());
    else if(userData.isError()) Firebase.printf("Error task: %s, msg: %s, code: %d\n", userData.uid().c_str(), userData.error().message().c_str(), userData.error().code());

    // Deal with availible data.
    else if(userData.available()) {    

        // Update the last stream result to avoid repeate processing.
        RealtimeDatabaseResult &RTDB = userData.to<RealtimeDatabaseResult>();
        lastReadId = userData.uid();
        lastReadPayload = userData.payload();

        // Notify the appropriate tasks.
        if (RTDB.isStream()) {
            // Port data received to the proper form for decoding.
            uint64_t bitsToBeDecoded = RTDB.to<uint64_t>();
            
            // Grab the path of the field which was updated and act accordingly
            String fieldPath = RTDB.dataPath().substring(1);

            // Controller field updated.
            if(fieldPath.equals(SL_CTRL_PATH)) {
                Serial.println("Controller Field Updated. Notifying Manual Movement task if needed.");
                decodeAndUpdateUserDriveCommands((uint32_t) bitsToBeDecoded);
                if(_stateManager->getSentryMovementState() == MovementState::ms_MANUAL)
                    if(user_ctrld_mvmt_handle != NULL) xTaskNotify(user_ctrld_mvmt_handle, -1, eNoAction);
                    else Serial.println("Failed to notify user_ctrld_mvmt_task. Task likely not initialized.");
            }

            // User threhsold configuration field updated.
            else if(fieldPath.equals(SL_CONFIG_PATH)) {
                Serial.println("User Configuration Field Updated. Notifying Update_Thresholds_Task.");
                decodeAndUpdateUserConfigurationData((uint64_t) bitsToBeDecoded);
                if(update_thresholds_handle != NULL) xTaskNotify(update_thresholds_handle, -1, eNoAction);
                else Serial.println("Failed to notify update_thresholds_task. Task likely not initialized.");
            }

            // User in-app activity status updated.
            else if(fieldPath.equals(SL_ACTIVE_PATH)) {
                Serial.println("User In App Field Updated. Notify Later.");
                //xTaskNotify(rx_user_config_handle, -1, eNoAction);
            }

            // Deal with altered database paths.
            else {
                Serial.println("A previously undefined path was updated. Sentry currently only looks for the following subpaths: ");
                Serial.printf("%s, %s, and %s\n", SL_CTRL_PATH, SL_CONFIG_PATH, SL_ACTIVE_PATH);
                Serial.printf("Please add the path :%s to see changes.\n", RTDB.dataPath().c_str());
                Serial.printf("---------------------------->|%lu|\n", millis());
                Firebase.printf("task: %s\n", userData.uid().c_str());
                Firebase.printf("event: %s\n", RTDB.event().c_str());
                Firebase.printf("path: %s\n", RTDB.dataPath().c_str());
                Firebase.printf("data: %s\n", RTDB.to<const char *>());
                Firebase.printf("type: %d\n", RTDB.type());
            }
        }

        // Display other operations for now. Figure out if it's necessary later.
        else {
            Serial.println("----------------------------");
            Firebase.printf("task: %s, payload: %s\n", userData.uid().c_str(), userData.c_str());
        }
        //Firebase.printf("Free Heap: %d\n", ESP.getFreeHeap());
    }
    
}

// Start the receiver.
void Receiver::begin() {
    initTasks();
}

// Create the tasks to receive user data.
BaseType_t Receiver::beginUserDataRxTask() {
    BaseType_t res;
    res = xTaskCreatePinnedToCore(
        &rx_user_data_task,             // Pointer to task function.
        "rx_user_data_task",            // Task name.
        TaskStackDepth::tsd_RECEIVE,    // Size of stack allocated to the task (in bytes).
        this,                           // Pointer to parameters used for task creation.
        TaskPriorityLevel::tpl_HIGH,    // Task priority level.
        &rx_user_data_handle,           // Pointer to task handle.
        1                               // Core that the task will run on.
    );
    return res;
}

// Create all the necessary receiver tasks.
void Receiver::initTasks() {
    BaseType_t taskCreated;

    taskCreated = beginUserDataRxTask();
    if(taskCreated != pdPASS) Serial.printf("Receive User Data task not created. Fail Code: %d\n", taskCreated);
    else Serial.println("Receive User Data task created.");

}

/**
 * Retrieve the type of data the receiver received.
 * @param dataPathReceived Location in Firebase of data pulled from Firebase.
 * @returns The type of data read by the receiver.
 */
UserDataType Receiver::getDataTypeReceived(String dataPathReceived) {
    
    // Identifty data type.
    UserDataType res;
    if(dataPathReceived.equals(SL_CTRL_PATH)) res = UserDataType::UDT_MVMT;
    else if(dataPathReceived.equals(SL_CONFIG_PATH)) res = UserDataType::UDT_CONFIG;
    else if(dataPathReceived.equals(SL_ACTIVE_PATH)) res = UserDataType::UDT_ACTIVITY;
    
    // Return.
    return res;
}

/**
 * Decodes user configuration data and updates the global configuration data packet.
 * @param userConfigData Data to be decoded.
 */
void Receiver::decodeAndUpdateUserConfigurationData(uint64_t userConfigData) {

    Serial.printf("Data to be decoded: %llu\n", userConfigData);

    // Decode.
    float noiseLvl = (userConfigData >> SL_UserCfgInfo::SPL_LSB) & SL_UserCfgInfo::THN_MASK;
    float pressureLvl = (userConfigData >> SL_UserCfgInfo::PRS_LSB) & SL_UserCfgInfo::PRS_MASK;
    float temperatureLvl = (userConfigData >> SL_UserCfgInfo::TMP_LSB) & SL_UserCfgInfo::THN_MASK;
    float humidityLvl = (userConfigData >> SL_UserCfgInfo::HUM_LSB) & SL_UserCfgInfo::THN_MASK;
    float aqiLvl = (userConfigData >> SL_UserCfgInfo::AQI_LSB) & SL_UserCfgInfo::AQI_HPE_MASK;
    float hpeLvl = (userConfigData >> SL_UserCfgInfo::HPE_LSB) & SL_UserCfgInfo::AQI_HPE_MASK;

    // Fill the global configuration pacaket.
    userConfiguration->userAirQualityIndexThreshold = aqiLvl;
    userConfiguration->userHumidityLevelThreshold = humidityLvl;
    userConfiguration->userNoiseLevelThreshold = noiseLvl;
    userConfiguration->userPressureLevelThreshold = pressureLvl;
    userConfiguration->userTemperatureLevelThreshold = temperatureLvl;
    userConfiguration->userPresenceEstimationThreshold = hpeLvl;

    Serial.printf("Temperature Level Threshold: %.2f\n", userConfiguration->userTemperatureLevelThreshold);
    Serial.printf("Humidity Level Threshold: %.2f\n", userConfiguration->userHumidityLevelThreshold);
    Serial.printf("Noise Level Threshold: %.2f\n", userConfiguration->userNoiseLevelThreshold);
    Serial.printf("Pressure Level Threshold: %.2f\n", userConfiguration->userPressureLevelThreshold);
    Serial.printf("Air Quality Index Threshold: %.2f\n", userConfiguration->userAirQualityIndexThreshold);
    Serial.printf("Human Presence Estimation Threshold: %.2f\n", userConfiguration->userPresenceEstimationThreshold);
}

/**
 * Decodes user drive command data and updates the global drive data packet.
 * @param UserDriveCommandData Data to be decoded.
 */
void Receiver::decodeAndUpdateUserDriveCommands(uint32_t UserDriveCommandData) {

    // Decode initial data.
    bool ctrlPower = (UserDriveCommandData >> SL_ctrlInfo::ACTIVE_LSB) & SL_ctrlInfo::ACTIVE_MASK;
    char ctrlState = (UserDriveCommandData >> SL_ctrlInfo::CTRL_STA_LSB) & SL_ctrlInfo::CD_STA_MASK;
    char dpadField = (UserDriveCommandData >> SL_ctrlInfo::DPAD_STA_LSB) & SL_ctrlInfo::ACTIVE_MASK;
    signed short int joystickX = (UserDriveCommandData >> SL_ctrlInfo::JSTK_X_LSB) & SL_ctrlInfo::JOYSTICK_MASK;
    signed short int joystickY = (UserDriveCommandData >> SL_ctrlInfo::JSTK_Y_LSB) & SL_ctrlInfo::JOYSTICK_MASK;

    // Further decode.
    bool usingDpad = (ctrlState == DPAD_ACTIVE);
    bool usingJoystick = (ctrlState == JSTK_ACTIVE);
    bool dpadU = (dpadField == SL_ctrlInfo::DPAD_U);
    bool dpadD = (dpadField == SL_ctrlInfo::DPAD_D);
    bool dpadL = (dpadField == SL_ctrlInfo::DPAD_L);
    bool dpadR = (dpadField == SL_ctrlInfo::DPAD_R);

    // Fill the global configuration packet.
    userMovementCommands->active = ctrlPower;
    userMovementCommands->usingDpad = usingDpad;
    userMovementCommands->usingJoystick = usingJoystick;
    userMovementCommands->dpad_Forward = dpadU;
    userMovementCommands->dpad_Backward = dpadD;
    userMovementCommands->dpad_Left = dpadL;
    userMovementCommands->dpad_Right = dpadR;
    userMovementCommands->joystick_X = joystickX;
    userMovementCommands->joystick_Y = joystickY;

}


/**
 * Callback method that responds to the BLE Client connecting to 
 * the BLE Server that is the Sentry.
 * @param pServer Pointer to the BLE Server.
 */
void Receiver::onConnect(BLEServer* pServer) {
    // Set sentry connection state to BLE Connected.
    _stateManager->setSentryConnectionState(ConnectionState::ns_BLE);
}

/**
 * Callback method that responds to the BLE Client disconnecting from
 * the BLE Server that is the Sentry.
 * @param pServer Pointer to the BLE Server.
 */
void Receiver::onDisconnect(BLEServer *pServer) {
    // Set sentry connection state to none (indicating BLE disconnected).
    _stateManager->setSentryConnectionState(ConnectionState::ns_NONE);  
}

/**
 * Callback method that responds to the BLE Client writing to the 
 * characteristic shared by the BLE Server that is the Sentry.
 * @param pCharacteristic Pointer to the BLE Characteristic in question.
 */
void Receiver::onWrite(BLECharacteristic *pCharacteristic) {
    // Grab the value written to the characteristic from the BLE Client.
    bleDataAvailable = true;
    String value = pCharacteristic->getValue();

    // Read the data. Update buffer.
    value.trim();   // Trim to remove any leading or trailing whitespace.
    if (value.length() > 0) bleDataBuffer = value;
    else bleDataBuffer = BLE_INVALID_RX;
}

String Receiver::receiveBLEData() {
    bleDataAvailable = false;
    String data = bleDataBuffer;
    bleDataBuffer = "";
    return data;
}

bool Receiver::checkIfBLEDataAvailible(UserDataType udtData) {
    bool res = false;

    switch (udtData) {
        case UserDataType::UDT_CONFIG :
            break;
        case UserDataType::UDT_FB_AUTH :
            break;
        case UserDataType::UDT_MVMT :
            break;

        // Currently only Wi-Fi credentials are sent over BLE.
        case UserDataType::UDT_WIFI_AUTH :
            return bleDataAvailable;
    }

    return res;
}

bool Receiver::checkForRepeatedResult(AsyncResult res) {
    bool idsEqual = lastReadId.equals(res.uid());
    bool payloadsEqual = lastReadPayload.equals(res.payload());
    return (idsEqual && payloadsEqual);
}