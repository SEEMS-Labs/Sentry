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

        // Potentially take semaphore to use app loop.
        _rcvr->receiveSentryLinkUserData();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Pre processing method to call actual reception processor.
void Receiver::receiveSentryLinkUserData() {
    AsyncResult *sentryLinkStreamResult = _connManager->getSentryLinkStreamResult();
    receiveSentryLinkStream(*sentryLinkStreamResult);
} 

// Stream call back method to process information read from firebase.
AsyncResultCallback Receiver::receiveSentryLinkStream(AsyncResult &userData) {
    
    // Do no proccesing if there is nothing to process.
    if(!userData.isResult()) return NULL;

    // Deal with non relevant events.
    if (userData.isEvent()) Firebase.printf("Event task: %s, msg: %s, code: %d\n", userData.uid().c_str(), userData.eventLog().message().c_str(), userData.eventLog().code());
    if (userData.isDebug()) Firebase.printf("Debug task: %s, msg: %s\n", userData.uid().c_str(), userData.debug().c_str());
    if (userData.isError()) Firebase.printf("Error task: %s, msg: %s, code: %d\n", userData.uid().c_str(), userData.error().message().c_str(), userData.error().code());

    // Deal with availible data.
    if(userData.available()) {
        RealtimeDatabaseResult &RTDB = userData.to<RealtimeDatabaseResult>();
        if (RTDB.isStream()) {
            Serial.println("----------------------------");
            Firebase.printf("task: %s\n", userData.uid().c_str());
            Firebase.printf("event: %s\n", RTDB.event().c_str());
            Firebase.printf("path: %s\n", RTDB.dataPath().c_str());
            Firebase.printf("data: %s\n", RTDB.to<const char *>());
            Firebase.printf("type: %d\n", RTDB.type());

            // The stream event from RealtimeDatabaseResult can be converted to the values as following.
            bool v1 = RTDB.to<bool>();
            int v2 = RTDB.to<int>();
            float v3 = RTDB.to<float>();
            double v4 = RTDB.to<double>();
            String v5 = RTDB.to<String>();
            
            String fieldPath = RTDB.dataPath().substring(1);
            Serial.println(fieldPath);
            
            if(fieldPath.equals(SL_CTRL_PATH)) {
                Serial.println("Controller Field Updated. Notify Later.");
                //xTaskNotify(rx_dpad_handle, -1, eNoAction);
            }
            else if(fieldPath.equals(SL_CONFIG_PATH)) {
                Serial.println("User Configuration Field Updated. Notify Later.");
                //xTaskNotify(rx_user_config_handle, -1, eNoAction);
            }
            else if(fieldPath.equals(SL_ACTIVE_PATH)) {
                Serial.println("User In App Field Updated. Notify Later.");
                //xTaskNotify(rx_user_config_handle, -1, eNoAction);
            }
            else Serial.println("Ummm. You're Cooked man.");
        }
        else {
            Serial.println("----------------------------");
            Firebase.printf("task: %s, payload: %s\n", userData.uid().c_str(), userData.c_str());
        }
        Firebase.printf("Free Heap: %d\n", ESP.getFreeHeap());
    }

    // Return nothing, the result of this method isn't used.
    return NULL;    
}

// Start the receiver.
void Receiver::begin() {
    initTasks();
}

// Create the tasks to receive user data.
void Receiver::beginUserDataRxTask() {
    xTaskCreatePinnedToCore(
        &rx_user_data_task,     // Pointer to task function.
        "rx_user_data_task",    // Task name.
        TASK_STACK_DEPTH,       // Size of stack allocated to the task (in bytes).
        this,                   // Pointer to parameters used for task creation.
        MAX_PRIORITY,           // Task priority level.
        &rx_user_data_handle,   // Pointer to task handle.
        1                       // Core that the task will run on.
    );
}

// Create all the necessary receiver tasks.
void Receiver::initTasks() {
    beginUserDataRxTask();
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
 * Decodes user configuration data.
 * @param userConfigData Data to be decoded.
 * @return A UserSentryConfig packet holding the decoded data.
 */
UserSentryConfig Receiver::decodeUserConfigurationData(uint64_t userConfigData) {

    // Decode.
    float temperatureLvl = (userConfigData >> SL_UserCfgInfo::TMP_LSB) & SL_UserCfgInfo::THN_MASK;
    float humidityLvl = (userConfigData >> SL_UserCfgInfo::HUM_LSB) & SL_UserCfgInfo::THN_MASK;
    float noiseLvl = (userConfigData >> SL_UserCfgInfo::SPL_LSB) & SL_UserCfgInfo::THN_MASK;
    float pressureLvl = (userConfigData >> SL_UserCfgInfo::PRS_LSB) & SL_UserCfgInfo::PRS_MASK;
    float aqiLvl = (userConfigData >> SL_UserCfgInfo::AQI_LSB) & SL_UserCfgInfo::AQI_MASK;

    // Fill the new configuration.
    UserSentryConfig configData;
    configData.userAirQualityIndexThreshold = aqiLvl;
    configData.userHumidityLevelThreshold = humidityLvl;
    configData.userNoiseLevelThreshold = noiseLvl;
    configData.userPressureLevelThreshold = pressureLvl;
    configData.userTemperatureLevelThreshold = temperatureLvl;

    // Return.
    return configData;
}

/**
 * Decodes user drive command data.
 * @param UserDriveCommandData Data to be decoded.
 * @return A UserDriveCommands packet holding the decoded data.
 */
UserDriveCommands Receiver::decodeUserDriveCommands(uint32_t UserDriveCommandData) {

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

    // Fill the new configuration.
    UserDriveCommands driveData;
    driveData.active = ctrlPower;
    driveData.usingDpad = usingDpad;
    driveData.usingJoystick = usingJoystick;
    driveData.dpad_Forward = dpadU;
    driveData.dpad_Backward = dpadD;
    driveData.dpad_Left = dpadL;
    driveData.dpad_Right = dpadR;
    driveData.joystick_X = joystickX;
    driveData.joystick_Y = joystickY;

    // Return.
    return driveData;
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