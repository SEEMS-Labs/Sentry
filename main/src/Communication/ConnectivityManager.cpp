#include "ConnectivityManager.h"

// Define Semaphore handles.
SemaphoreHandle_t firebase_app_mutex = NULL;

/**
 * Check the ESP32 preferences to see if the user has previously set Wi-Fi
 * credentials. Optionally, if Wi-Fi credentials do exist upon checking, they 
 * will be set within this method.
 * @return True if Wi-Fi credentials were previously set, false otherwise.
 */
bool ConnectivityManager::checkForCredentials() {
    Serial.println("Checking if Credentials Exist.");

    // Override normal process if testing w/ static wi-fi creds.
    bool credsExist = false;
    if(EE2_TEST_MODE && (ssid != DEFAULT_SSID) && (password != DEFAULT_PASS)) {
        return true;    // True = good credentials. Only override if your credentials are guaranteed.
    }

    // Create preferences namespace in read mode.
    taskENTER_CRITICAL(&preferencesMutex);
    preferences.begin(PREF_CREDS, true);
    
    // Check for intial exisitence of SSID (if this doesn't exist, this is the first Sentry init).
    bool isInitialBoot = !preferences.isKey(PREF_CREDS_SSID);
    if(isInitialBoot) credsExist = false;

    // Check if the credentials are valid in every non-initial boot.
    else {
        String id = preferences.getString(PREF_CREDS_SSID, DEFAULT_SSID);
        String pass = preferences.getString(PREF_CREDS_PASS, DEFAULT_PASS);
        credsExist = (id != DEFAULT_SSID) && (pass != DEFAULT_PASS);

        // Set credentials for future use should they exist.
        if(credsExist) setWiFiCredentials(id, pass);
    }
    
    if(!credsExist) Serial.println("Credentials did not exist.");
    // End the preferences namespace and return.
    preferences.end();
    taskEXIT_CRITICAL(&preferencesMutex);
    return credsExist;
}

/**
 * Ensure that credentials as saved in the manager's "Ssid" and "password"
 * fields by temporarily starting the Sentry's Wi-Fi mode and attempting to 
 * connect using the stored credentials. 
 * @return True if the stored Wi-Fi credentials were able to establish a succseful
 * Wi-Fi connection, false otherwise.
 */
bool ConnectivityManager::checkCredentialValidity() {

    Serial.println("Checking credential validity.");

    // Don't bother checking if no credentials are present.
    bool successStatus = false;
    if(ssid == DEFAULT_SSID && password == DEFAULT_PASS) successStatus = false;

    // Begin checking credentials that were present.
    else {
        WiFi.mode(WIFI_MODE_STA);
        WiFi.begin(ssid, password);
        delay(100);
        successStatus = WiFi.waitForConnectResult() == WL_CONNECTED;
        WiFi.disconnect(true, true);
        delay(1000);
        WiFi.mode(WIFI_MODE_NULL);
    }

    if(successStatus == true) Serial.println("Valid Credentials.");
    else Serial.println("Invalid Credentials.");

    // Return.
    return successStatus;
}

/**
 * Set the Wi-Fi credentials of this Connectivity Manager for use in connecting to Wi-Fi.
 * @param ssid Wi-Fi network name.
 * @param password Wi-Fi network password.
 */
void ConnectivityManager::setWiFiCredentials(String ssid, String password) {
    Serial.println("Credentials existed. Setting credentials");
    Serial.printf("SSID: %s, PASS: %s\n", ssid, password);
    this->ssid = ssid;
    this->password = password;
}

/**
 * Start the Wi-Fi, Bluetooh, and Firbase network connections
 * necessary for the Sentry's communication system.
 */
void ConnectivityManager::begin() {

    // Check for Credentials and, if they exist, their validity.
    bool properCredentialsSaved = checkForCredentials() && checkCredentialValidity();

    // Begin credentialed startup process.
    if(properCredentialsSaved) {
        Serial.println("Proper Credentials Found in Memory. Beginning Credentialed Startup.");
        _stateManager->setSentryStartupState(StartupState::ss_VALID);
        onCredentialedStartup();
        Serial.println("Credentialed startup completed.");
    }

    // Begin credentialess startup process.
    else {
        Serial.println("No or Improper Credentials Found in Memory. Beginning Credentialess Startup.");
        _stateManager->setSentryStartupState(StartupState::ss_INVALID);
        onCredentialessStartup();
        Serial.println("Credentialess startup completed.");
    }

    // Start the transmitter and receiver.
    _transmitter->begin();
    _receiver->begin();
}

/**
 * Sentry startup process to be used when valid Wi-Fi credentials were
 * found store in system memory. Initializes Wi-Fi and Firebase connections.
 */
void ConnectivityManager::onCredentialedStartup() {

    // Transmit cam ip.
    if(CAMERA_ON) {
        connect(ConnectionType::ct_CAM);
        disconnect(ConnectionType::ct_CAM);
    }
    
    // Connect wi-Fi and Firebase.
    connect(ConnectionType::ct_WIFI);
    connect(ConnectionType::ct_FB);
}

/**
 * Starts the Sentry up w/o credentials. This method should configure the 
 * Sentry as a BLE Server and then wait for connection from SentryLink where 
 * it will receive the appropriate user Wi-Fi credentials. Once these have been 
 * received, the BLE Server should be shut down and credentials should be saved 
 * to this Managers relevant members as well as the appropriate space dedicated 
 * to storing Wi-Fi credentials in the ESP32's Preferences library.
 */
void ConnectivityManager::onCredentialessStartup() {

    // Reset Wi-Fi credentials since they were deemed invalid.
    this->ssid = DEFAULT_SSID;
    this->password = DEFAULT_PASS;
    
    // Configure BLE server. 
    connect(ConnectionType::ct_BT);
    
    // Wait until server is closed and credentials are deemed valid.
    bool readyToDisconnect = false;
    bool bleDataAvailable = false;
    String dataReceived = "";
    String dataSanitized = "";
    for(;;) {
        
        // Wait for sentry to be connected.
        while(1) {
            // Don't wait if credentials have already been updated.
            if(readyToDisconnect) break;

            // End wait if state transition from NONE to BLE occurs.
            else {
                Serial.println("Waiting for BLE Client connection.");
                if(_stateManager->getSentryConnectionState() == ConnectionState::ns_BLE) {
                    Serial.println("--BLE Active: Connected to BLE Client.");
                    _transmitter->transmitBLE(BLETransmitCode::ble_tx_wait, characteristic); // Tx waiting on connection.
                    isBluetoothConnected = true;
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        // Wait for credentials.
        while(_stateManager->getSentryConnectionState() == ConnectionState::ns_BLE && !readyToDisconnect) {
            Serial.println("Waiting for Network Credentials.");
            // Check if any information is ready to be read from the receiver.
            bleDataAvailable = _receiver->checkIfBLEDataAvailible(UserDataType::UDT_WIFI_AUTH);
            if(bleDataAvailable && !readyToDisconnect) {
                // Read the data.
                dataReceived = _receiver->receiveBLEData();
                Serial.println("Data Availible!");
                vTaskDelay(pdMS_TO_TICKS(500));    // Small delay for BLE Stack before updating it.

                // Parse through data if valid. Indicate that the data received was of valid length.
                if(dataReceived != BLE_INVALID_RX) {
                    // Identify SSID vs Password.
                    dataSanitized = dataReceived.substring(1);

                    Serial.printf("Received data transmission: %s -> %s\n", dataReceived.c_str(), dataSanitized.c_str());

                    if(dataReceived[0] != '0' && dataReceived[0] != '1') _transmitter->transmitBLE(BLETransmitCode::ble_tx_data_invalid, characteristic);
                    else {
                        _transmitter->transmitBLE(BLETransmitCode::ble_tx_data_valid, characteristic);
                        if(dataReceived[0] == '0') ssid = dataSanitized;
                        else if(dataReceived[0] == '1') password = dataSanitized;
                    } 
                }

                // Indicate that the data received was of invalid length.
                else _transmitter->transmitBLE(BLETransmitCode::ble_tx_data_invalid, characteristic);
            }
            
            // Check credential validity if credentials have both been set.
            if(ssid != DEFAULT_SSID && password != DEFAULT_PASS && !readyToDisconnect) {
                Serial.println("Credentials now fully available! Checking Validity!");
                readyToDisconnect = checkCredentialValidity();

                // Inform Sentry Link that credentials were valid. Leave loop.
                if(readyToDisconnect) {
                    Serial.println("Credentials VALID! Sending Signal to disconnect!");
                    _transmitter->transmitBLE(BLETransmitCode::ble_tx_creds_valid, characteristic);
                    break;
                }

                // Reset manager credentials and inform SentryLink of invalid credentials.
                else {
                    Serial.println("Credentials INVALID! Sending Signal to retry!");
                    ssid = DEFAULT_SSID;
                    password = DEFAULT_PASS;
                    _transmitter->transmitBLE(BLETransmitCode::ble_tx_creds_invalid, characteristic);
                }
            }

            // Yield to BLE and Wi-Fi Tasks.
            vTaskDelay(pdMS_TO_TICKS(1000));
        } 

        // Wait for disconnection after credentials received.
        if(_stateManager->getSentryConnectionState() == ConnectionState::ns_BLE && readyToDisconnect) {
            
            while(_stateManager->getSentryConnectionState() == ConnectionState::ns_BLE) {
                Serial.println("Waiting for Client to disconnect.");
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            
            // Process completed. Update credentials and close BLE Server.
            if(_stateManager->getSentryConnectionState() == ConnectionState::ns_NONE) {
                isBluetoothConnected = false;
                Serial.println("--BLE Client Disconnected. Updating credentials and closing BLE Server.");
                updatePreferredCredentials();
                break;
            }
        }

        // Deal w/ errant disconnections made in the midst of data transmission.
        if(_stateManager->getSentryConnectionState() == ConnectionState::ns_NONE && !readyToDisconnect) {
            // Process incomplete, Disconnection was unexpected. Restart process.
            Serial.printf("--BLE Active: Client disconnected w/o sending proper credentials, Try Again...\n");
            ssid = DEFAULT_SSID;
            password = DEFAULT_PASS;
            server->getAdvertising()->start();
        }

        // Pause for 1 second to wait for a valid wi-fi connection. Overhead for BLE Stack.
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // End the BLE Server begin startup w/ new valid credentials.
    disconnect(ConnectionType::ct_BT);
    onCredentialedStartup();
}

/**
 * Takes credentials tendered from SentryLink and uses them to update
 * those stored in the ESP32 Preferences in order to ensure seamless 
 * future credentialed startups.
 */
void ConnectivityManager::updatePreferredCredentials() {
    // Create preferences namespace in read/write mode.
    taskENTER_CRITICAL(&preferencesMutex);
    preferences.begin(PREF_CREDS, false);  

    // update credentials. 
    preferences.putString(PREF_CREDS_SSID, ssid);
    preferences.putString(PREF_CREDS_PASS, password);

    String name = preferences.getString(PREF_CREDS_SSID, DEFAULT_SSID);
    String password = preferences.getString(PREF_CREDS_PASS, DEFAULT_PASS);

    Serial.printf("Newly Stored Creds: SSID = %s, PASS = %s\n", name.c_str(), password.c_str());
    // End preferences namespace.
    preferences.end();
    taskEXIT_CRITICAL(&preferencesMutex);
}

/**
 * Starts the connection from the Sentry to 1 of the 3 different 
 * network types needed for Sentry operation.
 * @param cType The type of connection to be initiated.
 */
void ConnectivityManager::connect(ConnectionType cType) {
    switch (cType) {
        case ConnectionType::ct_FB :
            Serial.println("Beginning Connection to Firebase!");
            createFirebaseSemaphores();
            initFirebase();
            Serial.println("Firebase Initialized!");
            break;
        case ConnectionType::ct_WIFI :
            Serial.println("Beginning Connection to Wi-Fi!");
            initWiFi();
            Serial.println("Wi-Fi Initialized!");
            break;
        case ConnectionType::ct_BT :
            Serial.println("Beginning Init of BLE Server!");
            initSentryBLEServer();
            Serial.println("BLE Server Initialized!");
            break;
        case ConnectionType::ct_CAM :
            Serial.println("Beginning Connection to Sentry Cam for Exchange of Ip.");
            initSentryCamIpRetrieval();
            Serial.println("Connection to SentryCam process complete");
            break;
    }
}

/**
 * Ends the connection between the Sentry and 1 of the 3 different 
 * network types needed for Sentry operation.
 * @param cType The type of connection to be concluded.
 */
void ConnectivityManager::disconnect(ConnectionType cType) {
    switch (cType) {
        case ConnectionType::ct_FB :
            deinitFirebase();
            break;
        case ConnectionType::ct_WIFI :
            deinitWiFi();
            break;
        case ConnectionType::ct_BT :
            deinitSentryBLEServer();
            break;
        case ConnectionType::ct_CAM : 
            deinitSentryCamIpRetrieval();
            break;
    }
}

/**
 * Checks the connection status between the Sentry and 1 of the 
 * 3 different network types needed for Sentry operation.
 * @param cType The type of connection to be assessed.
 */
bool ConnectivityManager::isConnected(ConnectionType cType) {
    bool res = false;
    switch (cType) {
        case ConnectionType::ct_FB      :   res = isFirebaseConnected;
        case ConnectionType::ct_WIFI    :   res = isWiFiConnected;
        case ConnectionType::ct_BT      :   res = isBluetoothConnected;
    }
    return res;
}

/**
 * Initialize the Sentry's Firebase connection. Should only be called after 
 * succcesful connection to Wi-Fi.
 */
void ConnectivityManager::initFirebase() {
    Firebase.printf("Firebase Client v%s\n", FIREBASE_CLIENT_VERSION);
    Serial.println("Initializing app..."); 

    // Client for transmitter.
    sslClient.setInsecure();

    // Client for receiver.
    sslStreamClient.setInsecure();

    initializeApp(aClient, _fbApp, getAuth(userAuth), auth_debug_print, "AuthTask");

    // Binding the FirebaseApp for authentication handler. To unbind, use Database.resetApp();
    _fbApp.getApp<RealtimeDatabase>(_rtdb);
    _rtdb.url(DATABASE_URL);

    // Setup the firebase stream for the reciever to watch.
    aStreamClient.setSSEFilters("get,put,patch,keep-alive,cancel,auth_revoked");
    _rtdb.get(aStreamClient, SENTRYLINK_ROOT, streamResult, true);

    // Attempt to connect to Firebase with a timeout.
    Serial.println("Check for succesful Sentry Connection to Firebase.");
    ulong startTime = millis();
    while (!_fbApp.ready() && (millis() - startTime) < FB_CONN_TIMEOUT_PERIOD) {
        //Serial.println("Waiting for Firebase to be ready...");
        _fbApp.loop();
        delay(1); // Give Firebase more time to stabilize.
    }
    ulong endTime = millis();

    // Show the connection status and timed value.
    if((endTime - startTime) < FB_CONN_TIMEOUT_PERIOD && _fbApp.ready()) Serial.printf("Connection to Firebase took: %lu ms\n", endTime - startTime);
    else Serial.println("Connection to Firebase timed out.");
    
    // Deal with non connection state.
    if(_fbApp.ready() == false) {
        Serial.println("Unsuccesful connection. Timeout reached....");
        while(true) {
            Serial.println("Press Reset Button and Try again Until a better solution is found out.");
            delay(1000);
        }
    }

    // Set the "sentry_conn" field in Firebase to notify the partner app of successful connection.
    else {
        bool cam_ip_status = _rtdb.set<String>(aClient, FB_SENTRY_CAM_IP, cameraIp);
        bool status = _rtdb.set<bool>(aClient, FB_SENTRY_CONN, true);
        if(_fbApp.ready() && status && cam_ip_status) {
            Serial.println("Connection Good. Sentry Firebase status succesfully updated.");
            isFirebaseConnected = true;
        }
        else Serial.println("Connection Bad. Sentry Firebase status not updated.");
    }
    
    // Update Connectivity state.
    _stateManager->setSentryConnectionState(ConnectionState::ns_FB_AND_WF);
}

/**
 * Initialize the Sentry's Wi-Fi connection. Should only be called after 
 * the succcesful validation of Wi-Fi credentials.
 */
void ConnectivityManager::initWiFi() {
    WiFi.begin(ssid, password);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(300);
    }
    isWiFiConnected = true;
    Serial.printf("\nConnected W/ IP: %s\n", WiFi.localIP().toString().c_str());   

    // Update Connection State.
    _stateManager->setSentryConnectionState(ConnectionState::ns_WF_ONLY);
}

/**
 * Initialize the Sentry's BLE server. Should only be called after 
 * the confirmation of the presence of invalid Wi-Fi credentials.
 */
void ConnectivityManager::initSentryBLEServer() {

    // Don't attempt to start an already active BLE Server.
    if(isBLEServerActive == true) return;
    isBLEServerActive = true;

    // Initialize server, service, and characteristic.
    BLEDevice::init("SEEMS-Sentry");
    server = BLEDevice::createServer();
    service = server->createService(SERVICE_UUID);
    characteristic = service->createCharacteristic(
        CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | 
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_INDICATE
    );

    // Add characteristic descriptors.
    characteristic->addDescriptor(new BLE2902()); // Client characteristic configuration descriptor (CCCD).
    BLE2901 *userDescription = new BLE2901();
    userDescription->setDescription("Receive User Networking Configuration Information (Wi-Fi).");  // custom descriptor.
    userDescription->setAccessPermissions(ESP_GATT_PERM_READ);  // Read only.
    characteristic->addDescriptor(userDescription);

    // Set initial characteristic value.
    characteristic->setValue("Sentry Ready.");

    // Set callbacks for server and characteristic.
    server->setCallbacks(_receiver);
    characteristic->setCallbacks(_receiver);

    // Create and modify advertiser.
    advertiser = BLEDevice::getAdvertising();
    advertiser->addServiceUUID(SERVICE_UUID);
    advertiser->setScanResponse(false);

    // Start the servive and advertising.
    service->start();
    server->getAdvertising()->start();
}

/**
 * Deinitialize the Sentry's connection to Firebase.
 */
void ConnectivityManager::deinitFirebase() {

}

/**
 * Deinitialize the Sentry's Wi-FI connection.
 */
void ConnectivityManager::deinitWiFi() {

}

/**
 * Deinitialize the Sentry's BLE Server.
 */
void ConnectivityManager::deinitSentryBLEServer() {
    // Don't attempt to stop an already stopped BLE server.
    if(isBLEServerActive == false) return;
    isBLEServerActive = false;

    // Stop advertising and service.
    server->getAdvertising()->stop();
    service->stop();
    BLEDevice::deinit(true);
    delay(1000);
}

void ConnectivityManager::auth_debug_print(AsyncResult &aResult) {
    if (aResult.isEvent())
    {
        Firebase.printf("Event task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.eventLog().message().c_str(), aResult.eventLog().code());
    }

    if (aResult.isDebug())
    {
        Firebase.printf("Debug task: %s, msg: %s\n", aResult.uid().c_str(), aResult.debug().c_str());
    }

    if (aResult.isError())
    {
        Firebase.printf("Error task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.error().message().c_str(), aResult.error().code());
    }
}

/**
 * Create the mutex that guards the firebase app used in transmission.
 */
void ConnectivityManager::createFirebaseSemaphores() {
    firebase_app_mutex = xSemaphoreCreateMutex();
}

/**
 * Construct pointer to the transmitter. This is done seperately from the initialzer list
 * because of Circular Inclusion between the transmitter and this manager.
 */
void ConnectivityManager::constructTransmitter(SensorData *envData, Alerts *envStatus) {
    _transmitter = new Transmitter(envData, envStatus, this);   // Construct pointer to the transmitter object.
}

void ConnectivityManager::constructReceiver(UserSentryConfig *userConfiguration, UserDriveCommands *userMovementCommands) {
    _receiver = new Receiver(userConfiguration, userMovementCommands, this);
}

/**
 * Return pointer to the Firebase object.
 */
RealtimeDatabase *ConnectivityManager::getFirebaseDatabase() { return &_rtdb; }

/**
 * Return pointer to the Asynchonous Client object.
 */
AsyncClientClass *ConnectivityManager::getAsyncClient() { return &aClient; }

/**
 * Return pointer to the Firebase App object.
 */
FirebaseApp *ConnectivityManager::getFbApp() { return &_fbApp; }

/**
 * Return pointer to the stream result that holds information transmitted by the Sentry.
 */
AsyncResult ConnectivityManager::getSentryLinkStreamResult() {
    return streamResult;
}

Transmitter *ConnectivityManager::getTransmitter() { return _transmitter; }
Receiver *ConnectivityManager::getReceiver() { return _receiver; }

void ConnectivityManager::initSentryCamIpRetrieval() {
    sentryEspNowNode->addInfoToSend(ssid.c_str(), password.c_str());
    sentryEspNowNode->registerProcessCameraIPCallBack(cameraIpCallBack);
    sentryEspNowNode->start();
    while(sentryEspNowNode->credentialsPassedThrough() != true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void ConnectivityManager::deinitSentryCamIpRetrieval() {
    free(sentryEspNowNode);
}

void ConnectivityManager::constructEspNowNode(const uint8_t* peerMacAddress, bool masterMode) {
    sentryEspNowNode = new EspNowNode(peerMacAddress, true);
}

BaseType_t ConnectivityManager::cameraIpCallBack(const char *ip) {
    cameraIpAcquired = true;
    cameraIp = String(ip);   
    return pdPASS;
}
