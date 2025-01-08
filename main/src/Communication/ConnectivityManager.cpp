#include "ConnectivityManager.h"

void ConnectivityManager::initFirebase() {

}

void ConnectivityManager::initWiFi() {
    
}

void ConnectivityManager::initSentryBLEServer() {

    // Set BLE active.
    isBLEServerActive = true;
}

void ConnectivityManager::deinitFirebase() {

}

void ConnectivityManager::deinitWiFi() {

}

/**
 * Check the ESP32 preferences to see if the user has previously set Wi-Fi
 * credentials. Optionally, if Wi-Fi credentials do exist upon checking, they 
 * will be set within this method.
 * @return True if Wi-Fi credentials were previously set, false otherwise.
 */
bool ConnectivityManager::checkForCredentials() {

    // Create preferences namespace in read mode.
    bool credsExist = false;
    preferences.begin("credentials", true);
    
    // Check for intial exisitence of SSID (if this doesn't exist, this is the first Sentry init).
    bool isInitialBoot = !preferences.isKey("SSID");
    if(isInitialBoot) credsExist = false;

    // Check if the credentials are valid in every non-initial boot.
    else {
        String name = preferences.getString("SSID", DEFAULT_SSID);
        String password = preferences.getString("password", DEFAULT_PASS);
        credsExist = (name != DEFAULT_SSID) && (password != DEFAULT_PASS);

        // Set credentials for future use should they exist.
        if(credsExist) setWiFiCredentials(ssid, password);
    }

    // End the preferences namespace and return.
    preferences.end();
    return credsExist;
}

void ConnectivityManager::deinitSentryBLEServer() {

}

bool ConnectivityManager::checkCredentialValidity() {
    WiFi.mode(WIFI_MODE_STA);
    WiFi.begin(ssid, password);
    bool successStatus = WiFi.waitForConnectResult() == WL_CONNECTED;
    WiFi.disconnect(true, true);
    return successStatus;
}

/**
 * Starts the Sentry up w/o credentials. This method should configure the 
 * Sentry as a BLE Server and then wait for connection from SentryLink where 
 * it will receive the appropriate user Wi-Fi credentials. Once these have been 
 * received, the BLE Server should be shut down and credentials should be saved 
 * to this Managers relevant memebrs as well as the appropriate space for Wi-Fi
 * credentials in the ESP32's Preferences library.
 */
void ConnectivityManager::onCredentialessStartup() {

    // Reset Wi-Fi credentials if they were deemed invalid.
    if(_stateManager->getSentryStartupState() == StartupState::ss_INVALID) {
        this->ssid = DEFAULT_SSID;
        this->password = DEFAULT_PASS;
    }

    // Configure BLE server. 
    connect(ConnectionType::ct_BT);
    _transmitter.transmitBLE(BLETransmitCode::ble_tx_wait, characteristic);
    
    // Wait until server is closed and credentials are deemed valid.
    bool readyToDisconnect = false;
    bool bleDataAvailable = false;
    String dataReceived = "";
    String dataSanitized = "";
    for(;;) {

        // Check if any information is ready to be read from the receiver.
        bleDataAvailable = _receiver.checkIfDataAvailible(UserDataType::UDT_WIFI_AUTH);
        if(bleDataAvailable) {
            dataReceived = _receiver.receiveBLEData();
            vTaskDelay(pdMS_TO_TICKS(500));    // Small delay for BLE Stack (?).

            // Indicate that the data received was of valid length.
            if(dataReceived != BLE_INVALID_RX) {
                _transmitter.transmitBLE(BLETransmitCode::ble_tx_data_valid, characteristic);
                
                // Identify SSID vs Password.
                dataSanitized = dataReceived.substring(1);
                if(dataReceived[0] == '0') ssid = dataSanitized;
                else password = dataSanitized;
            }

            // Indicate that the data received was of invalid length.
            else _transmitter.transmitBLE(BLETransmitCode::ble_tx_data_invalid, characteristic);
        }

        // Check credential validity.
        if(ssid != DEFAULT_SSID && password != DEFAULT_PASS) {
            readyToDisconnect = checkCredentialValidity();

            // Inform Sentry Link that credentials were valid. Leave loop.
            if(readyToDisconnect) {
                _transmitter.transmitBLE(BLETransmitCode::ble_tx_creds_valid, characteristic);
                break;
            }

            // Reset manager credentials and inform SentryLink of invalid credentials.
            else {
                ssid = DEFAULT_SSID;
                password = DEFAULT_PASS;
                _transmitter.transmitBLE(BLETransmitCode::ble_tx_creds_invalid, characteristic);
            }
        } 

        // Pause for 1 second to wait for a valid wi-fi connection. Overhead for BLE Stack.
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // End the BLE Server and begin startup w/ new valid credentials.
    disconnect(ConnectionType::ct_BT);
    onCredentialedStartup();
}

void ConnectivityManager::onCredentialedStartup() {

    // Connect wi-Fi and Firebase.
    connect(ConnectionType::ct_WIFI);
    connect(ConnectionType::CT_FB);
}

void ConnectivityManager::setWiFiCredentials(String ssid, String password) {
    this->ssid = ssid;
    this->password = password;
}

void ConnectivityManager::begin() {

    // Grab credentials.
    bool credentialsExist = checkForCredentials();

    // Startup w/ credentials.
    if(credentialsExist) {        
        // Ensure credentials are valid.
        bool credsValid = checkCredentialValidity();
        if(credsValid) {
            // Set startup state. Connect.
            _stateManager->setSentryStartupState(StartupState::ss_VALID);
            onCredentialedStartup();
            return;
        }
    }
    
    // Startup w/o credentials or w/ invalid credentials.
    if(credentialsExist) _stateManager->setSentryStartupState(StartupState::ss_INVALID);
    else _stateManager->setSentryStartupState(StartupState::ss_PREMIER);
    onCredentialessStartup();

}

void ConnectivityManager::connect(ConnectionType cType) {
    switch (cType) {
        case ConnectionType::CT_FB :
            initFirebase();
            break;
        case ConnectionType::ct_WIFI :
            initWiFi();
            break;
        case ConnectionType::ct_BT :
            initSentryBLEServer();
            break;
    }
}

void ConnectivityManager::disconnect(ConnectionType cType) {
    switch (cType) {
        case ConnectionType::CT_FB :
            deinitFirebase();
            break;
        case ConnectionType::ct_WIFI :
            deinitWiFi();
            break;
        case ConnectionType::ct_BT :
            deinitSentryBLEServer();
            break;
    }
}

bool ConnectivityManager::isConnected(ConnectionType cType) {
    bool res = false;
    switch (cType) {
        case ConnectionType::CT_FB      :   res = isFirebaseConnected;
        case ConnectionType::ct_WIFI    :   res = isWiFiConnected;
        case ConnectionType::ct_BT      :   res = isBluetoothConnected;
    }
    return res;
}