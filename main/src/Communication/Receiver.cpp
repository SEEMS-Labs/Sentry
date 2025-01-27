#include "Receiver.h"

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

bool Receiver::checkIfDataAvailible(UserDataType udtData) {
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

String Receiver::receiveBLEData() {
    bleDataAvailable = false;
    String data = bleDataBuffer;
    bleDataBuffer = "";
    return data;
}