#include "Receiver.h"

void Receiver::onConnect(BLEServer* pServer) {
    _stateManager->setSentryConnectionState(ConnectionState::ns_BLE);
}

void Receiver::onDisconnect(BLEServer *pServer) {
    _stateManager->setSentryConnectionState(ConnectionState::ns_NONE);
}

void Receiver::onWrite(BLECharacteristic *pCharacteristic) {
    // Grab the value written to the characteristic from the BLE Client.
    bleDataAvailable = true;
    String value = pCharacteristic->getValue();

    // Read the data. Update buffer.
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