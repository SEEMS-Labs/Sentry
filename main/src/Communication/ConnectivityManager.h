
// Include gaurd.
#ifndef CONNECTIVITYMANAGER_H
#define CONNECTIVITYMANAGER_H

#include "Sentry/main/src/sentryConfigInfo.h"
#include "Sentry/main/src/StateManager.h"
#pragma once
#include "Transmitter.h"
#include "Receiver.h"
#include <FirebaseClient.h>
#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2901.h>
#include <BLE2902.h>
#include <Preferences.h>

// Forward definition of Transmitter.
class Transmitter;

class ConnectivityManager {

private:
    // General members.
    bool isWiFiConnected = false;      // The Sentry's Wi-Fi connection status.
    bool isBluetoothConnected = false; // The Sentry's BLE Connection status.
    bool isBLEServerActive = false;    // The Sentry BLE Server activation status.
    bool isFirebaseConnected = false;  // The Sentry's Firebase connection status.
    Transmitter *_transmitter;         // The transmitter arm of this connectivity manager.
    Receiver _receiver;                // The receiver arm of this connectivity manager.

    // Wi-Fi + Firebase members.
    String ssid = (EE2_TEST_MODE) ? WIFI_SSID : DEFAULT_SSID;         // User Wi-Fi SSID.
    String password = (EE2_TEST_MODE) ? WIFI_PASSWORD : DEFAULT_PASS; // User Wi-Fi password.
    UserAuth userAuth;
    FirebaseApp _fbApp;
    RealtimeDatabase _rtdb;
    WiFiClientSecure sslClient;
    AsyncClientClass aClient;
    AsyncResult aResultNoCallback;

    // Bluetooth members.
    BLEServer *server = NULL;
    BLEService *service = NULL;
    BLECharacteristic *characteristic = NULL;
    BLEAdvertising *advertising = NULL;
    BLEAdvertising *advertiser = NULL;

    // Permanent memory + State manager access.
    Preferences preferences;
    StateManager *_stateManager;

    // Transmitter.
    void initTransmitter(SensorData *envData, Alerts *envStatus);

    // Connectivity management.
    void initFirebase();
    void deinitFirebase();
    void fbAuthHandler();
    void fbLogResult(AsyncResult &aResult);
    void fpPrintError(int code, String msg);

    void initWiFi();
    void deinitWiFi();

    void initSentryBLEServer();
    void deinitSentryBLEServer();

    // Wi-Fi credential management.
    bool checkForCredentials();
    bool checkCredentialValidity();
    void updatePreferredCredentials();
    void onCredentialessStartup();
    void onCredentialedStartup();
    void setWiFiCredentials(String ssid, String password);

public:
    ConnectivityManager(
        SensorData *envData,                                                                           // Pointer to the global environmental data packet.
        Alerts *envStatus,                                                                             // Pointer to the global alerts data packet.
        UserSentryConfig &userConfiguration,                                                           // Address to the global custom user sentry configuration data packet.
        UserDriveCommands &userMovementCommands) :                                                     // Address to the global user movement commands data packet.
                                                   _receiver(userConfiguration, userMovementCommands), // Construct the receiver object.
                                                   userAuth(API_KEY, USER_EMAIL, USER_PASSWORD),       // Construct a garbage UserAuth that will be later reset. Required becaue C++ init list.
                                                   _fbApp(),                                           // Construct the Firebase App object.
                                                   _rtdb(),                                            // construct the Real Time Database object.
                                                   sslClient(),                                        // Construct SSL Client object.
                                                   aResultNoCallback(),                                // Construct the AsyncResult object.
                                                   aClient(sslClient),                                 // Construct the Async Result Object.
                                                   _stateManager(StateManager::getManager())           // Grab the instance of the state manager singleton.
    {
        // Initialze data reading buffer.
        envData->airQualityIndex = -1.0;
        envData->humidityLevel = -1.0;
        envData->noiseLevel = -1.0;
        envData->pressureLevel = -1.0;
        envData->temperatureLevel = -1.0;
        envData->bVOClevel = -1;
        envData->CO2Level = -1;

        // Initialze alerts reading buffer.
        envStatus->airQualityStatus = false;
        envStatus->humidityStatus = false;
        envStatus->noiseStatus = false;
        envStatus->pressureStatus = false;
        envStatus->temperatureStatus = false;

        initTransmitter(envData, envStatus); // Construct pointer to the transmitter object.
    }

    void begin();
    void connect(ConnectionType cType);
    void disconnect(ConnectionType cType);
    bool isConnected(ConnectionType cType);

    RealtimeDatabase getFirebaseDatabase();
    AsyncClientClass getAsyncClient();
    FirebaseApp getFbApp();
};

// End include gaurd.
#endif