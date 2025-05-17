
// Include gaurd.
#ifndef CONNECTIVITYMANAGER_H
#define CONNECTIVITYMANAGER_H

#include "Sentry/main/src/sentryConfigInfo.h"
#include "Sentry/main/src/StateManager.h"
#include "EspNowNode.h"

#pragma once
#include "Transmitter.h"
#pragma once
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

// Forward definition of Transmitter and Receiver.
class Transmitter;
class Receiver;

/**
 * The Sentry's Wi-Fi, Bluetooth, and Firebase Connection Manager.
 */
class ConnectivityManager {

private:
    // General members.
    bool isWiFiConnected = false;       // The Sentry's Wi-Fi connection status.
    bool isBluetoothConnected = false;  // The Sentry's BLE Connection status.
    bool isBLEServerActive = false;     // The Sentry BLE Server activation status.
    bool isFirebaseConnected = false;   // The Sentry's Firebase connection status.
    inline static bool cameraIpAcquired = false;
    inline static String cameraIp;
    Transmitter *_transmitter;          // The transmitter arm of this connectivity manager.
    Receiver *_receiver;                // The receiver arm of this connectivity manager.
    EspNowNode *sentryEspNowNode;       // The connection mechanism between Sentry and SentryCam.

    // Wi-Fi + Firebase members.
    String ssid = (EE2_TEST_MODE) ? WIFI_SSID_t : DEFAULT_SSID;         // User Wi-Fi SSID.
    String password = (EE2_TEST_MODE) ? WIFI_PASSWORD_t : DEFAULT_PASS; // User Wi-Fi password.
    UserAuth userAuth;
    FirebaseApp _fbApp;
    RealtimeDatabase _rtdb;
    WiFiClientSecure sslClient, sslStreamClient;
    AsyncClientClass aClient, aStreamClient;
    AsyncResult aResultNoCallback, streamResult;

    // Bluetooth members.
    BLEServer *server = NULL;
    BLEService *service = NULL;
    BLECharacteristic *characteristic = NULL;
    BLEAdvertising *advertising = NULL;
    BLEAdvertising *advertiser = NULL;

    // Permanent memory + State manager access.
    Preferences preferences;
    StateManager *_stateManager;

    // Transmitter and Receiver.
    void createFirebaseSemaphores();
    void constructTransmitter(SensorData *envData, Alerts *envStatus);
    void constructReceiver(UserSentryConfig *userConfiguration, UserDriveCommands *userMovementCommands);

    // Connectivity management.
    void initFirebase();
    void deinitFirebase();
    static void auth_debug_print(AsyncResult &aResult);

    void initWiFi();
    void deinitWiFi();

    void initSentryBLEServer();
    void deinitSentryBLEServer();

    void initSentryCamIpRetrieval();
    void deinitSentryCamIpRetrieval();
    void constructEspNowNode(const uint8_t* peerMacAddress, bool masterMode);
    static BaseType_t cameraIpCallBack(const char *ip);

    // Wi-Fi credential management.
    bool checkForCredentials();
    bool checkCredentialValidity();
    void updatePreferredCredentials();
    void onCredentialessStartup();
    void onCredentialedStartup();
    void setWiFiCredentials(String ssid, String password);

public:
    ConnectivityManager(
        SensorData *envData,                                                                            // Pointer to the global environmental data packet.
        Alerts *envStatus,                                                                              // Pointer to the global alerts data packet.
        UserSentryConfig *userConfiguration,                                                            // Pointer to the global custom user sentry configuration data packet.
        UserDriveCommands *userMovementCommands) :                                                      // Pointer to the global user movement commands data packet.
                                                userAuth(API_KEY, USER_EMAIL, USER_PASSWORD, 3000),     // Construct a garbage UserAuth that will be later reset. Required becaue C++ init list.
                                                _fbApp(),                                               // Construct the Firebase App object.
                                                _rtdb(),                                                // construct the Real Time Database object.
                                                sslClient(),                                            // Construct SSL Client object.
                                                sslStreamClient(),                                      // Construct SSL Stream Client object.
                                                aResultNoCallback(),                                    // Construct the AsyncResult object.
                                                streamResult(),                                         // Construct the Stream AsyncResult object.
                                                aClient(sslClient),                                     // Construct the AsyncClient Object.
                                                aStreamClient(sslStreamClient),                         // Construct the Stream Async Client Object.
                                                _stateManager(StateManager::getManager())               // Grab the instance of the state manager singleton.
    {
        constructTransmitter(envData, envStatus);
        constructReceiver(userConfiguration, userMovementCommands);
        constructEspNowNode(dev_Cam_C, true);
    }

    void begin();
    void connect(ConnectionType cType);
    void disconnect(ConnectionType cType);
    bool isConnected(ConnectionType cType);

    RealtimeDatabase *getFirebaseDatabase();
    AsyncClientClass *getAsyncClient();
    FirebaseApp *getFbApp();
    AsyncResult getSentryLinkStreamResult();
    Transmitter *getTransmitter();
    Receiver *getReceiver();
};

// End include gaurd.
#endif