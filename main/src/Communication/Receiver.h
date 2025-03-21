
// Include gaurd.
#ifndef RECEIVER_H
#define RECEIVER_H

#include "Sentry/main/src/sentryConfigInfo.h"
#include "Sentry/main/src/StateManager.h"
#pragma once
#include "ConnectivityManager.h"
#include <FirebaseClient.h>
#include <BLEServer.h>
#include <BLECharacteristic.h>

// Forward definition of ConnectivityManager.
class ConnectivityManager;

void rx_user_data_task(void *pvReceiver);   // Receive user data from SentryLink via Firebase.

/**
 * The Receiver of the Sentry. This manages data reception from both Bluetooth and Firebase.
 */
class Receiver : public BLEServerCallbacks, public BLECharacteristicCallbacks {

    private:
        UserSentryConfig *userConfiguration;        // Pointer to custom user configuration from SentryLink. Globally available to be read.
        UserDriveCommands *userMovementCommands;    // Pointer to user movement commands given from SentryLink. Globally availible to be read.
        StateManager *_stateManager;                // Sentry State manager.
        ConnectivityManager *_connManager;          // Sentry connectivty manager.

        // BLE Reception material.
        bool bleDataAvailable = false;              // Represents if the BLE Server has data available from client.
        String bleDataBuffer = "";                  // Buffer of data received from BLE client.

        // Firebase Reception material.
        String lastReadData = "";
        String lastReadId = "";
        bool checkForRepeatedResult(AsyncResult res);
        void receiveSentryLinkStream(AsyncResult &userData);
        UserDataType getDataTypeReceived(String dataPathReceieved);
        void decodeAndUpdateUserConfigurationData(uint64_t userConfigData);
        void decodeAndUpdateUserDriveCommands(uint32_t UserDriveCommandData);
        
        // Initialize the tasks of the receiver.
        void initTasks();
        BaseType_t beginUserDataRxTask();

    public:
        Receiver(UserSentryConfig *userConfiguration, UserDriveCommands *userMovementCommands, ConnectivityManager *_manager) : 
            userConfiguration(userConfiguration), 
            userMovementCommands(userMovementCommands),
            _connManager(_manager),
            _stateManager(StateManager::getManager()) {};
        
        /** 
         * Start the receiver.
        */
        void begin();

        /**
         * Check to see if BLE data is ready.
         */
        bool checkIfBLEDataAvailible(UserDataType udtData);

        /**
         * Method to retrieve the data sent to the Sentry over BLE stored
         * in the receivers buffers.
         */
        String receiveBLEData();
        
        /**
         * Overridden method to be called on connection to SentryBLEServer.
         * @param pServer - Pointer to the SentryBLEServer;
         */
        void onConnect(BLEServer *pServer) override;

        /**
         * Overridden method to be called on disconnection to SentryBLEServer.
         * @param pServer - Pointer to the SentryBLEServer;
         */
        void onDisconnect(BLEServer *pServer) override;

        /**
         * Overridden method to be called on write to SentryBLEServer Characteristic.
         * @param pServer - Pointer to the SentryBLEServer;
         */
        void onWrite(BLECharacteristic *pCharacteristic) override;

        void receiveSentryLinkUserData();

};

#endif /* Receiver.h */