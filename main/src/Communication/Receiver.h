
// Include gaurd.
#ifndef RECEIVER_H
#define RECEIVER_H

#include "Sentry/main/src/sentryConfigInfo.h"
#include "Sentry/main/src/StateManager.h"
#include <BLEServer.h>
#include <BLECharacteristic.h>

class Receiver : public BLEServerCallbacks, public BLECharacteristicCallbacks {

    private:
        UserSentryConfig &userConfiguration;        // Holds custom user configuration from SentryLink. Globally available to be read.
        UserDriveCommands &userMovementCommands;    // Holds user movement commands given from SentryLink. Globally availible to be read.
        StateManager *_stateManager;                // Sentry State manager.

        bool bleDataAvailable = false;              // Represents if the BLE Server has data available from client.
        String bleDataBuffer = "";                  // Buffer of data received from BLE client.

        bool isConfigDataAvailable();
        bool areDriveCommandsAvailible();
        void updateUserSentryConfig(UserSentryConfig *config);
        void updateUserDriveCommands(UserDriveCommands *commands);

    public:
        Receiver(UserSentryConfig &userConfiguration, UserDriveCommands &userMovementCommands) : 
            userConfiguration(userConfiguration), 
            userMovementCommands(userMovementCommands),
            _stateManager(StateManager::getManager()) {};

        void begin();
        bool checkIfUserInApp();
        bool checkIfDataAvailible(UserDataType udtData);
        void receiveConfigData();
        void receiveDriveCommandData();
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
};

#endif /* Receiver.h */