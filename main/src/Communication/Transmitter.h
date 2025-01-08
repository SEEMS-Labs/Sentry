
// Include gaurd.
#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include "Sentry/main/src/sentryConfigInfo.h"
#include "Sentry/main/src/StateManager.h"
#include <BLECharacteristic.h>

void tx_sensor_data_task(void *pvParameters);   // Transmit sensor data task.
void tx_alerts_task(void *pvParameters);        // Transit sentry alerts.

// Manages the Sentry's data tranmission to firebase. Maybe bluetooth later?
class Transmitter {

    private:    
        SensorData &envData;            // Sensor Readings Data Packet. Globally available for updates.
        Alerts &envStatus;              // Sentry Alerts Data Packet. Globally availible for updates.
        StateManager *_stateManager;    // Sentry State manager.

        // Per name.
        void updateSensorReadingsInFirebase();  

        // Per name.
        void updateAlertsInFirebase();  

        // Initialize the tasks of the transmitter.
        void initTasks();

    public:
        Transmitter(SensorData &envData, Alerts &envStatus) : 
            envData(envData), 
            envStatus(envStatus),
            _stateManager(StateManager::getManager()) {};
        
        // Start the Transmitter.
        void begin();

        /**
         * Transmit information only through the BLE Server to any client listening.
         * @param tx_code Transmission code to transmit to the BLE client. 
         * @param characteristic The specific BLE Characteristic on the server to be altered by the Sentry.
         */
        void transmitBLE(BLETransmitCode tx_code, BLECharacteristic *characteristic);

        // Transmit only environmental data to Firebase.
        void transmitSensorData();

        // Transmit only sentry alerts to Firebase.
        void transmitAlerts();
};

// End include guard.
#endif /* Transmitter.h */