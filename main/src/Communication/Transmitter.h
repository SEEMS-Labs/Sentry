
// Include gaurd.
#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include "Sentry/main/src/sentryConfigInfo.h"
#include "Sentry/main/src/StateManager.h"
#pragma once
#include "ConnectivityManager.h"
#include <FirebaseClient.h>
#include <BLECharacteristic.h>

// Forward definition of ConnectivityManager.
class ConnectivityManager;

void tx_sensor_data_task(void *pvTransmitter);   // Transmit sensor data task.
void tx_alerts_task(void *pvTransmitter);        // Transit sentry alerts.
void tx_bme_data_task(void *pvTransmitter);
void tx_mic_data_task(void *pvTransmitter);

// Manages the Sentry's data tranmission to firebase. Maybe bluetooth later?
class Transmitter {

    private:    
        SensorData *envData;                // Sensor Readings Data Packet. Globally available for updates.
        Alerts *envStatus;                  // Sentry Alerts Data Packet. Globally availible for updates.
        StateManager *_stateManager;        // Sentry State manager.
        ConnectivityManager *_connManager;  // Sentry connectivty manager.

        // Per name.
        object_t buildSensorReadingsTransmission();  

        // Per name.
        object_t buildAlertsTransmission();  
        
        // Per name.
        void updateFirebase(String writeAddress, object_t jsonString);

        // Initialize the tasks of the transmitter.
        void initTasks();

    public:
        Transmitter(SensorData *envData, Alerts *envStatus, ConnectivityManager *_manager) : 
            envData(envData), 
            envStatus(envStatus),
            _connManager(_manager),
            _stateManager(StateManager::getManager()) {};
        
        // Start the transmitter tasks.
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

        void transmitMicData();
        void transmitDistanceData();
};

// End include guard.
#endif /* Transmitter.h */