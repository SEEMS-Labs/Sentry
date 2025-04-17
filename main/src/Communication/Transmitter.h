
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

// Represents different tranmssion states.
enum class DataTransmissionType {
    tx_BME_DATA,
    tx_MIC_DATA,
    tx_ALERTS,
    tx_TEST
};

void tx_alerts_task(void *pvTransmitter);       // Transmit Sentry environmental alerts to Firebase.
void tx_bme_data_task(void *pvTransmitter);     // Transmit Sentry BME data readings to Firebase.
void tx_mic_data_task(void *pvTransmitter);     // Transmit Sentry Mic data readings to Firebase.

// Manages the Sentry's data tranmission to firebase. Maybe bluetooth later?
class Transmitter {

    private:    
        SensorData *envData;                // Pointer to Sensor Readings Data Packet. Globally available for updates.
        Alerts *envStatus;                  // Pointer to Sentry Alerts Data Packet. Globally availible for updates.
        StateManager *_stateManager;        // Sentry State manager.
        ConnectivityManager *_connManager;  // Sentry connectivty manager.

        JsonWriter jsonWriter;      // Json String Writer for transmission packets.
        object_t bmeTxBuffer[8];    // Transmission Buffer for BME data transmission.
        object_t alertTxBuffer[7];  // Transmission Buffer for alert transmission.
        float micTxBuffer;          // Transmission Buffer for BME data transmission.

        /**
         * Build and construct the BME environmental data packet for transmission.
         */
        void buildBmeDataTransmission();   

        /**
         * Build and construct the mic data packet for transmission.
         */
        void buildMicDataTransmisison();

        /**
         * Build and construct the alerts data packet for transmission.
         */
        void buildAlertsTransmission();  
        
        /**
         * Update the appropriate fields in firebase as decided by the parameters passed.
         * @param writeAddress Address of field to write to in Firebase.
         * @param dtt Type of tranmission being written to Firebase given that 
         */
        void updateFirebase(String writeAddress, DataTransmissionType dtt);

        // Initialize the tasks of the transmitter.
        void initTasks();
        BaseType_t beginBmeTxTask();
        BaseType_t beginMicTxTask();
        BaseType_t beginAlertsTxTask();

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
        void transmitSensorData(DataTransmissionType dtt);

        // Transmit only sentry alerts to Firebase.
        void transmitAlerts();

};

// End include guard.
#endif /* Transmitter.h */