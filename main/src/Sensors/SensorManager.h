// Include gaurd.
#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

// Grab required headers.
#include "Sentry/main/src/sentryConfigInfo.h"
#pragma once
#include "HCSR04/HCSR04.h"

#pragma once
#include "Microphone/Microphone.h"

#pragma once
#include "BME688/BME688.h"
#include <Preferences.h>

// Forward definition of Sensors.
class HCSR04;
class Microphone;
class BME688;

// Some quality of life type definitions.
typedef int SensorID;               // Sensor Id.
typedef uint32_t NotificationMask;  // Mask to delineate between Notifcations.
typedef uint32_t milliSeconds;      // Milliseconds.

// Internal Task info.   
#define F_US_ID ((SensorID) 0)  // Id for Sentry's front ultrasonic sensor.
#define B_US_ID ((SensorID) 1)  // Id for Sentry's back ultrasonic sensor.
#define L_US_ID ((SensorID) 2)  // Id for Sentry's left ultrasonic sensor.
#define R_US_ID ((SensorID) 3)  // Id for Sentry's right ultrasonic sensor.

#define F_US_READY ((NotificationMask) 0x0001)  // Front ultrasonic sensor notification.
#define B_US_READY ((NotificationMask) 0x0010)  // Back ultrasonic sensor notification.
#define L_US_READY ((NotificationMask) 0x0100)  // Left ultrasonic sensor notification.
#define R_US_READY ((NotificationMask) 0x1000)  // Right ultrasonic sensor notification.

#define TTR_US 40  // Time-to-read a single ultrasonic sensor (in milliseconds).
#define US_READ_TIME ((milliSeconds) pdMS_TO_TICKS(TTR_US))     // The maximum time it takes to read an ultrasonic sensor (in ticks).
#define MAX_US_POLL_TIME ((4 * US_READ_TIME) + 10)              // The delay between polling all 4 ultrasonic sensors w/ some buffer time.

#define TTR_BME 3000  // Time-to-read the BME688 (in milliseconds).
#define BME_READ_TIME ((milliSeconds) pdMS_TO_TICKS(TTR_BME))       // The maximum time it takes to read the BME688 (in ticks).
#define MAX_BME_POLL_TIME 6000                                      // The delay between each new polling of the BME688.

void update_thresholds_task(void *pvSensorManager); // Updating sensor thresholds task.
void poll_US_task(void *pvSensorManager);           // Polling ultrasonic sensor task.
void poll_mic_task(void *pvSensorManager);          // Polling microphone task.
void poll_bme_task(void *pvSensorManager);          // Polling BME688 task.

void IRAM_ATTR on_front_us_echo_changed(void *arg);  // ISR that deals with timing of front ultrasonic sensor's trigger pulse. Arg is a ref to sensor in question.
void IRAM_ATTR on_back_us_echo_changed(void *arg);   // ISR that deals with timing of back ultrasonic sensor's trigger pulse. Arg is a ref to sensor in question.
void IRAM_ATTR on_left_us_echo_changed(void *arg);   // ISR that deals with timing of left ultrasonic sensor's trigger pulse. Arg is a ref to sensor in question.
void IRAM_ATTR on_right_us_echo_changed(void *arg);  // ISR that deals with timing of right ultrasonic sensor's trigger pulse. Arg is a ref to sensor in question.

/**
 * Class designed to manage Sentry sensors and sensor tasks.
 */
class SensorManager {

    //*****************************  General Management  *********************************/
    private:
        Alerts *alertInfo;                  // Pointer to Packet of important Sentry alert info for SentryLink.
        ObstacleData *obstacleInfo;         // Pointer to Packet of obstacle detection data for navigation.
        UserSentryConfig *userConfig;       // Pointer to Packet of sensor threshold information for updates.
        SensorData *environmentalData;      // Pointer to Packet of important environmental sensor readings for SentryLink.
        StateManager *stateManager;         // Pointer to the Sentry's State Manager.
        Preferences preferences;            // Access to the Sentry's Permanent Memory.
        void constructAllSensors();

    public:
        /**
         * Create the main Sensor Manager.
         */
        SensorManager(SensorData *envData, Alerts *envStatus, UserSentryConfig *userConfig, ObstacleData *obstacleInfo) : 
            stateManager(StateManager::getManager()),
            alertInfo(envStatus),
            environmentalData(envData),
            userConfig(userConfig),
            obstacleInfo(obstacleInfo)
        {
            constructAllSensors();
        };

        void initAllSensors();          // Initialize all 6 sensors of the Sentry.
        void attachAllInterrupts();     // Attach all sensor based interrupts of the Sentry.
        void beginAllTasks();           // Begin all sensor based tasks of the Sentry.
        Alerts *getAlertsPacket();
        ObstacleData *getObstaclesPacket();
        SensorData *getEnvironmentalDataPacket();
        UserSentryConfig *getUserSentryConfigDataPacket(); 
        void updateSensorThresholds();
        BaseType_t beginUpdateThresholdTask();

    //************************************************************************************/
    
    //*****************************  Ultrasonic Sensors  *********************************/
    private:
        HCSR04 *frontUS;     // Front-facing ultrasonic sensor of the Sentry.
        HCSR04 *backUS;      // Rear-facing ultrasonic sensor of the Sentry.
        HCSR04 *leftUS;      // Left-facing ultrasonic sensor of the Sentry.
        HCSR04 *rightUS;     // Right-facing ultrasonic sensor of the Sentry.

        float isrPulseDuration = -1;      // Stores the duration of the pulse captured by ISR.
        unsigned long isrPulseStart = -1; // Stores the time at which the sensor's echo has begun from ISR.
        unsigned long isrPulseEnd = -1;   // Stores the time at which the sensor's echo has finished from ISR.    

    public:
        void initUS();
        BaseType_t beginReadUltrasonicTask();
        HCSR04 *fetchUS(SensorID id);
    //************************************************************************************/

    //*****************************  Noise Sensor/ Mic  **********************************/
    private:
        Microphone *mic;     // Noise sensor of the Sentry.
    
    public:
        void initMic();
        BaseType_t beginReadMicrophoneTask();
        Microphone *fetchMic();
        
    //************************************************************************************/

    //***********************************  BME688  ***************************************/
    private:
        BME688 *bme688;      // Environmental sensor of the Sentry.

    public:
        void initBME();
        BaseType_t beginReadBMETask();
        BME688 *fetchBME();
    //************************************************************************************/

};


// End include gaurd.
#endif /* SensorManager.h */
