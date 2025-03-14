// Include gaurd.
#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

// Grab Sentry related headers.
#include "Sentry/main/src/sentryConfigInfo.h"
#include "HCSR04/HCSR04.h"
#include "Microphone/mic.h"
#include "BME688/BME688.h"

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

void poll_US_task(void *pvSensorManager);   // Polling ultrasonic sensor task.
void poll_mic_task(void *pvSensorManager);     // Polling microphone task.
void poll_bme_task(void *pvSensorManager);  // Polling BME688 task.

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
        Alerts *alertInfo;               // Packet of important Sentry alert info for SentryLink.
        Obstacles obstacleInfo;         // Packet of obstacle detection data for navigation.
        SensorData *environmentalData;   // Packet of important environmental sensor readings for SentryLink.

    public:
        /**
         * Create the main Sensor Manager.
         */
        SensorManager(SensorData *envData, Alerts *envStatus) : 
            frontUS(TRIG_F, ECHO_F, F_US_ID, DEF_F_OBS_LIM),
            backUS(TRIG_B, ECHO_B, B_US_ID, DEF_B_OBS_LIM), 
            leftUS(TRIG_L, ECHO_L, L_US_ID, DEF_L_OBS_LIM), 
            rightUS(TRIG_R, ECHO_R, R_US_ID, DEF_R_OBS_LIM),
            bme688(BME_SDI, BME_SCK),
            mic(MIC_ANALOG_OUT),
            alertInfo(envStatus),
            environmentalData(envData)  {};

        void initAllSensors();          // Initialize all 6 sensors of the Sentry.
        void attachAllInterrupts();     // Attach all sensor based interrupts of the Sentry.
        void createSemaphores();        // Create all sensor based semaphores.
        void beginAllTasks();           // Begin all sensor based tasks of the Sentry.
        Alerts *getAlertsPacket();
        Obstacles *getObstaclesPacket();
        SensorData *getEnvironmentalDataPacket();

    //************************************************************************************/
    
    //*****************************  Ultrasonic Sensors  *********************************/
    private:
        HCSR04 frontUS;     // Front-facing ultrasonic sensor of the Sentry.
        HCSR04 backUS;      // Rear-facing ultrasonic sensor of the Sentry.
        HCSR04 leftUS;      // Left-facing ultrasonic sensor of the Sentry.
        HCSR04 rightUS;     // Right-facing ultrasonic sensor of the Sentry.

        float isrPulseDuration = -1;      // Stores the duration of the pulse captured by ISR.
        unsigned long isrPulseStart = -1; // Stores the time at which the sensor's echo has begun from ISR.
        unsigned long isrPulseEnd = -1;   // Stores the time at which the sensor's echo has finished from ISR.    

    public:
        void beginReadUltrasonicTask();
        HCSR04 *fetchUS(SensorID id);
    //************************************************************************************/

    //*****************************  Noise Sensor/ Mic  **********************************/
    private:
        Microphone mic;     // Noise sensor of the Sentry.
    
    public:
        void beginReadMicrophoneTask();
        Microphone *fetchMic();
        
    //************************************************************************************/

    //***********************************  BME688  ***************************************/
    private:
        BME688 bme688;      // Environmental sensor of the Sentry.

    public:
        void beginReadBMETask();
        BME688 *fetchBME();
    //************************************************************************************/

};


// End include gaurd.
#endif /* SensorManager.h */
