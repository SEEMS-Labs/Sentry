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

// Task info.   
#define TASK_STACK_DEPTH 6000   // Max stack size.
#define MAX_PRIORITY 10         // Max task priority.
#define MEDIUM_PRIORITY 5       // Medium task priority.
#define LOW_PRIORITY 1          // Low task priority.

#define F_US_ID ((SensorID) 0)  // Id for Sentry's front ultrasonic sensor.
#define B_US_ID ((SensorID) 1)  // Id for Sentry's back ultrasonic sensor.
#define L_US_ID ((SensorID) 2)  // Id for Sentry's left ultrasonic sensor.
#define R_US_ID ((SensorID) 3)  // Id for Sentry's right ultrasonic sensor.

#define F_US_READY ((NotificationMask) 0x0001)  // Front ultrasonic sensor notification.
#define B_US_READY ((NotificationMask) 0x0010)  // Back ultrasonic sensor notification.
#define L_US_READY ((NotificationMask) 0x0100)  // Left ultrasonic sensor notification.
#define R_US_READY ((NotificationMask) 0x1000)  // Right ultrasonic sensor notification.

#define TTR 40  // Time-to-read (a single ultrasonic sensor);
#define US_READ_TIME ((milliSeconds) pdMS_TO_TICKS(TTR))    // The maximum time it takes to read an ultrasonic sensor (in ms).
#define MAX_US_POLL_TIME ((4 * US_READ_TIME) + 10)          // The maximum time it takes to poll all 4 ultrasonic sensors w/ some buffer time.

extern TaskHandle_t poll_US_handle;                // Task handle for polling the ultrasonic sensors.
extern TaskHandle_t poll_mic_handle;               // Task handle for polling the microphone's analog output.
extern TaskHandle_t poll_bme_handle;               // Task handle for polling the BME688.

void poll_US_task(void *pvSensorManager);   // Polling ultrasonic sensor task.
void poll_mic_task(void *pvParameters);     // Polling microphone task.
void poll_bme_task(void *pvParameters);     // Polling BME688 task.

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
    public:
        SensorManager() : 
            frontUS(TRIG_F, ECHO_F, F_US_ID, DEF_F_OBS_LIM),
            backUS(TRIG_B, ECHO_B, B_US_ID, DEF_B_OBS_LIM), 
            leftUS(TRIG_L, ECHO_L, L_US_ID, DEF_L_OBS_LIM), 
            rightUS(TRIG_R, ECHO_R, R_US_ID, DEF_R_OBS_LIM),
            mic(MIC_ANALOG_OUT, DEF_MIC_LEVEL_LIM)  {};
        void initAllSensors();          // Initialize all 6 sensors of the Sentry.
        void attachAllInterrupts();     // Attach all sensor based interrupts of the Sentry.
        void beginAllTasks();           // Begin all sensor based tasks of the Sentry.

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
        HCSR04 *fetchUS(SensorID id);
    //************************************************************************************/

    //*****************************  Noise Sensor/ Mic  **********************************/
    private:
        Microphone mic;     // Noise sensor of the Sentry.
    
    public:

    //************************************************************************************/

    //***********************************  BME688  ***************************************/
    private:
        BME688 bme688;      // Environmental sensor of the Sentry.

    public:
    //************************************************************************************/

};


// End include gaurd.
#endif /* SensorManager.h */
