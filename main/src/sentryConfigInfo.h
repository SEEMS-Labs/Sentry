
// Include guard.
#ifndef SENTRY_CONFIG_INFO_H
#define SENTRY_CONFIG_INFO_H

#include <Arduino.h>

#define EE2_TEST_MODE 1     // To override certain function flows for testing purposes.
#define SERIAL_ONLY_MODE 0  // Mode to deal with things only via the serial terminal and no wi-fi.

// Serial Monitor Constants.
#define BAUD_RATE 115200
#define ONE_SECOND 1000

// Test Wifi. (Covered via BLE communication).
#define WIFI_SSID "SEEMS"
#define WIFI_PASSWORD "@Ucf2025"

// Test Firebase authentication. (To be covered via BLE Communication).
#define USER_EMAIL "es849112@ucf.edu"
#define USER_PASSWORD "rCpnKBR4ZhtefmL"

// Firebase RTDB Keys/Urls.
#define API_KEY "AIzaSyAGnxeU6342_TcjpmTKPT_WjB4AVTODfMk"
#define DATABASE_URL "https://seems-hub-default-rtdb.firebaseio.com/"

/*********************************************************
    SENSOR PIN DEFINITONS AND DEFAULT THRESHOLD LEVELS
**********************************************************/
// Ultrasonic Sensors.
#define TRIG_F 4           // Front HC-SR04 Trigger Pin.
#define ECHO_F 5           // Front HC-SR04 Echo Pin.
#define DEF_F_OBS_LIM 12    // Front HC-SR04 Obstacle Detection Threshold (in inches).

#define TRIG_B 14           // Back HC-SR04 Trigger Pin.
#define ECHO_B 13           // Back HC-SR04 Echo Pin.
#define DEF_B_OBS_LIM 12    // Back HC-SR04 Obstacle Detection Threshold (in inches).

#define TRIG_L 12           // Left HC-SR04 Trigger Pin.
#define ECHO_L 11           // Left HC-SR04 Echo Pin.
#define DEF_L_OBS_LIM 2     // Left HC-SR04 Obstacle Detection Threshold (in inches).

#define TRIG_R 10           // Right HC-SR04 Trigger Pin.
#define ECHO_R 9            // Right HC-SR04 Echo Pin.
#define DEF_R_OBS_LIM 2     // Right HC-SR04 Obstacle Detection Threshold (in inches).

// BME688 Environmental Sensor.
#define BME_SDI 1   // BME688 SDI (SDA) Pin for I2C.
#define BME_SCK 2   // BME688 SCK (SCL) Pin for I2C.

// Noise Sensor.
#define MIC_ANALOG_OUT 8        // Microphone Analog Output Pin.
#define DEF_MIC_LEVEL_LIM 50    // Microphone Noise Level Threshold (in dB).

// Motors and Motor Drivers.
#define L_ENC_A     18      // Left Motor Encoder Output A.
#define L_ENC_B     21      // Left Motor Encoder Output B.
#define L_MOT_EN    4       // Left Motor Driver Enable Input Pin.
#define L_MOT_PWM1  5       // Left Motor Driver Direction Input Pin.
#define L_MOT_PWM2  6       // Left Motor Driver PWM Input Pin.
#define L_MOT_DIAG  7       // Left Motor Driver Fault Output Pin.
#define L_MOT_OCM   17      // Left Motor Driver Current Sense Output Pin.
#define R_ENC_A     47      // Right Motor Encoder Output A.
#define R_ENC_B     48      // Right Motor Encoder Output B.
#define R_MOT_EN    39      // Right Motor Driver Enable Input Pin.
#define R_MOT_PWM1  40      // Right Motor Driver Direction Input Pin.
#define R_MOT_PWM2  41      // Right Motor Driver PWM Input Pin.
#define R_MOT_DIAG  42      // Right Motor Driver Fault Output Pin.
#define R_MOT_OCM   38      // Right Motor Driver Current Sense Output Pin.

/*********************************************************
                ALERT DEFAULT THRESHOLDS
**********************************************************/
#define DEF_AQI_LIM 151     // Default Air Quality Threshold (in AQI scale).
#define DEF_NOISE_LIM 80    // Default Noise Level Threshold (in dB).
#define DEF_TEMP_LIM 100    // Default Temperature Level Trheshold (in degrees C).
#define DEF_HUM_LIM 90      // Default Humidity Level Threshold (in relative humidity %).
#define DEF_PRES_LIM 1200   // Default Pressure Level Threshold (in hPa).
#define DEF_HP_EST_LIM 150  // Default Human Presence Estimation Threshold (in inches).
#define DEF_CO2_LIM 2500    // Default CO2 Level Threshold (in ppm).
#define DEF_VOC_LIM 20      // Default bVOC Level Threshold (in ppm).


/*********************************************************
            COMMUNICATION CONFIGURATION
**********************************************************/
#define FB_ENV_DATA_ADDRESS "readings/"
#define FB_ALERTS_ADDRESS   "alerts/"

#define AQ_ALERT_KEY "airQuality"
#define HUM_ALERT_KEY "humidity"
#define TMP_ALERT_KEY "temperature"
#define DB_SPL_ALERT_KEY "noise"
#define PRESSURE_ALERT_KEY "pressure"
#define MOTION_ALERT_KEY "presence"

#define AQ_DATA_KEY "airQuality"
#define HUM_DATA_KEY "humidity"
#define PRESSURE_DATA_KEY "pressure"
#define TMP_DATA_KEY "temperature"
#define DB_SPL_DATA_KEY "noise"
#define VOC_DATA_KEY "bvoc"
#define CO2_DATA_KEY "co2"

typedef float SensorReading;   // Represents sensor values. Data should never require more than 16-bits.
typedef bool Status;           // Represents alert status as a boolean.
const Status UNSAFE = false;
const Status SAFE = true;

// Data packet that holds sentry sensor data to be transmitted.
typedef struct _sensorData SensorData;
struct _sensorData {
    SensorReading airQualityIndex;
    SensorReading temperatureLevel;
    SensorReading humidityLevel;
    SensorReading pressureLevel;
    SensorReading noiseLevel;
    SensorReading bVOClevel;
    SensorReading CO2Level;
};

// Data packet that holds sentry alert data to be transmitted.
typedef struct _alertData Alerts;
struct _alertData {
    Status airQualityStatus;
    Status temperatureStatus;
    Status humidityStatus;
    Status pressureStatus;
    Status bVOCStatus;
    Status co2Status;
    Status noiseStatus;
    Status motion;
};

#define AQI_BREACHED_MASK           0x01
#define CO2_BREACHED_MASK           0x02
#define PRESSURE_BREACHED_MASK      0x04
#define VOC_BREACHED_MASK           0x08
#define TEMPERATURE_BREACHED_MASK   0x10
#define HUMIDITY_BREACHED_MASK      0x20

// Data packet that holds sentry obstacle detection data.
typedef struct _obstcleDetectionData Obstacles;
struct _obstcleDetectionData {
    Status frontObstacleDetected;
    Status backObstacleDetected;
    Status leftObstacleDetected;
    Status rightObstacleDetected;
};

// Represents different types of network connections which the Sentry can have.
enum class ConnectionType {
    ct_WIFI,
    ct_BT,
    ct_FB
};

#define FB_USER_ACTIVITY_STATUS_ADDRESS "user_in_app"
#define FB_USER_CONFIG_ADDRESS "user_config"
#define FB_USER_CONTROLLER_STATUS_ADDRESS "controller/active"
#define FB_USER_DPAD_ADDRESS "controller/dpad"
#define FB_USER_JOYSTICK_ADDRESS "controller/joystick"
#define FB_USING_DPAD_STATUS_ADDRESS "controller/using_dpad"
#define FB_USING_JOYSTICK_STATUS_ADDRESS "controller/using_joystick"

// Data packet that holds sentry configuration information transmitted from SentryLink.
typedef struct _userSentryConfig UserSentryConfig;
struct _userSentryConfig {
    SensorReading userAirQualityIndexThreshold;
    SensorReading userTemperatureLevelThreshold;
    SensorReading userHumidityLevelThreshold;
    SensorReading userPressureLevelThreshold;
    SensorReading userNoiseLevelThreshold;
};

// Data packet that holds sentry driving instructions from the user transmitted from SentryLink.
typedef struct _userDriveCommands UserDriveCommands;
struct _userDriveCommands {
    Status dpad_Forward;
    Status dpad_Backward;
    Status dpad_Left;
    Status dpad_Right;
    signed short int joystick_X;    // Expecting signed 16-bit x-coordinates.
    signed short int joystick_Y;    // Expecting signed 16-bit y-coordinates.
};

// Represents different types of data received from SentryLink.
enum class UserDataType {
    UDT_CONFIG,     // User configuration data.
    UDT_MVMT,       // User movement command data.
    UDT_FB_AUTH,    // User Firebase credentials (equivalent to SentryLink credentials).
    UDT_WIFI_AUTH,  // User Wi-Fi credentials.
};

#define SERVICE_UUID        "ab3b4f86-a60b-439f-98a0-ebb022b74550"
#define CHARACTERISTIC_UUID "e8f99c04-2c62-4660-bc38-30e488e1fd5d"

#define DEFAULT_SSID "xUCF"
#define DEFAULT_PASS "x2025"

#define BLE_INVALID_RX "???"

enum class BLETransmitCode : unsigned short {
    ble_tx_wait = 0x01,             // Code 1: Waiting for credentials. 
    ble_tx_creds_valid = 0x02,      // Code 2: Credentials received were valid.
    ble_tx_creds_invalid = 0x03,    // Code 3: Credentials received were invalid.
    ble_tx_data_valid = 0x04,       // Code 4: Transmission received was valid.
    ble_tx_data_invalid = 0x05      // Code 5: Transmission received was invalid.
};

/*********************************************************
                    Tasks Handles and Materials
**********************************************************/
#define TASK_STACK_DEPTH 16384  // Max stack size.
#define MAX_PRIORITY 10         // Max task priority.
#define MEDIUM_PRIORITY 5       // Medium task priority.
#define LOW_PRIORITY 1          // Low task priority.

extern TaskHandle_t poll_US_handle;     // Task handle for polling the ultrasonic sensors.
extern TaskHandle_t poll_mic_handle;    // Task handle for polling the microphone's analog output.
extern TaskHandle_t poll_bme_handle;    // Task handle for polling the BME688.

extern TaskHandle_t tx_bme_data_handle;
extern TaskHandle_t tx_mic_data_handle;
extern TaskHandle_t tx_alerts_handle;           // Task handle to transmit sentry alert data to firebase.
extern TaskHandle_t rx_user_data_handle;        // Task handle to receive user config and command data from firebase/bluetooth.
extern TaskHandle_t check_if_data_ready_handle; // Task handle to check periodically if user data is available.

extern TaskHandle_t move_sentry_handle;         // Task handle to control motors.
extern TaskHandle_t walk_algorithm_handle;      // Task handle to Enhance Random Walk algo.

extern SemaphoreHandle_t firebase_app_mutex;    // Semaphore Handle for mutex gaurding access to Firebase app for Transmission.

/*********************************************************
                Task Notification Values
**********************************************************/
typedef uint NotificationValue;     // 32-bit notification values for inter-task notification.
const NotificationValue OBSTACLE_THRESHOLD_BREACHED = 0x0001;   // Mask representing that the Obstacle Detection Threshold of an HCSR04 Sensor has been passed.
const NotificationValue PRESENCE_THRESHOLD_BREACHED = 0x0002;   // Mask representing that the Presence Detection Threshold of an HCSR04 Sensor has been passed.
const NotificationValue MIC_DATA_READY = 0x0001;                // Mask representing that the microphone was just read and its data is ready to be transmitted.
const NotificationValue BME_DATA_READY = 0x0002;                // Mask representing that the BME688 was just read and its data is ready to be transmitted.
const NotificationValue US_READY = 0x0003;              

/*********************************************************
                Sentry Operation States
**********************************************************/

enum class _power_states {
    ps_CHARGED,
    ps_LOW_BATTERY,
    ps_VERY_LOW_BATTERY,
    ps_CHARGING
};

enum class _wake_states {
    ws_ACTIVE,
    ws_STANDBY,
    ws_SHUTDOWN
};

enum class _data_transmission_states {
    ds_IDLE,
    ds_TRANSMIT,
    ds_RECEIVE
};

enum class _movement_states {
    ms_IDLE,
    ms_MANUAL,
    ms_AUTONOMOUS,
    ms_HOMING,
    ms_EMERGENCY_STOP
};

enum class _startup_states {
    ss_VALID,       // Representative of Sentry booting w/ valid stored network credentials.
    ss_INVALID      // Representative of Sentry booting w/ no or invalid stored network credentials.
};

enum class _network_connectivity_states {
    ns_NONE,        // Representative of Sentry having 0 network connections.
    ns_FB_AND_WF,   // Representative of Sentry being connected to Firebase and Wi-Fi.
    ns_WF_ONLY,     // Representative of Sentry being connected to Wi-Fi only.
    ns_BLE          // Representative of Sentry being connected to BLE Client only.
};

using MovementState = _movement_states;
using DataTransmissionState = _data_transmission_states;
using WakeState = _wake_states;
using PowerState = _power_states;
using StartupState = _startup_states;
using ConnectionState = _network_connectivity_states;


/*********************************************************
                Sentry Preferences (NVS Memory)
**********************************************************/
#define PREF_CREDS "credentials"
#define PREF_CREDS_SSID "SSID"
#define PREF_CREDS_PASS "PASS"

// End include gaurd.
#endif /*sentryConfigInfo.h*/