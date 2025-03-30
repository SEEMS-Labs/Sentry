
// Include guard.
#ifndef SENTRY_CONFIG_INFO_H
#define SENTRY_CONFIG_INFO_H

#include <Arduino.h>

#define EE2_TEST_MODE 1     // To override certain function flows for testing purposes.
#define SERIAL_ONLY_MODE 0  // Mode to deal with things only via the serial terminal and no wi-fi.
#define MVMT_ACTIVE 0       // Mode to deal with movement being used.
#define USE_HPE_DEF_THD 1   // Mode to use default hpe threshold in code, not on esp32 memory.

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
#define MIC_ANALOG_OUT 6        // Microphone Analog Output Pin.
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
#define R_MOT_OCM   10      // Right Motor Driver Current Sense Output Pin.

/*********************************************************
                ALERT THRESHOLDS
**********************************************************/
#define DEF_AQI_LIM 151     // Default Air Quality Threshold (in AQI scale).
#define DEF_NOISE_LIM 80    // Default Noise Level Threshold (in dB).
#define DEF_TEMP_LIM 100    // Default Temperature Level Trheshold (in degrees C).
#define DEF_HUM_LIM 90      // Default Humidity Level Threshold (in relative humidity %).
#define DEF_PRES_LIM 1200   // Default Pressure Level Threshold (in hPa).
#define DEF_HP_EST_LIM 36  // Default Human Presence Estimation Threshold (in inches).
//#define DEF_CO2_LIM 2500    // Default CO2 Level Threshold (in ppm).
//#define DEF_VOC_LIM 20      // Default bVOC Level Threshold (in ppm).

#define MAX_AQI 500
#define MIN_AQI 50

#define MAX_NOISE 120
#define MIN_NOISE 80

#define MAX_TEMP 85     // In degrees Celsius.
#define MIN_TEMP -40

#define MAX_HPE 150     // Inches.
#define MIN_HPE 24      

#define MAX_HUM 100     // Percent.
#define MIN_HUM 0

#define MAX_PRES 1100   // hPa.
#define MIN_PRES 300    

/*********************************************************
                MOTOR DEFAULTS
**********************************************************/
#define DEFAULT_SPEED 200

/*********************************************************
            COMMUNICATION CONFIGURATION
**********************************************************/
#define FB_CONN_TIMEOUT_PERIOD 15000 

#define FB_SENTRY_CONN      ((String) "sentry/sentry_conn")
#define FB_ENV_DATA_ADDRESS ((String) "sentry/readings/")
#define FB_ALERTS_ADDRESS   ((String) "sentry/alerts/")

#define SENTRYLINK_ROOT ((String) "sentrylink/")
#define SL_CTRL_PATH ((String) "controller")
#define SL_CONFIG_PATH ((String) "user_config")
#define SL_ACTIVE_PATH ((String) "user_in_app")

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

#define THD_ALERT 0xFF          // Defualt return value of threshold checking methods.

typedef float SensorReading;    // Represents sensor values. Data should never require more than 16-bits.
typedef bool Status;            // Represents alert status as a boolean.
const Status UNSAFE = true;     // This means that an alert has been triggered on a specific parameter. 
const Status SAFE = false;      // This means that an alert has not been triggered on a specific parameter.

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
typedef struct _obstcleDetectionData ObstacleData;
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
    SensorReading userCO2LevelThreshold;
    SensorReading userVOCLevelThreshold;
    SensorReading userNoiseLevelThreshold;
    SensorReading userPresenceEstimationThreshold;
};

/**
 * Access information for the sentry link controller path being read in firebase.
 */
enum SL_ctrlInfo {
    JOYSTICK_MASK =  0x02FF,    // Mask for retrieving Joystick coordinate information from the Sentrylink controller address.
    CD_STA_MASK = 0x0003,       // Mask for retrieving Dpad and Controller state from the Sentrylink controller address.
    ACTIVE_MASK = 0x0001,       // Mask for retrieving Controller active status from the Sentrylink controller address.
    JSTK_Y_LSB = 15,            // LSB of the Joystick Y Coordinates field within Sentrylink controller address.
    JSTK_X_LSB = 5,             // LSB of the Joystick X Coordinates field within Sentrylink controller address.
    DPAD_STA_LSB = 2,           // LSB of the Dpad field within Sentrylink controller address.
    CTRL_STA_LSB = 1,           // LSB of the controller configuration field within Sentrylink controller address.
    ACTIVE_LSB = 0,             // LSB of the controller activity field within Sentrylink controller address.
    DPAD_L = 0,                 // Decimal value of Dpad Field if Left is selected.
    DPAD_R = 1,                 // Decimal value of Dpad Field if Right is selected.
    DPAD_U = 2,                 // Decimal value of Dpad Field if Up is selected.
    DPAD_D = 3,                 // Decimal value of Dpad Field if Down is selected.
    DPAD_ACTIVE = 1,            // Decimal value of controller state field if Dpad is selected.
    JSTK_ACTIVE = 2             // Decimal value of controller state if joystick is selected.
};

// Data packet that holds sentry driving instructions from the user transmitted from SentryLink.
typedef struct _userDriveCommands UserDriveCommands;
struct _userDriveCommands {

    // Controller active state.
    bool active;

    // Field that informs the Sentry if its Dpad is selected.
    bool usingDpad;  
    
    // Field that informs the Sentry if its Joystick is selected.
    bool usingJoystick;      
    
    // Dpad "button" states. Only 1 state can be active (true) at a time.
    bool dpad_Forward;
    bool dpad_Backward;
    bool dpad_Left;
    bool dpad_Right;

    /**
     * Joystick coordinates.
     */
    signed short int joystick_X;    // Expecting signed 10-bit x-coordinates.
    signed short int joystick_Y;    // Expecting signed 10-bit y-coordinates.
};

/**
 * Access information for the sentry link user configuration path being read in firebase.
 */
enum SL_UserCfgInfo {
    THN_MASK = 0x007F,      // Mask for retrieving Temp/Hum/Noise thresholds from the Sentrylink user configuration address.
    PRS_MASK = 0x07FF,      // Mask for retrieving Pressure thresholds from the Sentrylink user configuration address.
    AQI_HPE_MASK = 0x01FF,  // Mask for retrieving Air Quality and Human Presence Estimation thresholds from the Sentrylink user configuration address.
    TMP_LSB = 0,            // LSB of the Temperature threshold field within Sentrylink user configuration address.
    HUM_LSB = 7,            // LSB of the Humidity threshold field within Sentrylink user configuration address.
    SPL_LSB = 14,           // LSB of the Noise threshold field within Sentrylink user configuration address.
    HPE_LSB = 21,           // LSB of the Human Presence Estimation threshold field within Sentrylink user configuration address.
    AQI_LSB = 30,           // LSB of the Air Quality threshold field within Sentrylink user configuration address.
    PRS_LSB = 39            // LSB of the Pressure threshold field within Sentrylink user configuration address.
};

// Represents different types of data received from SentryLink.
enum class UserDataType {
    UDT_ACTIVITY,   // User in-ap activity status.
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
extern SemaphoreHandle_t user_config_mutex;    
extern TaskHandle_t update_thresholds_handle;   // Task Handle for updating the sensor thresholds to user specifications.
extern TaskHandle_t poll_US_handle;             // Task handle for polling the ultrasonic sensors.
extern TaskHandle_t poll_mic_handle;            // Task handle for polling the microphone's analog output.
extern TaskHandle_t poll_bme_handle;            // Task handle for polling the BME688.

extern SemaphoreHandle_t firebase_app_mutex;    // Semaphore Handle for mutex gaurding access to Firebase app for Transmission.
extern TaskHandle_t tx_bme_data_handle;         // Task handle to transmit sentry BME688 readings to firebase.
extern TaskHandle_t tx_mic_data_handle;         // Task handle to transmit sentry Mic readings to firebase.
extern TaskHandle_t tx_alerts_handle;           // Task handle to transmit sentry alert data to firebase.
extern TaskHandle_t rx_user_data_handle;        // Task handle to receive user config and command data from firebase/bluetooth.

extern TaskHandle_t user_ctrld_mvmt_handle;     // Task handle to control motors via user direction.
extern TaskHandle_t erw_mvmt_handle;            // Task handle to control motors via Enhanced Random Walk algo.
extern TaskHandle_t monitor_ocm_handle;         // Task handle to read the current monitoring pins of the drive system.
extern TaskHandle_t monitor_diag_handle;        // Task handle to read the diagnostic pins of the drive system.


// Size of the stack allocated on the heap to a task (in bytes).
enum TaskStackDepth {
    tsd_POLL = 6000,        // Size given to tasks who read sensors.
    tsd_SET = 5000,         // Size given to tasks who simply set values.
    tsd_TRANSMIT = 6000,    // Size given to tasks who transmit information to firebase.
    tsd_RECEIVE = 8000,     // Size given to tasks who receive information from firebase.
    tsd_DRIVE = 10000,      // Size given to tasks who drive the Sentry's Locomotion.
    tsd_MAX = 16384         // Maximum size given to a task.
};

// Priority level of a task.
enum TaskPriorityLevel {
    tpl_LOW = 1,
    tpl_MEDIUM_LOW = 5,
    tpl_MEDIUM = 10,
    tpl_MEDIUM_HIGH = 15,
    tpl_HIGH = 20
};

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

enum class _sensor_threshold_states {
    ts_PRE_STARTUP,     // Representative of Sentry not yet having examined sensor threshold preferences.
    ts_POST_STARTUP     // Reperesentative of Sentry already having examined sensor threshold preferences.
};

using MovementState = _movement_states;
using DataTransmissionState = _data_transmission_states;
using WakeState = _wake_states;
using PowerState = _power_states;
using StartupState = _startup_states;
using ConnectionState = _network_connectivity_states;
using ThresholdState = _sensor_threshold_states;

/*********************************************************
                Sentry Preferences (NVS Memory)
**********************************************************/
#define PREF_CREDS "credentials"    // Wi-Fi crediential namespace.
#define PREF_CREDS_SSID "SSID"      // Wi-Fi SSID key.
#define PREF_CREDS_PASS "PASS"      // Wi-Fi Password key.

#define PREF_SENSOR_THDS "SENSOR_THDS"  // Sensor threshold namespace.
#define PREF_IAQ_THD "IAQ_THD"          // IAQ threshold key.
#define PREF_CO2_THD "CO2_THD"          // CO2 threshold key.
#define PREF_PRES_THD "PRS_THD"         // Pressure threshold key.
#define PREF_VOC_THD "VOC_THD"          // VOC threshold key.
#define PREF_TEMP_THD "TMP_THD"         // Temperature threshold key.
#define PREF_HUM_THD "HUM_THD"          // Humidity threshold key.
#define PREF_SPL_THD "SPL_THD"          // DeciBel SPL threshold key.
#define PREF_HPE_THD "HPE_THD"          // Human Presence Estismation threshold key.

// End include gaurd.
#endif /*sentryConfigInfo.h*/