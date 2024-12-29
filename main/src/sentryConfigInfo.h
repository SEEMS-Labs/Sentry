
// Include guard.
#ifndef SENTRY_CONFIG_INFO_H
#define SENTRY_CONFIG_INFO_H

// Serial Monitor Constants.
#define BAUD_RATE 115200
#define ONE_SECOND 1000

// KC Wifi.
//#define WIFI_SSID "WhiteSky-KnightsCircle"
//#define WIFI_PASSWORD "d9wk6xfg"

// Jax Wifi.
#define WIFI_SSID "NETGEAR66"
#define WIFI_PASSWORD "dynamicbug829"

// Firebase authentication.
#define USER_EMAIL "es849112@ucf.edu"
#define USER_PASSWORD "rCpnKBR4ZhtefmL"

// Firebase RTDB Keys/Urls.
#define API_KEY "AIzaSyAGnxeU6342_TcjpmTKPT_WjB4AVTODfMk"
#define DATABASE_URL "https://seems-hub-default-rtdb.firebaseio.com/"

/*********************************************************
    SENSOR PIN DEFINITONS AND DEFAULT THRESHOLD LEVELS
**********************************************************/
// Ultrasonic Sensors.
#define TRIG_F 16           // Front HC-SR04 Trigger Pin.
#define ECHO_F 15           // Front HC-SR04 Echo Pin.
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
#define L_MOT_DIR   5       // Left Motor Driver Direction Input Pin.
#define L_MOT_PWM   6       // Left Motor Driver PWM Input Pin.
#define L_MOT_DIAG  7       // Left Motor Driver Fault Output Pin.
#define L_MOT_OCC   17      // Left Motor Driver Current Sense Output Pin.
#define R_ENC_A     47      // Right Motor Encoder Output A.
#define R_ENC_B     48      // Right Motor Encoder Output B.
#define R_MOT_EN    39      // Right Motor Driver Enable Input Pin.
#define R_MOT_DIR   40      // Right Motor Driver Direction Input Pin.
#define R_MOT_PWM   41      // Right Motor Driver PWM Input Pin.
#define R_MOT_DIAG  42      // Right Motor Driver Fault Output Pin.
#define R_MOT_OCC   38      // Right Motor Driver Current Sense Output Pin.

/*********************************************************
                ALERT DEFAULT THRESHOLDS
**********************************************************/
#define DEF_AQI_LIM 200     // Default Air Quality Threshold (in AQI scale).
#define DEF_NOISE_LIM 50    // Default Noise Level Threshold (in dB).
#define DEF_TEMP_LIM 78     // Default Temperature Level Trheshold (in degrees F).
#define DEF_HUM_LIM 60      // Default Humidity Level Threshold (in relative humidity %).
#define DEF_PRESS_LIM 1000  // Default Pressure Level Threshold (in hPa).
#define DEF_HP_EST_LIM 150  // Default Human Presence Estimation Threshold (in inches).

// End include gaurd.
#endif /*sentryConfigInfo.h*/