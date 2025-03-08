
// External libraries.
#include <Arduino.h>

///*
// Internal headers.
#include "sentryConfigInfo.h"
#include "Device.h"

// Global data storage.
SensorData outgoingData;
Alerts outgoingAlerts;
UserSentryConfig incomingUserConfig;
UserDriveCommands incomingUserCommands;

// Create Sentry.
Device sentry(&outgoingData, &outgoingAlerts, incomingUserConfig, incomingUserCommands);

void setup() {
    Serial.begin(BAUD_RATE);    
    vTaskDelay(pdMS_TO_TICKS(10000));           // 10s delay to allow time to pullup serial monitor for debug.
    Serial.println("Entering Device Setup.");
    for(int i = 0; i < 10; i++) {
        Serial.println(".");
        delay(500);
    }
    sentry.test_data_to_firebase();
    //testDrive(sentry);
    //vTaskDelete(NULL);                          // End setup.
} 

void loop() {
    sentry.loop();
    //Serial.println("Looooop");
    //vTaskDelay(pdMS_TO_TICKS(1000));
}
//*/

/*
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <FirebaseClient.h>
#include <ExampleFunctions.h>

#define BAUD_RATE 115200
#define ONE_SECOND 1000

//#define WIFI_SSID "WhiteSky-KnightsCircle"
//#define WIFI_PASSWORD "d9wk6xfg"
#define WIFI_SSID "SEEMS"
#define WIFI_PASSWORD "@Ucf2025"

#define USER_EMAIL "es849112@ucf.edu"
#define USER_PASSWORD "rCpnKBR4ZhtefmL"

#define API_KEY "AIzaSyAGnxeU6342_TcjpmTKPT_WjB4AVTODfMk"
#define DATABASE_URL "https://seems-hub-default-rtdb.firebaseio.com/"

// Fucntions for example.
void setAsync();
void processData(AsyncResult &aResult);

// Constructs from example.
using AsyncClient = AsyncClientClass;
SSL_CLIENT ssl_client;
AsyncClient aClient(ssl_client);
UserAuth user_auth(API_KEY, USER_EMAIL, USER_PASSWORD);
FirebaseApp app;
RealtimeDatabase Database;
AsyncResult aResult_no_callback;

void setup() {
    Serial.begin(BAUD_RATE);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(300);
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    Firebase.printf("Firebase Client v%s\n", FIREBASE_CLIENT_VERSION);

    Serial.println("Initializing app..."); 
    set_ssl_client_insecure_and_buffer(ssl_client);
    initializeApp(aClient, app, getAuth(user_auth), auth_debug_print, "authTask");

    // Binding the FirebaseApp for authentication handler.
    // To unbind, use Database.resetApp();
    app.getApp<RealtimeDatabase>(Database);

    Database.url(DATABASE_URL);
} 

float reading = 0.6;
bool status_loop;
object_t json1, json2, json3, json4, json;
object_t jsonStrings[5];
JsonWriter writer;
bool taskComplete = false;
void loop() {
    app.loop();
    Serial.println("We looping");

    // Check if reauthentication is required
    if (app.ready()) {
        Serial.println("Async set Values");
        setAsync();
    }

    processData(aResult_no_callback);

    uint32_t rand = esp_random();
    if(rand % 5 == 0) delay(2 * ONE_SECOND);
    else if(rand % 3 == 0) delay(0.1 * ONE_SECOND);
    else if (rand % 2 == 0) delay(0.01 * ONE_SECOND);
    else delay(ONE_SECOND);

}

void setAsync() {
    writer.create(jsonStrings[1], "airQuality", reading++);
    writer.create(jsonStrings[2], "humidity", reading++);
    writer.create(jsonStrings[3], "pressure", reading++); 
    writer.create(jsonStrings[4], "temperature", reading++);
    writer.join(jsonStrings[0], 4, jsonStrings[1], jsonStrings[2], jsonStrings[3], jsonStrings[4]);

    Serial.println("updating readings... ");
    Database.set<object_t>(aClient, "readings/", jsonStrings[0], processData, "setJsonTask");
}

void processData(AsyncResult &aResult) {
    // Exits when no result available when calling from the loop.
    if (!aResult.isResult())
        return;

    if (aResult.isEvent())
    {
        Firebase.printf("Event task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.eventLog().message().c_str(), aResult.eventLog().code());
    }

    if (aResult.isDebug())
    {
        Firebase.printf("Debug task: %s, msg: %s\n", aResult.uid().c_str(), aResult.debug().c_str());
    }

    if (aResult.isError())
    {
        Firebase.printf("Error task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.error().message().c_str(), aResult.error().code());
    }

    if (aResult.available())
    {
        Firebase.printf("task: %s, payload: %s\n", aResult.uid().c_str(), aResult.c_str());
    }
}

*/

/*
// External libraries.
#include <Arduino.h>
#include <bsec.h>
#include <Wire.h>

// BME Stuff.
#define _BME_SCK 2
#define _BME_SDI 1
Bsec bme688;
void initWire();
void beginBME();
void error(String str);
void checkSensorStatus();
void readSensor();
void toggleSensor();
bool bme_active = false;
bsec_virtual_sensor_t sensorList[13] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_STABILIZATION_STATUS,
    BSEC_OUTPUT_RUN_IN_STATUS,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_GAS_PERCENTAGE
};


void setup() {
    Serial.begin(115200);    
    vTaskDelay(pdMS_TO_TICKS(10000));           // 10s delay to allow time to pullup serial monitor for debug.
    Serial.println("Entering Device Setup.");
    //sentry.testComms();

    initWire();
    beginBME();
    //vTaskDelete(NULL);                          // End setup.
} 

void loop() {
    int64_t time = bme688.getTimeMs();
    Serial.printf("--Normal Sentry Operations: %lld\n", time);
    readSensor();
    //if(time % 60000 < 1000) {
    //    if(bme_active) Serial.println("--Turning Sensor off.");
    //    else Serial.println("--Turning sensor on.");
    //    toggleSensor();
    //}
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void initWire() {
    Wire.setPins(_BME_SDI, _BME_SCK);
    Wire.begin();
}

void beginBME() {
    bme688.begin(BME68X_I2C_ADDR_HIGH, Wire);
    String output = "\nBSEC library version " + String(bme688.version.major) + "." + String(bme688.version.minor) + "." + String(bme688.version.major_bugfix) + "." + String(bme688.version.minor_bugfix);
    Serial.println(output);
    checkSensorStatus();

    bme688.updateSubscription(sensorList, 13, BSEC_SAMPLE_RATE_LP);
    checkSensorStatus();
    bme_active = true;

    // Data table Header.
    output = "Timestamp [ms], IAQ, IAQ accuracy, Static IAQ, CO2 equivalent, breath VOC equivalent, raw temp[°C], pressure [hPa], raw relative humidity [%], gas [Ohm], Stab Status, run in status, comp temp[°C], comp humidity [%], gas percentage";
    Serial.println(output);
}

void checkSensorStatus() {
    bsec_library_return_t status = bme688.bsecStatus;
    if(status != BSEC_OK) {
        if(status < BSEC_OK) {
            Serial.printf("BSEC Error Code: %s\n", String(status).c_str());
            for(;;) error("BSEC");
        }
        else Serial.printf("BSEC Warning Code: %s\n", String(status).c_str());
    }

    int8_t bme_status = bme688.bme68xStatus;
    if(bme_status != BME68X_OK) {
        if(bme_status < BME68X_OK) {
            Serial.printf("BME688 Error Code: %d", bme_status);
            for(;;) error("BME688");
        }
        else Serial.printf("BME688 Warning Code: %s", String(bme_status).c_str());
    }
}

void error(String str) {
    Serial.println("ERROR!" + str);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

String result;
void readSensor() {
    if(!bme_active) return;
    unsigned long time_trigger = millis();
    bool ready = bme688.run();
    if (ready) { // If new data is available
        result = String(time_trigger);
        result += ", " + String(bme688.iaq);
        result += ", " + String(bme688.iaqAccuracy);
        result += ", " + String(bme688.staticIaq);
        result += ", " + String(bme688.co2Equivalent);
        result += ", " + String(bme688.breathVocEquivalent);
        result += ", " + String(bme688.rawTemperature);
        result += ", " + String(bme688.pressure);
        result += ", " + String(bme688.rawHumidity);
        result += ", " + String(bme688.gasResistance);
        result += ", " + String(bme688.stabStatus);
        result += ", " + String(bme688.runInStatus);
        result += ", " + String(bme688.temperature);
        result += ", " + String(bme688.humidity);
        result += ", " + String(bme688.gasPercentage);
        Serial.println(result);

        Serial.print("Pressure: ");
        Serial.print(bme688.pressure/100.0);
        Serial.println(" hPa");
        
        Serial.print("Temperature: ");
        Serial.print(bme688.temperature);
        Serial.println(" *C");
        
        Serial.print("Humidity: ");
        Serial.print(bme688.humidity);
        Serial.println(" %");
        
        Serial.print("IAQ: ");
        Serial.print(bme688.iaq);
        Serial.println(" Index");
        
        Serial.print("CO2 Equivalent: ");
        Serial.print(bme688.co2Equivalent);
        Serial.println(" PPM");
        
        Serial.print("Breath VOC Equivalent: ");
        Serial.print(bme688.breathVocEquivalent);
        Serial.println(" PPM");
        Serial.println();
    } else {
        if(!bme_active) Serial.print("Sensor Inactive!: ");
        checkSensorStatus();
    }

}

void toggleSensor() {
    // Switch sensor off.
    if(bme_active) {
        bme_active = false;
        bme688.updateSubscription(sensorList, 13, BSEC_SAMPLE_RATE_DISABLED);
    }

    // Switch sensor on.
    else {
        bme_active = true;
        bme688.updateSubscription(sensorList, 13, BSEC_SAMPLE_RATE_LP);
    }
}

*/