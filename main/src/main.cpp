#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <FirebaseClient.h>
//#include <ExampleFunctions.h>

#define BAUD_RATE 115200
#define ONE_SECOND 1000

#define WIFI_SSID "``````"
#define WIFI_PASSWORD "`````````"
#define USER_EMAIL "``````````"
#define USER_PASSWORD "`````````"
#define API_KEY "```````````````"
#define DATABASE_URL "`````````"

// Constructs from example.
void auth_debug_print(AsyncResult &aResult);
WiFiClientSecure ssl_client;
using AsyncClient = AsyncClientClass;
AsyncClient aClient1(ssl_client);
UserAuth user_auth(API_KEY, USER_EMAIL, USER_PASSWORD, 3000);
FirebaseApp app;
RealtimeDatabase Database1;
AsyncResult databaseResult;
void processData(AsyncResult &res);

// Tasks.
SemaphoreHandle_t app_mutex = NULL;
const int _size = 8192;
typedef struct _info {
    RealtimeDatabase *db1;
    AsyncClient *ac1;
} Info;
void tx_1_task(void *pvParams);
void tx_2_task(void *pvParams);
void inc_1_task(void *pvParams);
void inc_2_task(void *pvParams);
void createTasks(int numTasks);

float reading = 0.6;
float aqi_reading = -1;
float hum_reading = -1;
float pres_reading = -1;
float temp_reading = -1;
float db_reading = 1;
bool status = false;
bool status_loop1 = true;
bool status_loop2 = true;

typedef struct _sensorData SensorData;
struct _sensorData {
    float airQualityLevel;
    float temperatureLevel;
    float humidityLevel;
    float pressureLevel;
    float noiseLevel;
};

Info fbInfo;
SensorData dataPacket;
TaskHandle_t tx_1_handle = NULL;
TaskHandle_t tx_2_handle = NULL;
TaskHandle_t inc_1_handle = NULL;
TaskHandle_t inc_2_handle = NULL;

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

    (ssl_client).setInsecure();
    Serial.println("Initializing app..."); 
    initializeApp(aClient1, app, getAuth(user_auth), auth_debug_print, "AuthTask");
    app.getApp<RealtimeDatabase>(Database1);
    Database1.url(DATABASE_URL);
    
    // Push bool
    Serial.println("Notifiying Firebase that Sentry has connected succesfully.");
    status = Database1.set<bool>(aClient1, "sentry_conn", true);
    if (status) Serial.println("Succesful Connection.");
    else {
        Serial.println("RIP...");
    }

    // start tasks.
    Serial.printf("Number of running tasks Before Setup: %d\n", uxTaskGetNumberOfTasks());
    createTasks(2);
    Serial.printf("Number of running tasks after Setup: %d\n", uxTaskGetNumberOfTasks());
    Serial.println("Booting");
    for(int i = 0; i < 10; i++) { 
        Serial.print(".");
        delay(300);
    }
} 

void loop() {
    processData(databaseResult); 
}

void createTasks(int numTasks) {
    // Setup info.
    app_mutex = xSemaphoreCreateMutex();
    fbInfo.db1 = &Database1;
    fbInfo.ac1 = &aClient1;

    // Create the task to transmit sensor data.
    Serial.println("Creating Task 1");
    xTaskCreatePinnedToCore(
        &tx_1_task,   // Pointer to task function.
        "tx_1_task",  // Task name.
        _size,       // Size of stack allocated to the task (in bytes).
        &fbInfo,                   // Pointer to parameters used for task creation.
        2,           // Task priority level.
        &tx_1_handle, // Pointer to task handle.
        1                       // Core that the task will run on.
    );

    if(numTasks == 1) return;

    // Create the task to transmit Noise alerts.
    Serial.println("Creating Task 2"); 
    xTaskCreatePinnedToCore(
        &tx_2_task,   // Pointer to task function.
        "tx_2_task",  // Task name.
        _size,       // Size of stack allocated to the task (in bytes).
        &fbInfo,                   // Pointer to parameters used for task creation.
        2,           // Task priority level.
        &tx_2_handle, // Pointer to task handle.
        1                       // Core that the task will run on.
    );

    // Create the task to transmit Noise alerts.
    Serial.println("Creating Task 3"); 
    xTaskCreatePinnedToCore(
        &inc_1_task,   // Pointer to task function.
        "tx_3_task",  // Task name.
        _size,       // Size of stack allocated to the task (in bytes).
        &dataPacket,                   // Pointer to parameters used for task creation.
        1,           // Task priority level.
        &inc_1_handle, // Pointer to task handle.
        1                       // Core that the task will run on.
    );

    Serial.println("Creating Task 4"); 
    xTaskCreatePinnedToCore(
        &inc_2_task,   // Pointer to task function.
        "inc_2_task",  // Task name.
        _size,       // Size of stack allocated to the task (in bytes).
        &dataPacket,                   // Pointer to parameters used for task creation.
        1,           // Task priority level.
        &inc_2_handle, // Pointer to task handle.
        1                       // Core that the task will run on.
    );
}

void processData(AsyncResult &res) {
    // Exits when no result available when calling from the loop.
    if (!res.isResult())
        return;

    if (res.isEvent())
    {
        Firebase.printf("Event task: %s, msg: %s, code: %d\n", res.uid().c_str(), res.eventLog().message().c_str(), res.eventLog().code());
    }

    if (res.isDebug())
    {
        Firebase.printf("Debug task: %s, msg: %s\n", res.uid().c_str(), res.debug().c_str());
    }

    if (res.isError())
    {
        Firebase.printf("Error task: %s, msg: %s, code: %d\n", res.uid().c_str(), res.error().message().c_str(), res.error().code());
    }

    if (res.available())
    {
        Firebase.printf("task: %s, payload: %s\n", res.uid().c_str(), res.c_str());
    }
}

void auth_debug_print(AsyncResult &aResult)
{
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
}

#define WAS_READY 0x01
#define WAS_NOT_READY 0x02
#define MUTEX_TAKEN 0x03

unsigned long t1_last, t2_last;
unsigned long t1_tx_time;
void tx_1_task(void *pvParams) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // Grab firebase database.
    Info *fbInfo = static_cast<Info *>(pvParams);
    RealtimeDatabase *db = fbInfo->db1;
    AsyncClient *asc = fbInfo->ac1;

    object_t json1, json2, json3, json4, json;
    JsonWriter writer;
    char t1_app_status = 0x00;

    // Task loop.
    for(;;) {
        if(xSemaphoreTake(app_mutex, portMAX_DELAY) == pdTRUE) {
            // Loop Firebase app.
            Serial.println("Taking App Mutex to Task 1.");
            app.loop();
            if(app.ready()) {
                writer.create(json1, "airQuality", dataPacket.airQualityLevel);
                writer.create(json2, "pressure", dataPacket.pressureLevel);
                writer.create(json3, "humidity", dataPacket.humidityLevel);
                writer.create(json4, "temperature", dataPacket.temperatureLevel);  
                writer.join(json, 4, json1, json2, json3, json4);

                //Serial.printf("Free Heap before update: %d\n", ESP.getFreeHeap());
                Serial.printf("- t_elapsed env. update call = %lu\n", millis() - t1_last);
                t1_last = millis();

                status_loop1 = db->set<object_t>(*asc, "sensors/bme", json);
                status = db->set<bool>(*asc, "sentry_conn", false);
                Serial.printf("- t_set env in FB = %lu\n", millis() - t1_last);

                //if (status_loop1) Serial.printf("ok: %lf\n", aqi_reading);
                //else Serial.println("not ok");

                //Serial.printf("Free Heap after update: %d\n", ESP.getFreeHeap());
                t1_app_status = WAS_READY;
            }
            else {
                t1_app_status = WAS_NOT_READY;
                //Serial.println("App not ready in task1.");
            }

            Serial.println("Yielding App_Mutex from Task 1.");
            xSemaphoreGive(app_mutex);
        }
        else {
            t1_app_status = MUTEX_TAKEN;
            //Serial.println("app was taken(mutex) in task1. waiting for app");
        }   

        switch (t1_app_status) {
            case WAS_READY:
                //Serial.println("->WAS READY - Delay 3000 ms");
                vTaskDelay(pdMS_TO_TICKS(3000));
                break;

            case WAS_NOT_READY:
                //Serial.println("->WAS NOT READY - Delay 10 ms");
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
            
            case MUTEX_TAKEN:
                //Serial.println("->MUTEX TAKEN - Delay 10 ms");
                vTaskDelay(pdMS_TO_TICKS(10));
                break;

            default:
                //Serial.println("->Default App Status - Delay 100ms");
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
        }
    }
}

void tx_2_task(void *pvParams) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // Grab firebase database.
    Info *fbInfo = static_cast<Info *>(pvParams);
    char t2_app_status = 0x00;
    
    // Task loop.
    for(;;) {
        //Serial.println("---Entering Task 2");
        if(xSemaphoreTake(app_mutex, portMAX_DELAY) == pdTRUE) {
            Serial.println("Taking App Mutex to Task 2.");
            app.loop();
            if(app.ready()) {
                Serial.printf("- t_elapsed dB update call = %lu\n", millis() - t2_last);
                t2_last = millis();

                status_loop2 = fbInfo->db1->set<float>(*(fbInfo->ac1), "sensors/mic", dataPacket.noiseLevel);
                status = fbInfo->db1->set<bool>(*(fbInfo->ac1), "sentry_conn", true);
                Serial.printf("- t_set dB FB set = %lu\n", millis() - t2_last);
                //if (status_loop2) Serial.printf("ok: %d\n", db_reading);
                //else Serial.println("not ok");
                t2_app_status = WAS_READY;
            }  
            else {
                //Serial.println("App not ready in task2.");
                t2_app_status = WAS_NOT_READY;
            }

            Serial.println("Yielding App_Mutex from Task 2.");
            xSemaphoreGive(app_mutex);
        }
        else {
            t2_app_status = MUTEX_TAKEN;
            //Serial.println("app was taken(mutex) in task2. waiting for app");
        }

        switch (t2_app_status) {
            case WAS_READY:
                //Serial.println("->WAS READY - Delay 125 ms");
                vTaskDelay(pdMS_TO_TICKS(125));
                break;

            case WAS_NOT_READY:
                //Serial.println("->WAS NOT READY - Delay 10 ms");
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
            
            case MUTEX_TAKEN:
                //Serial.println("->MUTEX TAKEN - Delay 10 ms");
                vTaskDelay(pdMS_TO_TICKS(10));
                break;

            default:
                //Serial.println("->Default App Status - Delay 100ms");
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
        }
    }
}

void inc_1_task(void *pvParams) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    SensorData *data = static_cast<SensorData *>(pvParams);

    for(;;) {

        /*
        dataPacket.airQualityLevel++;
        dataPacket.humidityLevel++;
        dataPacket.pressureLevel++;
        dataPacket.temperatureLevel++;
        */
        ///*
        data->airQualityLevel++;
        data->humidityLevel++;
        data->pressureLevel++;
        data->temperatureLevel++;
        //*/
        vTaskDelay(pdMS_TO_TICKS(2950));

    }
}

void inc_2_task(void *pvParams) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    SensorData *data = static_cast<SensorData *>(pvParams);

    for(;;) {
        data->noiseLevel++;
        //dataPacket.noiseLevel++;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}