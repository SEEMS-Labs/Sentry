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
ObstacleData obstacleInfo;

// Create Sentry.
Device sentry(&outgoingData, &outgoingAlerts, &incomingUserConfig, &incomingUserCommands, &obstacleInfo);

char *buffer;
void setup() {
    Serial.begin(BAUD_RATE);    
    vTaskDelay(pdMS_TO_TICKS(3000));           // delay to allow time to pullup serial monitor for debug.
    buffer = (char *) malloc(sizeof(char) * 20 * 40);
    Serial.println("Entering Device Setup.");
    for(int i = 0; i < 5; i++) {
        Serial.println(".");
        delay(500);
    }
    sentry.test();
} 

void loop() {
    //vTaskList(buffer);
    //Serial.printf("----------------------------\n");
    //Serial.printf(buffer);
    //Serial.printf("\n----------------------------\n");
    //sentry.showTaskMemoryUsage();
    //delay(3000);
    //sentry.loop();
}

//*/


/*
#include <Arduino.h>
#include <FirebaseClient.h>
#include <WiFiClientSecure.h>
#include "ExampleFunctions.h" // Provides the functions used in the examples.

#define WIFI_SSID "SEEMS"
#define WIFI_PASSWORD "@Ucf2025"
#define USER_EMAIL "es849112@ucf.edu"
#define USER_PASSWORD "rCpnKBR4ZhtefmL"
#define API_KEY "AIzaSyAGnxeU6342_TcjpmTKPT_WjB4AVTODfMk"
#define DATABASE_URL "https://seems-hub-default-rtdb.firebaseio.com/"

#define TASK_STACK_SIZE 8192
TaskHandle_t rx_dpad_handle = NULL;
TaskHandle_t rx_user_config_handle = NULL;
TaskHandle_t rx_process_handle = NULL;
TaskHandle_t read_bme_handle = NULL;
void rx_dpad_task(void *pvParams);
void rx_user_config_task(void *pvParams);
void rx_process_data_task(void *pvParams);
void read_bme_task(void *pvParams);
void createTasks();

#define SENTRYLINK_ROOT ((String) "sentrylink/")
#define CONTROLLER_PATH ((String) "controller")
#define CONFIG_PATH ((String) "user_config")

#define CTRL_JOYSTICK_MASK  0x02FF
#define CTRL_STATES_MASK    0x0003
#define CTRL_ON_MASK        0x0001
#define CTRL_JSTK_Y_LSB     15
#define CTRL_JSTK_X_LSB     5
#define CTRL_DP_LSB         2
#define CTRL_STA_LSB        1
#define CTRL_ON_LSB         0

#define CFG_THN_MASK 0x007F
#define CFG_P_MASK 0x01FF
#define CFG_A_MASK 0x07FF
#define CFG_TMP_LSB 0
#define CFG_HUM_LSB 7
#define CFG_SPL_LSB 14
#define CFG_PRS_LSB 21
#define CFG_AQI_LSB 30

int thresholds[5];
#define AQI_INDEX 0
#define HUM_INDEX 1
#define SPL_INDEX 2
#define PA_INDEX 3
#define TEMP_INDEX 4

WiFiClientSecure ssl_client, stream_ssl_client1, stream_ssl_client2;

using AsyncClient = AsyncClientClass;
AsyncClient aClient(ssl_client), sentryLinkStreamClient(stream_ssl_client1);

UserAuth user_auth(API_KEY, USER_EMAIL, USER_PASSWORD, 3000);
FirebaseApp app;
RealtimeDatabase Database;
AsyncResult streamResult1, streamResult2;

unsigned long ms = 0;

class Manager {
    private:
        WiFiClientSecure sslClient, sslStreamClient;
        AsyncClient a_client, a_streamClient;
        AsyncResult a_result, a_streamResult;
        FirebaseApp app;
        RealtimeDatabase database;
        UserAuth auth;

    public:
        Manager() : 
            sslClient(), a_result(), a_client(sslClient),
            sslStreamClient(), a_streamResult(), a_streamClient(sslStreamClient),
            app(), database(), auth(API_KEY, USER_EMAIL, USER_PASSWORD, 3000)
        {}

        AsyncClient *getAsyncClient() { return &a_client; }
        AsyncResult *getAsyncResult() { return &a_result; }
        AsyncClient *getStreamAsyncClient() { return &a_streamClient; }
        AsyncResult *getStreamAsyncResult() { return &a_streamResult; }
        WiFiClientSecure *getSSLClient() { return &sslClient; }
        WiFiClientSecure *getStreamSSLClient() { return &sslStreamClient; }
        FirebaseApp *getFirebaseApp() { return &app; }
        RealtimeDatabase *getRTDB() { return &database; }
        UserAuth *getUserAuth() {return &auth; }

};

class Rx {
    public:
        AsyncClient *_aClient, *_aStreamClient;
        AsyncResult *_aResult, *_aStreamResult;
        Manager *cm;
        
        Rx(Manager m) {
            cm = &m;
            _aClient = m.getAsyncClient();
            _aResult = m.getAsyncResult();
            _aStreamClient = m.getStreamAsyncClient();
            _aStreamResult = m.getStreamAsyncResult();
        }
        
        void processData(AsyncResult &aResult) {
            // Exits when no result available when calling from the loop.
            if (!aResult.isResult()) return;
            Serial.println("We're past");

            if (aResult.isEvent()) Firebase.printf("Event task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.eventLog().message().c_str(), aResult.eventLog().code());

            if (aResult.isDebug()) Firebase.printf("Debug task: %s, msg: %s\n", aResult.uid().c_str(), aResult.debug().c_str());

            if (aResult.isError()) Firebase.printf("Error task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.error().message().c_str(), aResult.error().code());

            if (aResult.available()) {
                RealtimeDatabaseResult &RTDB = aResult.to<RealtimeDatabaseResult>();
                if (RTDB.isStream()) {
                    Serial.println("----------------------------");
                    Firebase.printf("task: %s\n", aResult.uid().c_str());
                    Firebase.printf("event: %s\n", RTDB.event().c_str());
                    Firebase.printf("path: %s\n", RTDB.dataPath().c_str());
                    Firebase.printf("data: %s\n", RTDB.to<const char *>());
                    Firebase.printf("type: %d\n", RTDB.type());

                    // The stream event from RealtimeDatabaseResult can be converted to the values as following.
                    bool v1 = RTDB.to<bool>();
                    int v2 = RTDB.to<int>();
                    float v3 = RTDB.to<float>();
                    double v4 = RTDB.to<double>();
                    String v5 = RTDB.to<String>();
                    
                    String fieldPath = RTDB.dataPath().substring(1);
                    Serial.println(fieldPath);
                    
                    if(fieldPath.equals(CONTROLLER_PATH)) {
                        xTaskNotify(rx_dpad_handle, -1, eNoAction);
                    }
                    else if(fieldPath.equals(CONFIG_PATH)) {
                        xTaskNotify(rx_user_config_handle, -1, eNoAction);
                    }
                    else Serial.println("Ummm. You're Cooked man.");
                }
                else {
                    Serial.println("----------------------------");
                    Firebase.printf("task: %s, payload: %s\n", aResult.uid().c_str(), aResult.c_str());
                }
                Firebase.printf("Free Heap: %d\n", ESP.getFreeHeap());
            }
        }

        void rx_user_data() {

        }


};

Manager cm;
Rx *receiver = new Rx(cm);

void setup()
{
    Serial.begin(115200);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(300);
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    Firebase.printf("Firebase Client v%s\n", FIREBASE_CLIENT_VERSION);

    cm.getSSLClient()->setInsecure();
    cm.getStreamSSLClient()->setInsecure();

    //ssl_client.setInsecure();
    //stream_ssl_client1.setInsecure();

    Serial.println("Initializing app...");
    initializeApp(
        *cm.getAsyncClient(), 
        *cm.getFirebaseApp(), 
        getAuth(*cm.getUserAuth()), 
        auth_debug_print, 
        "🔐 authTask"
    );

    //initializeApp(aClient, app, getAuth(user_auth), auth_debug_print, "🔐 authTask");

    // Or intialize the app and wait.
    //initializeApp(aClient, app, getAuth(user_auth), 120 * 1000, auth_debug_print);

    //app.getApp<RealtimeDatabase>(Database);
    //Database.url(DATABASE_URL);
    cm.getFirebaseApp()->getApp<RealtimeDatabase>(*cm.getRTDB());
    cm.getRTDB()->url(DATABASE_URL);

    // In SSE mode (HTTP Streaming) task, you can filter the Stream events by using AsyncClientClass::setSSEFilters(<keywords>),
    // which the <keywords> is the comma separated events.
    // The event keywords supported are:
    // get - To allow the http get response (first put event since stream connected).
    // put - To allow the put event.
    // patch - To allow the patch event.
    // keep-alive - To allow the keep-alive event.
    // cancel - To allow the cancel event.
    // auth_revoked - To allow the auth_revoked event.
    // To clear all prevousely set filter to allow all Stream events, use AsyncClientClass::setSSEFilters().
    //sentryLinkStreamClient.setSSEFilters("get,put,patch,keep-alive,cancel,auth_revoked");
    cm.getStreamAsyncClient()->setSSEFilters("get,put,patch,keep-alive,cancel,auth_revoked");


    // The "unauthenticate" error can be occurred in this case because we don't wait
    // the app to be authenticated before connecting the stream.
    // This is ok as stream task will be reconnected automatically when the app is authenticated.
    //Database.get(sentryLinkStreamClient, SENTRYLINK_ROOT, receiver->processData(streamResult1), true, "sentryLinkStreamTask");

    // Async call with AsyncResult for returning result.
    //Database.get(sentryLinkStreamClient, SENTRYLINK_ROOT, streamResult1, true);
    cm.getRTDB()->get(
        *cm.getStreamAsyncClient(), 
        SENTRYLINK_ROOT, 
        *cm.getStreamAsyncResult(), 
        true
    );
    
    ulong startTime = millis();
    int timeoutMax = 5000;

    while(cm.getFirebaseApp()->ready() == false && (millis() - startTime) < timeoutMax) {
        Serial.println("Waiting to connect to Firebase.");
        delay(1);
    }

    if((millis() - startTime) > timeoutMax) {
        while(1) {
            Serial.println("Connection failed. Try again.");
            delay(1000);
        }
    }
    
    cm.getRTDB()->set<bool>(
        *cm.getAsyncClient(),
        "sentry/sentry_conn",
        true
    );

    createTasks();
}

void loop() {  
   // app.loop(); 
}

void rx_dpad_task(void *pvParams) {
    // Setup.

    // Main task loop.
    for(;;) {
        // Wait for dpad data received.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        Serial.println("Received Dpad Change");

    }
}

void rx_user_config_task(void *pvParams) {
    // Setup.

    // Main task loop.
    for(;;) {
        // Wait for config data received.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        Serial.println("Received config Change");

    }
}

void rx_process_data_task(void *pvParams) {
    // Setup.
    Rx *rcvr = static_cast<Rx *>(pvParams);
    AsyncResult *res = (cm.getStreamAsyncResult());
    FirebaseApp *f_app = (cm.getFirebaseApp());

    // Main task loop.
    for(;;) {
        // poll the receiver.
        //Serial.println("Polling Receiver for data!");
        f_app->loop();
        rcvr->processData(*res);
        vTaskDelay(pdMS_TO_TICKS(125));
    }
}

void read_bme_task(void *pvParams) {
    // Setup.

    // Main task loop.
    for(;;) {
        // poll the receiver.
        Serial.println("Reading BME");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void createTasks() {
    // Create the task to receive motor control commands.
    xTaskCreatePinnedToCore(
        &rx_dpad_task,   // Pointer to task function.
        "rx_dpad_task",  // Task name.
        TASK_STACK_SIZE,       // Size of stack allocated to the task (in bytes).
        NULL,                   // Pointer to parameters used for task creation.
        2,           // Task priority level.
        &rx_dpad_handle, // Pointer to task handle.
        1                       // Core that the task will run on.
    );

    // Create the task to receive user config commands.
    xTaskCreatePinnedToCore(
        &rx_user_config_task,   // Pointer to task function.
        "rx_user_config_task",  // Task name.
        TASK_STACK_SIZE,       // Size of stack allocated to the task (in bytes).
        NULL,                   // Pointer to parameters used for task creation.
        2,           // Task priority level.
        &rx_user_config_handle, // Pointer to task handle.
        1                       // Core that the task will run on.
    );

    // Create the task to process data received.
    xTaskCreatePinnedToCore(
        &rx_process_data_task,   // Pointer to task function.
        "rx_process_data_task",  // Task name.
        TASK_STACK_SIZE,       // Size of stack allocated to the task (in bytes).
        NULL,                   // Pointer to parameters used for task creation.
        4,           // Task priority level.
        &rx_process_handle, // Pointer to task handle.
        1                       // Core that the task will run on.
    );

    // Create the task to read bme received.
    xTaskCreatePinnedToCore(
        &read_bme_task,   // Pointer to task function.
        "read_bme_Task",  // Task name.
        TASK_STACK_SIZE,       // Size of stack allocated to the task (in bytes).
        NULL,                   // Pointer to parameters used for task creation.
        3,           // Task priority level.
        &read_bme_handle, // Pointer to task handle.
        1                       // Core that the task will run on.
    );
}

*/