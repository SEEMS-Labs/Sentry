#include "device.h"

void Device::begin() {
    
    // Enable the Sentry's ADC to 12-bit resolution.
    analogReadResolution(12);

    // Initialize Sentry Subsystems.
    initSensorSystem();
    initDriveSystem();
    initCommunicationSystem();
}

void Device::sleep_mode_1() {

}

void Device::sleep_mode_2() {

}

void Device::shutdown() {

}

TB9051FTG Device::get_drive_system() { return _drive_system; }

void Device::testComms() {
    // Initialize Communication System.
    Serial.println("Beginning Comms Test.");
    for(int i = 0; i < 10; i++) {
        Serial.println(".");
        delay(500);
    }
    _communication_system.begin();
    Serial.println("Ending Comms Test.");
}

void Device::test_bme_data_to_firebase() {
    Serial.println("Beginning Transmission of Sensor Data to Firebase test");
    for(int i = 0; i < 10; i++) {
        Serial.println(".");
        delay(500);
    }
    _communication_system.begin();
    _sensor_system.initBME();
    _sensor_system.beginReadBMETask();
}

void Device::test_mic_data_to_firebase() {
    Serial.println("Beginning Transmission of Microphone Data to Firebase test");
    for(int i = 0; i < 10; i++) {
        Serial.println(".");
        delay(500);
    }
    _communication_system.begin();
    _sensor_system.initMic();
    _sensor_system.beginReadMicrophoneTask();
}

void Device::test_bme_and_mic_data_to_firebase() {
    Serial.println("Beginning Transmission of BME and Microphone Data to Firebase test");
    for(int i = 0; i < 10; i++) {
        Serial.println(".");
        delay(500);
    }

    _communication_system.begin();
    _sensor_system.initMic();
    _sensor_system.initBME();

    BaseType_t taskCreated;
    taskCreated = _sensor_system.beginReadBMETask();
    if(taskCreated != pdPASS) Serial.printf("Read BME task not created. Fail Code: %d\n", taskCreated);
    else Serial.println("Read BME task created.");

    taskCreated = _sensor_system.beginReadMicrophoneTask();
    if(taskCreated != pdPASS) Serial.printf("Read Mic task not created. Fail Code: %d\n", taskCreated);
    else Serial.println("Read Mic task created.");
}

void Device::test_US() {

    // Init comms systems.
    _communication_system.begin();
    
    // Initialize sensor system stuff.
    _sensor_system.initUS();
    _sensor_system.attachAllInterrupts();
    _sensor_system.beginReadUltrasonicTask();
    _sensor_system.beginUpdateThresholdTask();
    StateManager::getManager()->setSentrySensorThresholdState(ThresholdState::ts_POST_STARTUP);

    // Diasble Left/Right/Back ultrasonics.
    _sensor_system.fetchUS(B_US_ID)->disable();
    _sensor_system.fetchUS(L_US_ID)->disable();
    _sensor_system.fetchUS(R_US_ID)->disable();  
}

void Device::initSensorSystem() {
    _sensor_system.initAllSensors();
    _sensor_system.attachAllInterrupts();
    _sensor_system.beginAllTasks();
}

void Device::initDriveSystem() {
    _drive_system.init();
    _drive_system.getLeftMotor()->disable();
    _drive_system.setSpeed(DEFAULT_SPEED);
}

void Device::initCommunicationSystem() {
    _communication_system.begin();
}

void Device::loop() {
    // Loop the FIREBASEAPP for auth purposes.
    _communication_system.getFbApp()->loop();
}

void Device::test_bme_data_to_serial() {
    _sensor_system.initBME();
    _sensor_system.beginReadBMETask();
}

void Device::test_motor() {
    // Initialize.
    analogReadResolution(12);
    initDriveSystem();

    for(int i = 0; i < 20; i++) {
        // Drive Forwards.
        Serial.println("Moving Forward!");
        _drive_system.moveForward();
        vTaskDelay(pdMS_TO_TICKS(10000));

        // Stop.
        //Serial.println("Stopping!");
        //_drive_system.stop(stopType::BRAKE);
        //vTaskDelay(pdMS_TO_TICKS(10));
        
        // Drive Backwards.
        //Serial.println("Moving Backward!");
        //_drive_system.moveBackward();
        //vTaskDelay(pdMS_TO_TICKS(10000));

        // Stop.
        Serial.println("Last Stop!");
        _drive_system.stop(stopType::BRAKE);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void Device::test_connection_to_firebase() {
    Serial.println("Beginning Connection to Firebase test");
    for(int i = 0; i < 10; i++) {
        Serial.println(".");
        delay(500);
    }
    _communication_system.begin();
}

void Device::test() {
    Serial.println("Beginning Test");
    for(int i = 0; i < 10; i++) {
        Serial.println(".");
        delay(500);
    }

    UBaseType_t taskCount = uxTaskGetNumberOfTasks();
    Serial.printf("Number of running tasks Before: %d\n", taskCount);

    _sensor_system.beginUpdateThresholdTask();
    Serial.println("Update threshold task initializd.");

    _communication_system.begin();
    Serial.println("Comms System Initialized.");

    _sensor_system.initMic();
    Serial.println("Mic Initialized.");

    _sensor_system.initBME();
    Serial.println("BME Initialized.");
    
    _sensor_system.beginReadBMETask();
    Serial.println("BME Task Initialized.");

    _sensor_system.beginReadMicrophoneTask();
    Serial.println("Mic Task Initialized.");

    taskCount = uxTaskGetNumberOfTasks();
    Serial.printf("Number of running tasks After: %d\n", taskCount);

    StateManager::getManager()->setSentrySensorThresholdState(ThresholdState::ts_POST_STARTUP);

}

void Device::showTaskMemoryUsage() {

    UBaseType_t poll_us_mem_left = uxTaskGetStackHighWaterMark(poll_US_handle);
    UBaseType_t poll_mic_mem_left = uxTaskGetStackHighWaterMark(poll_mic_handle);
    UBaseType_t poll_bme_mem_left = uxTaskGetStackHighWaterMark(poll_bme_handle);
    UBaseType_t tx_bme_data_mem_left = uxTaskGetStackHighWaterMark(tx_bme_data_handle);
    UBaseType_t tx_mic_data_mem_left = uxTaskGetStackHighWaterMark(tx_mic_data_handle);
    UBaseType_t tx_alerts_mem_left = uxTaskGetStackHighWaterMark(tx_alerts_handle);
    UBaseType_t rx_user_data_mem_left = uxTaskGetStackHighWaterMark(rx_user_data_handle);
    UBaseType_t user_ctrld_mvmt_mem_left = uxTaskGetStackHighWaterMark(user_ctrld_mvmt_handle);
    UBaseType_t walk_algo_mem_left = uxTaskGetStackHighWaterMark(erw_mvmt_handle);

    Serial.printf("poll_us_mem_left: \t%u bytes\n", poll_us_mem_left);
    Serial.printf("poll_mic_mem_left: \t%u bytes\n", poll_mic_mem_left);
    Serial.printf("poll_bme_mem_left: \t%u bytes\n", poll_bme_mem_left);
    Serial.printf("tx_bme_data_mem_left: \t%u bytes\n", tx_bme_data_mem_left);
    Serial.printf("tx_mic_data_mem_left: \t%u bytes\n", tx_mic_data_mem_left);
    Serial.printf("tx_alerts_mem_left: \t%u bytes\n", tx_alerts_mem_left);
    Serial.printf("rx_user_data_mem_left: \t%u bytes\n", rx_user_data_mem_left);
    Serial.printf("user_mvmt_mem_left: \t%u bytes\n", user_ctrld_mvmt_mem_left);
    Serial.printf("walk_algo_mem_left: \t%u bytes\n", walk_algo_mem_left);

}

