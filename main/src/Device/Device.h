
#ifndef DEVICE_H
#define DEVICE_H

#include "Sentry/main/src/sentryConfigInfo.h"
#include "Sentry/main/src/StateManager.h"
#include "SensorManager.h"
#include "ConnectivityManager.h"
#include "TB9051FTG.h"

/**
 * This class represents the Sentry as a whole. It is from here the Sentry will be initialized and controlled.
 */
class Device {
    private: 
        StateManager *_stateManager;                    // Sentry State Manager.
        ConnectivityManager _communication_system;      // Network Connectivity Management Unit.
        SensorManager _sensor_system;                   // Sensor Management Unit.
        TB9051FTG _drive_system;                        // Drive System Management Unit.

        void initADC();
        void createSemaphores();
        void initSensorSystem();                        // Start the BME688, Microphone, and Ultrasonics.
        void initDriveSystem();                         // Start the Motors
        void initCommunicationSystem();                 // Start the Commmunications.

    public:
        Device( 
            SensorData *envData,                                    // Address to the global environmental data packet.
            Alerts *envStatus,                                      // Address to the global alerts data packet.
            UserSentryConfig *userConfiguration,                    // Address to the global custom user sentry configuration data packet.
            UserDriveCommands *userMovementCommands,                // Address to the global user movement commands data packet.
            ObstacleData *obstacleInfo) :                           // Address to the global obstacle data packet.
            _communication_system(envData, envStatus, userConfiguration, userMovementCommands),
            _sensor_system(envData, envStatus, userConfiguration, obstacleInfo),
            _drive_system(userMovementCommands),
            _stateManager(StateManager::getManager()) {}

        // Start the Sentry.
        void begin();     

        // Unimplemented. Light Sleep mode for the Sentry.         
        void sleep_mode_1();

        // Unimplemented. Deeper Sleep mode for the Sentry.    
        void sleep_mode_2();

        // Unimplemented. Shuts Sentry Down during a dedicated shutdown.    
        void shutdown();
        
        // Returns the Drive System Manager of the Sentry.  
        TB9051FTG get_drive_system();
        
        void loop();
        
        ConnectivityManager _get_manager() { return _communication_system; }
        
        void showTaskMemoryUsage();
        void test();
        void testComms();
        void test_bme_data_to_serial();
        void test_bme_data_to_firebase();
        void test_mic_data_to_firebase();
        void test_bme_and_mic_data_to_firebase();
        void test_mic();
        void test_US();
        void test_motor();
        void test_connection_to_firebase();
};

#endif /* DEVICE_H */