
#ifndef DEVICE_H
#define DEVICE_H

#include "Sentry/main/src/sentryConfigInfo.h"
#include "Sentry/main/src/StateManager.h"
#include "SensorManager.h"
#include "ConnectivityManager.h"
#include "TB9051FTG.h"

// Default "speed" of the Sentry based on testing and PWM duty cycle (~78%).
#define DEFAULT_SPEED 200

/**
 * This class represents the Sentry as a whole. It is from here the Sentry will be initialized and controlled.
 */
class Device {
    private: 
        StateManager *_stateManager;                    // Sentry State Manager.
        ConnectivityManager _communication_system;      // Network Connectivity Management Unit.
        SensorManager _sensor_system;                   // Sensor Management Unit.
        TB9051FTG _drive_system;                        // Drive System Management Unit.

    public:
        Device( 
            SensorData &envData,                                    // Address to the global environmental data packet.
            Alerts &envStatus,                                      // Address to the global alerts data packet.
            UserSentryConfig &userConfiguration,                    // Address to the global custom user sentry configuration data packet.
            UserDriveCommands &userMovementCommands) :              // Address to the global user movement commands data packet.
            _communication_system(envData, envStatus, userConfiguration, userMovementCommands),
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

        void testComms();
};

#endif /* DEVICE_H */