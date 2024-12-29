
#ifndef DEVICE_H
#define DEVICE_H

#include "SensorManager.h"
#include "DRV8833.h"

// Default "speed" of the Sentry based on testing and PWM duty cycle (~78%).
#define DEFAULT_SPEED 200

/**
 * This class represents the Sentry as a whole. It is from here the Sentry will be initialized and controlled.
 */
class Device {
    private: 
       SensorManager sensors;   // Sensor Management Unit.
       DRV8833 driver;          // Drive System Management Unit.

    public:
        // Start the Sentry.
        void begin();     

        // Unimplemented. Light Sleep mode for the Sentry.         
        void sleep_mode_1();

        // Unimplemented. Deeper Sleep mode for the Sentry.    
        void sleep_mode_2();

        // Unimplemented. Shuts Sentry Down during a dedicated shutdown.    
        void shutdown();
        
        // Returns the Drive System Manager of the Sentry.  
        DRV8833 getDriver();
};

#endif /* DEVICE_H */