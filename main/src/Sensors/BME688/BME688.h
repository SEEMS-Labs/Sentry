
// Include gaurd.
#ifndef BME688_H
#define BME688_H

// Grab libraries. 
#include "SensorInterface.h"
#include <Arduino.h>

class BME688 : public SensorInterface {

    private:

    public:
        /**
         * Initializes the sensor pin connections wrt the ESP32 and enables sensor.
         */
        void init();

        /**
         * Poll the sensor and store the data.
         */
        float readSensor(TickType_t xMaxBlockTime);

        /**
         * Mark a sensor as relevant for output collection.
         */
        void enable();

        /**
         * Mark a sensor as irrelevant for output collection.
         */
        void disable();

        /**
         * Signal that a sensor has passed its threshold(s).
         * @return A byte where each bit set to 1 represents a sensor threshold that has been passed
         * and each bit set to 0 the opposite.
         */
        uint8_t passedThreshold();

        /**
         * Take the avarage of a buffer that holds past sensor data.
         * @return The average value of a buffer which stores recent past sensor data.
         */
        float averageBuffer();
};

// End include gaurd.
#endif /* BME688.h */