// Include guard.
#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#include <Arduino.h>

/**
 * Interace for all sensors to be used in SEEMS.
 */
class SensorInterface {
    public:

        /**
         * Initializes the sensor pin connections wrt the ESP32 and enables sensor.
         */
        virtual void init() = 0;

        /**
         * Poll the sensor and store the data. This must be called from a task.
         * @param xMaxBlockTime The maximum total time a sensor reading should be attempted before timeout.
         */
        virtual bool readSensor(TickType_t xMaxBlockTime) = 0;

        /**
         * Mark a sensor as relevant for output collection.
         */
        virtual void enable() = 0;

        /**
         * Mark a sensor as irrelevant for output collection.
         */
        virtual void disable() = 0;

        /**
         * Signal that a sensor has passed its threshold(s).
         * @return A byte where each bit set to 1 represents a sensor threshold that has been passed
         * and each bit set to 0 the opposite.
         */
        virtual char passedThreshold() = 0;

        /**
         * Take the avarage of a buffer that holds past sensor data.
         * @return The average value of a buffer which stores recent past sensor data.
         */
        virtual float averageBuffer() = 0;

        // Defaul destructor.
        virtual ~SensorInterface() = default;
};

// End include guard.
#endif