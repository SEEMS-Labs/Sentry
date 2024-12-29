// Include guard.
#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#include <cstdint>
#include <Arduino.h>

/**
 * Specifies type of sensor being dealt with.
 */
enum class SensorType {
    /**
     * Ultrasonic Sensor for presence detection.
     */
    ULTRASONIC, 

    /**
     * Noise Sensor/microphone for noise detection.
     */
    NOISE,

    /**
     * Environmental Sensor for measuring humidity, temperature, air quality, and pressure.
     */
    ENVIRONMENTAL
};

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
        virtual float readSensor(TickType_t xMaxBlockTime) = 0;

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
        virtual uint8_t passedThreshold() = 0;

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