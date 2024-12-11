// Include guard.
#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

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
         * Defines the sensors pins and connects them to the ESP32, and sets the thresholds of a sensor.
         * @param type the type of sensor being created.
         * @param pins An array of the pin numbers that the sensor is attached to on the ESP32.
         * @param thresholds[] An array holding all the thresholds relating to the Sensor.
         */
        virtual void init() = 0;

        /**
         * Poll the sensor and store the data.
         */
        virtual void readSensor() = 0;

        /**
         * Mark a sensor as relevant for output collection.
         */
        virtual void enable() = 0;

        /**
         * Mark a sensor as irrelevant for output collection.
         */
        virtual void disable() = 0;

        /**
         * Signal that a sensor has passed its threshold(s) so that action can be taken.
         * @return True if the sensor has passed its threshold, false othewise.
         */
        virtual bool passedThreshold() = 0;

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