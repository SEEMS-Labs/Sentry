// Include guard.
#ifndef MIC_H
#define MIC_H

#include "SensorInterface.h"

class Microphone : public SensorInterface {

    private:

        /**
         * Analog Input pin of this microphone.
         */
        int input;

        /**
         * Sampling Rate to be used in taking readings.
         */
        float samplingRate;

        /**
         * Buffer of past distances measured.
         */
        float noiseBuffer[50];

    public:
        /**
         * Defines the sensors pins and connects them to the ESP32, and sets the thresholds of a sensor.
         * @param type the type of sensor being created.
         * @param pins An array of the pin numbers that the sensor is attached to on the ESP32.
         * @param thresholds[] An array holding all the thresholds relating to the Sensor.
         */
        void init(SensorType type, int pins[], int thresholds[]) override;

        /**
         * Poll the sensor and store the data.
         */
        void readSensor() override;

        /**
         * Mark a sensor as relevant for output collection.
         */
        void enable() override;

        /**
         * Mark a sensor as irrelevant for output collection.
         */
        void disable() override;

        /**
         * Signal that a sensor has passed its threshold(s) so that action can be taken.
         * @return True if the sensor has passed its threshold, false othewise.
         */
        bool passedThreshold() override;

        /**
         * Take the avarage of a buffer that holds past sensor data.
         * @return The average value of a buffer which stores recent past sensor data.
         */
        float averageBuffer() override;

};

// End include guard.
#endif