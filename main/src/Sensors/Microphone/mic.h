// Include guard.
#ifndef MIC_H
#define MIC_H

#include "SensorInterface.h"
#include <Arduino.h>

class Microphone : public SensorInterface {

    private:

        /**
         * Analog Output pin of this microphone.
         */
        int output;

        /**
         * The noise threshold for this microphone.
         */
        float noiseThreshold;

        /**
         * Sampling Rate to be used in taking readings.
         */
        float samplingRate;

        /**
         * Index into the sound buffer pointing to the last decibel measured.
         */
        int noiseIndex = 0;
        
        /**
         * Size of noise buffer.
         */
        static constexpr int bufferSize = 10;

        /**
         * Buffer of past noise levels measured.
         */
        float noiseBuffer[bufferSize];

        bool active = false;

    public:
        /**
         * Defines the sensors pins and connects them to the ESP32, and sets the noise threshold.
         * @param output The analog output pin of this microphone.
         * @param noisethreshold The noise threshold of this microphone.
         */
        Microphone(int output) : output(output) {};

        /**
         * Initializes the sensor pin connections wrt the ESP32 and enables sensor.
         */
        void init() override;

        /**
         * Poll the sensor and store the data.
         */
        bool readSensor(TickType_t xMaxBlockTime) override;

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
        char passedThreshold() override;

        /**
         * Take the avarage of a buffer that holds past sensor data.
         * @return The average value of a buffer which stores recent past sensor data.
         */
        float averageBuffer() override;

        float rmsBuffer();
        float getLastSoundLevelReading();
        

};

// End include guard.
#endif /*mic.h*/