// Include guard.
#ifndef MIC_H
#define MIC_H

#include "SensorInterface.h"
#include "Sentry/main/src/sentryConfigInfo.h"
#include "Sentry/main/src/StateManager.h"
#pragma once
#include "Sentry/main/src/Sensors/SensorManager.h"
#include <Arduino.h>
#include <Preferences.h>

// Forward definition of SensorManager.
class SensorManager;

class Microphone : public SensorInterface {

    private:

        /**
         * Analog Output pin of this microphone.
         */
        int output;

        /**
         * The noise threshold for this microphone.
         */
        float noiseThreshold = DEF_NOISE_LIM;

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
        SensorManager *sensorManager;
        void setThresholdFromPreferences(Preferences preferences);
        void setThresholdAndUpdatePreferences(Preferences preferences);

    public:
        /**
         * Defines the sensors pins and connects them to the ESP32, and sets the noise threshold.
         * @param output The analog output pin of this microphone.
         * @param sensorManager The manager that oversees this microphone.
         */
        Microphone(int output, SensorManager *sensorManager) : 
            output(output),
            sensorManager(sensorManager) {};

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
         * Set this microphone's dbSPL threshold.
         * @param preferences Access to Sentry NVM (permanent memory).
         */
        void setThreshold(Preferences preferences);

        /**
         * Signal that a sensor has passed its threshold(s) so that action can be taken.
         * @return True if the sensor has passed its threshold, false othewise.
         */
        char passedThreshold() override;

        /**
         * Signal that a sensor has passed its threshold(s) so that action can be taken.
         * @return A byte reading 0xFF if the sensor has passed its threshold, 0x00 othewise.
         */
        float averageBuffer() override;

        float rmsBuffer();
        float getLastSoundLevelReading();
        float getThreshold();
        

};

// End include guard.
#endif /*Microphone.h*/