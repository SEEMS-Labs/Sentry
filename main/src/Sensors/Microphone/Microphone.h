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

#define DB_PERCENT_DIFF 2      // Meaningful percent difference between current buffer average and sound level Threshold (in %).
#define DB_WEAK_PERCENT 3      // Percent difference between current and last buffer averages indicating weak threhsold crossing (in %).
#define DB_STRONG_PERCENT 7   // Percent difference between current and last buffer averages weakly strong presence (in %).

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

        /**
         * Hold's this microphones last buffer average relative to the most recent reading.
         */
        float lastBufferAverage = 0.0;

        /**
         * Flag indicating microphone activity status (Actively polling or not).
         */
        bool active = false;

        /**
         * Pointer to this microphone's sensor manager.
         */
        SensorManager *sensorManager;

        /**
         * Method to set mic noise level threshold from preferences. Typically
         * called on startup.
         */
        void setThresholdFromPreferences(Preferences preferences);

        /**
         * Method to set mic noise level threshold and update it in preferences.
         * Typically called post startup and of a user requests to change preferences.
         */
        void setThresholdAndUpdatePreferences(Preferences preferences);

        /**
         * Utility method to calculate decibels from measured millivolts value.
         */
        float calculateDecibelLevel();

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
        bool readSensor(TickType_t xMaxBlockTime = 0) override;

        /**
         * Mark a sensor as relevant for output collection.
         */
        void enable() override;

        /**
         * Mark a sensor as irrelevant for output collection.
         */
        void disable() override;

        /**
         * Check if this sensor is active or not.
         * @return True if sensor is active, false otherwise.
         */
        bool isActive() override;

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
        float getLastAverageSoundLevel();
        

};

// End include guard.
#endif /*Microphone.h*/