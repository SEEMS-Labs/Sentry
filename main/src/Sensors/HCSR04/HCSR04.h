// Include guard.
#ifndef HCSR04_H
#define HCSR04_H

// Grab libraries. 
#include "SensorInterface.h"
#include "Sentry/main/src/sentryConfigInfo.h"
#include "Sentry/main/src/StateManager.h"
#pragma once
#include "Sentry/main/src/Sensors/SensorManager.h"
#include <Arduino.h>
#include <Preferences.h>

#define HPE_PERCENT_DIFF 2      // Meaningful percent difference between current buffer average and HPE Threshold (in %).
#define HPE_WEAK_PERCENT 5      // Percent difference between current and last buffer averages weakly indicating presence (in %).
#define HPE_STRONG_PERCENT 10   // Percent difference between current and last buffer averages strongly indicating presence (in %).

// Forward definition of SensorManager.
class SensorManager;

/**
 * Class representing the HC-SR04 Ultrasonic Sensors used as obstacle and presence detectors.
 */
class HCSR04 : public SensorInterface {

    private:
        /**
         * Field that serves as an identifer for this HC-SR04.
         */
        const int id;
        
        /**
         * Trigger pin of this HC-SR04.
         */
        const int trigger;

        /**
         * Echo pin of this HC-SR04.
         */
        const int echo;

        /**
         * The distance from the sensor (in inches) that an obstacle must be to be "detected".
         */
        float obstacleDetectionThreshold = -1.0;

        /**
         * The distance from the sensor (in inches) that an object/person must be to be "detected".
         */
        float presenceDetectionThreshold = DEF_HP_EST_LIM;

        /**
         * Quantifies if this sensor is on or not (should be polled or not).
         */
        bool active = false;

        /**
         * Index into the distance buffer pointing to the last distance measured.
         */
        int distIndex = 0;
        
        /**
         * Last average of the buffer.
         */
        float lastBufferAverage = 0;

        /**
         * Size of distance buffer.
         */
        static constexpr int bufferSize = 5;

        /**
         * Buffer of past distances measured.
         */
        float pastDistances[bufferSize];

        unsigned long isrPulseStart = 0; // Stores the time at which the sensor's echo has begun from ISR.
        unsigned long isrPulseEnd = 0;   // Stores the time at which the sensor's echo has finished from ISR.    

        /**
         * Pulse this ultrasonic sensor's trigger pin to initiate a measurement.
         */
        void pulseTrigger();

        /**
         * Compute the distance in inches measured by the sensor.
         */
        float computeInches();

        SensorManager *sensorManager;
        void setHpeThresholdFromPreferences(Preferences preferences);
        void setHpeThresholdAndUpdatePreferences(Preferences preferences);

    public:
        /**
         * Defines the sensors pins and connects them to the ESP32, and sets the thresholds of a sensor.
         * @param trigger The trigger pin of this sensor.
         * @param echo The echo pin of this sensor.
         * @param id The unique Id for this ultrasonic sensor.
         * @param obstacleDetectionThreshold The distance from the sensor (in inches) that an obstacle must be to be "detected".
         */
        HCSR04(int trigger, int echo, int id, int obstacleDetectionThreshold, SensorManager *sensorManager) : 
            trigger(trigger), 
            echo(echo), 
            id(id), 
            obstacleDetectionThreshold(obstacleDetectionThreshold), 
            sensorManager(sensorManager) {};

        /**
         * Initializes the sensor pin connections wrt the ESP32 and enables sensor.
         */
        void init() override;

        /**
         * Poll the sensor and store the data. This must only be called from within a task.
         * @param xMaxBlockTime The maximum time allotted to read a sensor.
         * @return True if the reading was successful, false otherwise.
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
         * Retrieve this sensors Human presence estimation threshold.
         */
        float getHpeThreshold();

        /**
         * Set this ultrasonic sensors human presence estimation threshold.
         * @param preferences Access to Sentry NVM (permanent memory).
         */
        void setHpeThreshold(Preferences preferences);

        /**
         * Signal that this ultrasonic sensor has passed one or both of its 2 thresholds.
         * @return A byte where the least two significant bits represent detection threshold
         * breaches and the next 3 represent the strength of the breach.
         * Bit 0: Obstacle detection,
         * Bit 1: Human presence estimation,
         * Bit 2: Strong breach (certain that the barrier has been broken).
         * Bit 3: Moderate breach.
         * Bit 4: Weak breach.
         */
        char passedThreshold() override;

        /**
         * Take the avarage of this sensors past distances buffer.
         * @return The average value of this sensors past distnaces.
         */
        float averageBuffer() override;

        /**
         * Retrieve this ultrasonic sensors id.
         * @return The id number of this ultrasonic sensor.
         */
        int identify();

        /**
         * Check if this sensor is active or not.
         * @return True if sensor is active, false otherwise.
         */
        bool isActive() override;

        /**
         * Set this sensors obstacle detection threshold.
         * @param threshold The new threshold to be integrated.
         */
        void setObstacleDetectionThreshold(float threshold);

        /**
         * Retrieve this sensors obstacle detection threshold.
         */
        float getObstacleDetectionThreshold();

        /**
         * For purposes of sensor reading in the appropriate ISR from the Sensor Manager.
         * Sets the start time of the echo pulse once it begins.
         * @param start This is the start time of the echo pulse once this sensor has begun measuring.
         */
        void setISRStartPulse(ulong start);

        /**
         * For purposes of sensor reading in the appropriate ISR from the Sensor Manager.
         * Sets the end time of the echo pulse once it begins.
         * @param end This is the end time of the echo pulse once this sensor has begun measuring.
         */
        void setIRSEndPulse(ulong end);

        /**
         * Get the last distance reading.
         * @return The last known distance reading from this sensor.
         */
        float getDistanceReading();

        float getLastBufferAverage();
};

// End include guard.
#endif /*HCSR04.h*/