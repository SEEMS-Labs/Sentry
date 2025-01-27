// Include guard.
#ifndef HCSR04_H
#define HCSR04_H

// Grab libraries. 
#include "SensorInterface.h"
#include "Sentry/main/src/sentryConfigInfo.h"
#include <Arduino.h>

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
        float obstacleDetectionThreshold;

        /**
         * The distance from the sensor (in inches) that an object/person must be to be "detected".
         */
        float presenceDetectionThreshold = 24;

        /**
         * Quantifies if this sensor is on or not (should be polled or not).
         */
        bool active = false;

        /**
         * Index into the distance buffer pointing to the last distance measured.
         */
        int distIndex = 0;
        
        /**
         * Size of distance buffer.
         */
        static constexpr int bufferSize = 256;

        /**
         * Buffer of past distances measured.
         */
        float pastDistances[bufferSize];

        unsigned long isrPulseStart = -1; // Stores the time at which the sensor's echo has begun from ISR.
        unsigned long isrPulseEnd = -1;   // Stores the time at which the sensor's echo has finished from ISR.    

        /**
         * Pulse this ultrasonic sensor's trigger pin to initiate a measurement.
         */
        void pulseTrigger();

        /**
         * Compute the distance in inches measured by the sensor.
         */
        float computeInches();

    public:
        /**
         * Defines the sensors pins and connects them to the ESP32, and sets the thresholds of a sensor.
         * @param trigger The trigger pin of this sensor.
         * @param echo The echo pin of this sensor.
         * @param id The unique Id for this ultrasonic sensor.
         * @param obstacleDetectionThreshold The distance from the sensor (in inches) that an obstacle must be to be "detected".
         */
        HCSR04(int trigger, int echo, int id, int obstacleDetectionThreshold) : trigger(trigger), echo(echo), id(id), obstacleDetectionThreshold(obstacleDetectionThreshold) {};

        /**
         * Initializes the sensor pin connections wrt the ESP32 and enables sensor.
         */
        void init() override;

        /**
         * Poll the sensor and store the data. This must only be called from within a task.
         * @param xMaxBlockTime The maximum time allotted to read a sensor.
         */
        float readSensor(TickType_t xMaxBlockTime) override;

        /**
         * Mark a sensor as relevant for output collection.
         */
        void enable() override;

        /**
         * Mark a sensor as irrelevant for output collection.
         */
        void disable() override;

        /**
         * Signal that this ultrasonic sensor has passed one or both of its 2 thresholds.
         * @return A byte where the Bit 7 (MSB) represents ths obstacle detection threshold 
         * and Bit 6 representes the presence detection threshold. A bit set to 1 indicates 
         * the distance threshold has been breached and a bit set to 0 indicates the opposite.
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
        bool isActive();

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
};

// End include guard.
#endif /*HCSR04.h*/