// Include guard.
#ifndef HCSR04_H
#define HCSR04_H

#include "SensorInterface.h"

class HCSR04 : public SensorInterface {

    private:
        /**
         * Field that serves as an identifer for this HC-SR04.
         */
        int id;
        
        /**
         * Trigger pin of this HC-SR04.
         */
        int trigger;

        /**
         * Echo pin of this HC-SR04.
         */
        int echo;

        /**
         * Quantifies if this sensor is on or not (should be polled or not).
         */
        bool active;

        /**
         * Buffer of past distances measured.
         */
        float pastDistances[256];

    public:
        /**
         * Defines the sensors pins and connects them to the ESP32, and sets the thresholds of a sensor.
         * @param trigger The trigger pin of this sensor.
         * @param echo The echo pin of this sensor.
         * @param id The unique Id for this ultrasonic sensor.
         */
        HCSR04(int trigger, int echo, int id);

        /**
         * Initializes the sensor's pin connections.
         */
        void init() override;

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