
// Include gaurd.
#ifndef BME688_H
#define BME688_H

// Grab libraries. 
#include "SensorInterface.h"
#include "Sentry/main/src/StateManager.h"
#pragma once
#include "Sentry/main/src/Sensors/SensorManager.h"
#include "Sentry/main/src/sentryConfigInfo.h"
#include <Arduino.h>
#include <bsec.h>
#include <Wire.h>
#include <Preferences.h>

// Forward definition of SensorManager.
class SensorManager;

/**
 * Class that represents the BME688 Environmental Sensor Module 
 * that serves as the core internal monitoring sensor of the Sentry.
 * Several Virtual Sensor's are available for measuring different
 * parameters. Currently only 5 are enabled: AQI, CO2, Pressure,
 * Temperature, and Humidity. 
 */
class BME688 : public SensorInterface {

    private:
        /**
         * I2C Serial Clock Line.
         */
        const int sck;  

        /**
         * I2C Serial Data Line.
         */
        const int sdi;  

        /**
         * Keep state of the last sensor reading.
         */
        bool lastReadingSuccesful = true;

        /**
         * This sensor's activty status.
         */
        bool active = false;

        /**
         * BSEC BME688 representation.
         */
        Bsec sensor;    

        /**
         * Number of metrics measured by the BME688.
         */
        static constexpr uint8_t numSensors = 5;   
        
        /**
         * Flag indicating if the sensors buffers are filled with valid values.
         * Used to ignore the first <bufferSize> samples (minute of operation).
         */
        bool sensorWarmedUp = false;

        /**
         * Size of sensor past reading buffers.
         * The sensor takes 3s to read so 3*buffersize = response time (s).
         */
        static constexpr int bufferSize = 5;  

        /**
         * Current virtual sensor that this Sensor is working with for data purposes.
         * Specifically to be used within the averageBuffer and passedTrheshold methods
         * for selection purposes. This attribute should be set before choosing a buffer to average.
         */
        bsec_virtual_sensor_t currentVirtualSensor = BSEC_OUTPUT_IAQ;

        int iaqIndex = 0;                   // IAQ history buffer index.
        float pastIAQ[bufferSize];          // IAQ history buffer.
        float iaqThreshold = DEF_AQI_LIM;   // IAQ threshold.

        int co2Index = 0;                   // CO2 history buffer index.
        float pastCO2[bufferSize];          // CO2 history buffer.
        //float co2Threshold = DEF_CO2_LIM;   // CO2 Threshold.

        int pressureIndex = 0;                  // Pressure history buffer index.
        float pastPressure[bufferSize];         // Pressure history buffer.
        float pressureThreshold = DEF_PRES_LIM; // Pressure threshold.

        int tempIndex = 0;                  // Temperature history buffer index.
        float pastTemp[bufferSize];         // Temperature history buffer.
        float tempThreshold = DEF_TEMP_LIM; // Temperature threshold.

        int humIndex = 0;                   // Humidity history buffer index.
        float pastHumidity[bufferSize];     // Humidity history buffer.
        float humThreshold = DEF_HUM_LIM;   // Humidity threshold.

        /**
         * BSEC BME688 output metrics (virtual sensors) list.
         */
        bsec_virtual_sensor_t sensorList[numSensors] = {
            BSEC_OUTPUT_IAQ,
            BSEC_OUTPUT_CO2_EQUIVALENT,
            BSEC_OUTPUT_RAW_PRESSURE,
            BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
            BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY
        };

        SensorManager *sensorManager;
        void setThresholdFromPreferences(Preferences preferences);
        void setThresholdAndUpdatePreferences(Preferences preferences);
        
    public:
        /**
         * Defines the sensors pins I2C pins for initialization.
         * @param sdi Serial Data In.
         * @param sck Serial Clock.
         * @param sensorManager Pointer to this BME688's overseer.
         */
        BME688(int sdi, int sck, SensorManager *sensorManager) : 
            sdi(sdi), 
            sck(sck), 
            sensorManager(sensorManager) {}

        /**
         * Initializes the sensor pin connections wrt the ESP32 and enables sensor.
         */
        void init() override;

        /**
         * Poll the sensor and store the data.
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
         * Check if this sensor is active or not.
         * @return True if sensor is active, false otherwise.
         */
        bool isActive() override;

        /**
         * Set the thresholds of this BME688 from memory.
         * @param preferences Access to Sentry NVM (permanent memory).
         */
        void setThresholds(Preferences preferences);

        /**
         * Signal that this BME688 has passed its threshold(s).
         * @return A byte where each bit set to 1 represents a sensor threshold that has been passed
         * and each bit set to 0 the opposite. 
         * Bit 0: IAQ, 
         * Bit 1: CO2,
         * Bit 2: Pressure,
         * Bit 3: VOC,
         * Bit 4: Temperature,
         * Bit 5: Humidity
         */
        char passedThreshold() override;

        /**
         * Take the avarage of a buffer that holds past sensor data.
         * @return The average value of a buffer which stores recent past sensor data.
         */
        float averageBuffer() override;

        /**
         * Checks the BME status.
         * @return The status of the BME688 in terms of the bsec library.
        */
        bsec_library_return_t checkSensorStatus();

        /**
         * Get the last IAQ reading.
         * @return The last known IAQ reading.
         */
        float get_IAQ_reading();

        /**
         * Get the last CO2 reading.
         * @return The last known CO2 reading.
         */
        float get_CO2_reading();

        /**
         * Get the last Pressure reading.
         * @return The last known pressure reading.
         */
        float get_pressure_reading();

        /**
         * Get the last Temperature reading.
         * @return The last known temperature reading.
         */
        float get_temp_reading();

        /**
         * Get the last Humidity reading.
         * @return The last known humidity reading.
         */
        float get_humidity_reading();

        /**
         * Set the virtual sensor type for reading and measuring.
         */
        void setCurrentVirtualSensor(bsec_virtual_sensor_t virtualSensor);

        float getAqiThreshold();
        float getTempThreshold();
        float getHumThreshold();
        float getPresThreshold();
};

// End include gaurd.
#endif /* BME688.h */