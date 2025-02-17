#include "BME688.h"

/**
 * Initializes the BME I2C pins and enables the sensor.
 */
void BME688::init() {
    // Start I2C.
    Wire.setPins(sdi, sck);
    Wire.begin();

    // Start BME688.
    sensor.begin(BME68X_I2C_ADDR_HIGH, Wire);
    enable();
}

/**
 * Poll each of the BMEs virtual sensors and store the data in the history buffers.
 * This must only be called from within a task.
 * @param xMaxBlockTime The maximum time allotted to the BME to get valid measurements.
 * @return True if the reading was successful, false otherwise.
 */
bool BME688::readSensor(TickType_t xMaxBlockTime) {
    
    // Enusre sensor is active before reading.
    bool res = false;
    if(!active) return res;

    // Prompt the sensor to begin taking measurments.
    sensor.run();

    // Yield to system while waiting for reading.
    vTaskDelay(xMaxBlockTime);  // Should be 3-300 seconds.
    bool success = sensor.run();

    // Ensure reading was succesful.
    if(success) {
        // Store the readings in their respective buffers.
        if(iaqIndex == bufferSize) iaqIndex = 0;
        pastIAQ[iaqIndex++] = sensor.iaq;

        if(co2Index == bufferSize) co2Index = 0;
        pastCO2[co2Index++] = sensor.co2Equivalent;

        if(pressureIndex == bufferSize) pressureIndex = 0;
        pastPressure[pressureIndex++] = sensor.pressure;

        if(vocIndex == bufferSize) vocIndex = 0;
        pastVOC[vocIndex++] = sensor.breathVocEquivalent;

        if(tempIndex == bufferSize) tempIndex = 0;
        pastTemp[tempIndex++] = sensor.temperature;

        if(humIndex == bufferSize) humIndex  = 0;
        pastHumidity[humIndex++] = sensor.humidity;

        // Set the return results.
        lastReadingSuccesful = true;
        res = true;

        Serial.print("Pressure: ");
        Serial.print(sensor.pressure/100.0);
        Serial.println(" hPa");
        
        Serial.print("Temperature: ");
        Serial.print(sensor.temperature);
        Serial.println(" *C");
        
        Serial.print("Humidity: ");
        Serial.print(sensor.humidity);
        Serial.println(" %");
        
        Serial.print("IAQ: ");
        Serial.print(sensor.iaq);
        Serial.println(" Index");
        
        Serial.print("CO2 Equivalent: ");
        Serial.print(sensor.co2Equivalent);
        Serial.println(" PPM");
        
        Serial.print("Breath VOC Equivalent: ");
        Serial.print(sensor.breathVocEquivalent);
        Serial.println(" PPM");
        Serial.println();
    }
    else lastReadingSuccesful = false;

    // Return.
    return res;
}

/**
 * Mark this BME as relevant for data output collection.
 */
void BME688::enable() {
    // Only to be called in non-active state.
    if(active == true) return;
    active = true;
    
    // Set sensor to low power mode (takes 3 seconds to read).
    sensor.updateSubscription(sensorList, numSensors, BSEC_SAMPLE_RATE_LP);
}

/**
 * Mark this BME as irrelevant for data output collection.
 */
void BME688::disable() {
    // Only to be called in active state.
    if(active == false) return;
    active = false;
    
    // Set sensor to low power mode (takes 3 seconds to read).
    sensor.updateSubscription(sensorList, numSensors, BSEC_SAMPLE_RATE_DISABLED);
}

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
char BME688::passedThreshold() {
    
    // Calculate the past average of all buffers.
    char res = 0;
    setCurrentVirtualSensor(BSEC_OUTPUT_IAQ);
    float iaqAvg = averageBuffer();

    setCurrentVirtualSensor(BSEC_OUTPUT_CO2_EQUIVALENT);
    float co2Avg = averageBuffer();

    setCurrentVirtualSensor(BSEC_OUTPUT_RAW_PRESSURE);
    float presAvg = averageBuffer();

    setCurrentVirtualSensor(BSEC_OUTPUT_BREATH_VOC_EQUIVALENT);
    float vocAvg = averageBuffer();
    
    setCurrentVirtualSensor(BSEC_OUTPUT_RAW_TEMPERATURE);
    float tempAvg = averageBuffer();

    setCurrentVirtualSensor(BSEC_OUTPUT_RAW_HUMIDITY);
    float humAvg = averageBuffer();
    
    // Perform masking operation. 
    if(iaqAvg > DEF_AQI_LIM) res |= AQI_BREACHED_MASK;
    if(co2Avg > DEF_CO2_LIM) res |= CO2_BREACHED_MASK;
    if(presAvg > DEF_PRES_LIM) res |= PRESSURE_BREACHED_MASK;
    if(vocAvg > DEF_VOC_LIM) res |= VOC_BREACHED_MASK;
    if(tempAvg > DEF_TEMP_LIM) res |= TEMPERATURE_BREACHED_MASK;
    if(humAvg > DEF_HUM_LIM) res |= HUMIDITY_BREACHED_MASK;

    // Return.
    return res;
}

/**
 * Take the average of a virtual sensor's past measurements buffer. 
 * In order to get the average desired, the 'setCurrentVirtualSensor' 
 * method should be called with the desired sensor as a parameter. 
 * @return The average value of the past measurements buffer of 
 * the currently specified virtual sensor.
 */
float BME688::averageBuffer() {

    // Calculate the average of the pre selected buffer.
    float average = 0.0;
    switch (currentVirtualSensor) {
        case BSEC_OUTPUT_IAQ :
            for(int i = 0; i < bufferSize; i++) average += pastIAQ[i];
            break;
        
        case BSEC_OUTPUT_CO2_EQUIVALENT :
            for(int i = 0; i < bufferSize; i++) average += pastCO2[i];
            break;

        case BSEC_OUTPUT_RAW_PRESSURE :
            for(int i = 0; i < bufferSize; i++) average += pastPressure[i];
            break;

        case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT :
            for(int i = 0; i < bufferSize; i++) average += pastVOC[i];
            break;

        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE :
            for(int i = 0; i < bufferSize; i++) average += pastTemp[i];
            break;

        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY :
            for(int i = 0; i < bufferSize; i++) average += pastHumidity[i];
            break;
        
        default :
            average = -1.0;
            break;
    }

    // Return.
    average /= bufferSize;
    return average;
}

/**
 * Checks the BME status.
 * @return The status of the BME688 in terms of the bsec library.
*/
bsec_library_return_t BME688::checkSensorStatus() { return sensor.bsecStatus; }

/**
 * Get the last IAQ reading.
 * @return The last known IAQ reading.
 */
float BME688::get_IAQ_reading() { 
    if(iaqIndex > 0) return pastIAQ[iaqIndex - 1]; 
    else return pastIAQ[bufferSize - 1];
}

/**
 * Get the last CO2 reading.
 * @return The last known CO2 reading.
 */
float BME688::get_CO2_reading() { 
    if(co2Index > 0 ) return pastCO2[co2Index - 1]; 
    else return pastCO2[bufferSize - 1];
}

/**
 * Get the last Pressure reading.
 * @return The last known pressure reading.
 */
float BME688::get_pressure_reading() { 
    if(pressureIndex > 0) return pastPressure[pressureIndex - 1]; 
    else return pastPressure[bufferSize - 1];
}

/**
 * Get the last VOC reading.
 * @return The last known VOC reading.
 */
float BME688::get_VOC_reading() { 
    if(vocIndex > 0) return pastVOC[vocIndex - 1]; 
    else return pastVOC[bufferSize - 1];
}

/**
 * Get the last Temperature reading.
 * @return The last known temperature reading.
 */
float BME688::get_temp_reading() { 
    if(tempIndex > 0) return pastTemp[tempIndex - 1]; 
    else return pastTemp[bufferSize - 1];
}

/**
 * Get the last Humidity reading.
 * @return The last known humidity reading.
 */
float BME688::get_humidity_reading() { 
    if(humIndex > 0) return pastHumidity[humIndex - 1]; 
    else return pastHumidity[bufferSize - 1];
}

/**
 * Choose between the virtual sensors this BME is reading from. This should 
 * be called before any attempt to check if thresholds were passed of to 
 * retrieve the average value of a sensors past measurements buffer.
 */
void BME688::setCurrentVirtualSensor(bsec_virtual_sensor_t virtualSensor) { currentVirtualSensor = virtualSensor; }