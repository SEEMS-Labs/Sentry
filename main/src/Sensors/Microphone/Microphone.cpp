#include "Microphone.h"

/**
 * Initializes the sensor pin connections wrt the ESP32 and enables sensor.
 */
void Microphone::init() {
    analogReadResolution(12);
    enable();
}

/**
 * Poll the sensor and store the data.
 */
bool Microphone::readSensor(TickType_t xMaxBlockTime) {
    bool res = false;

    // Don't read inactive mic.
    if(!active) return res;

    // Take analog reading.
    int _adc_reading = analogRead(output);
    //float decibels_measured = (3.3)*(_adc_reading/4095.0) * 50;
    float decibels_measured = analogReadMilliVolts(output) * 50.0/1000.0;

    // Store.
    if(noiseIndex == bufferSize) noiseIndex = 0;
    noiseBuffer[noiseIndex++] = decibels_measured;
        
    // Return.
    res = true;
    return res;
}

/**
 * Mark a sensor as relevant for output collection.
 */
void Microphone::enable() {
    active = true;
}

/**
 * Mark a sensor as irrelevant for output collection.
 */
void Microphone::disable() {
    active = false;
}

/**
 * Set this microphone's db SPL threshold.
 * @param preferences Access to Sentry NVM (permanent memory).
 */
void Microphone::setThreshold(Preferences preferences) {
    
    // Try grabbing data from preferences if sentry is starting up.
    if(StateManager::getManager()->getSentrySensorThresholdState() == ThresholdState::ts_PRE_STARTUP) 
        setThresholdFromPreferences(preferences);
    
    // Set sensor thresholds and update preferences.
    else setThresholdAndUpdatePreferences(preferences);

}

/**
 * Set this microphone's db SPL threshold from the preferences stored on the Sentry.
 * @param preferences Access to Sentry NVM (permanent memory).
 */
void Microphone::setThresholdFromPreferences(Preferences preferences) {
    Serial.println("Checking if Custom Mic Threshold Exist.");

    // Create preferences namespace in read mode.
    preferences.begin(PREF_SENSOR_THDS, true);
    
    // Check for intial exisitence of bme thresholds (if this doesn't exist, this is the first Sentry init).
    bool isInitialBoot = !preferences.isKey(PREF_SPL_THD);

    // Set thresholds to values found in preferences if the key did exist.
    if(isInitialBoot == false) {
        Serial.println("Custom Mic Threshold did exist. Setting custom limit.");
        noiseThreshold = preferences.getFloat(PREF_SPL_THD, DEF_NOISE_LIM);
        
    }
    else Serial.println("Custom Mic Threshold did not exist. Retaining default limit.");
    
    // End the preferences namespace and return.
    preferences.end();
}

/**
 * Set this microphone's db SPL threshold from the global threshold info packet
 * and update the preferences stored on sentry to reflect this change.
 * @param preferences Access to Sentry NVM (permanent memory).
 */
void Microphone::setThresholdAndUpdatePreferences(Preferences preferences) {
    // Grab the new mic threshold.
    float newLimit = sensorManager->getUserSentryConfigDataPacket()->userNoiseLevelThreshold;
    Serial.printf("Mic Threshold [%f -> %f]\n", noiseThreshold, newLimit);

    // Check to see if update should be made.
    bool limitChanged = (noiseThreshold != newLimit);
    bool newLimitInBounds = (newLimit <= MAX_NOISE) && (newLimit >= MIN_NOISE);

    // Update.
    if(limitChanged && newLimitInBounds) {
        Serial.println("Mic threshold change accepted.");
        noiseThreshold = newLimit;
        preferences.begin(PREF_SENSOR_THDS, false);
        preferences.putFloat(PREF_SPL_THD, newLimit);
        preferences.end();
    }
}

/**
 * Signal that a sensor has passed its threshold(s) so that action can be taken.
 * @return A byte reading 0xFF if the sensor has passed its threshold, 0x00 othewise.
 */
char Microphone::passedThreshold() {
    float res = averageBuffer();
    return (res > noiseThreshold) ? 0xFF : 0x00;
}

/**
 * Take the avarage of a buffer that holds past sensor data.
 * @return The average value of a buffer which stores recent past sensor data.
 */
float Microphone::averageBuffer() {
    float avg = 0.0;
    for(int i = 0; i < bufferSize; i++) avg += noiseBuffer[i];
    return avg/bufferSize;
}

float Microphone::rmsBuffer(){
    return -1.0;
}

float Microphone::getLastSoundLevelReading() {
    if(noiseIndex > 0) return noiseBuffer[noiseIndex - 1]; 
    else return noiseBuffer[bufferSize - 1];
}

float Microphone::getThreshold() { return noiseThreshold; }