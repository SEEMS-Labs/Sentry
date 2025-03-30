#include "HCSR04.h"
#include <Arduino.h>

/**
 * Initializes the sensor pin connections wrt the ESP32 and enables sensor.
 */
void HCSR04::init() {
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);
    enable();
}

/**
 * Poll the sensor and store the data.
 * @return True if the reading was successful, false otherwise.
 */
bool HCSR04::readSensor(TickType_t xMaxBlockTime) {
    
    // Ensure sensor is active before reading.
    bool res = false;
    if(!active) return res;

    // Pulse trigger for 10 us.
    pulseTrigger();
    
    // Wait for pulse to complete.
    uint32_t pulseFinishedEvent = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);

    // Compute distance and store.
    if(pulseFinishedEvent != 0) {
        float inches = computeInches();
        if(distIndex == bufferSize) distIndex = 0;
        pastDistances[distIndex++] = inches;
        res = true;
    }

    // Return.
    return res;
}

/**
 * Mark a sensor as relevant for output collection.
 */
void HCSR04::enable() {
    active = true;
}

/**
 * Mark a sensor as irrelevant for output collection.
 */
void HCSR04::disable() {
    active = false;
}

/**
 * Set this ultrasonic sensors human presence estimation threshold.
 * @param preferences Access to Sentry NVM (permanent memory).
 */
void HCSR04::setHpeThreshold(Preferences preferences) {

    // Try grabbing data from preferences if sentry is starting up.
    bool bootFromStart = StateManager::getManager()->getSentrySensorThresholdState() == ThresholdState::ts_PRE_STARTUP;

    // Set sensor thresholds and update preferences.
    if(!bootFromStart) setHpeThresholdAndUpdatePreferences(preferences);
    else setHpeThresholdFromPreferences(preferences);

}

/**
 * Set this ultrasonic sensors human presence estimation threshold from 
 * the preferences stored on the Sentry.
 * @param preferences Access to Sentry NVM (permanent memory).
 */
void HCSR04::setHpeThresholdFromPreferences(Preferences preferences) {
    
    if(USE_HPE_DEF_THD == 1) {
        Serial.println("Using the HPE Threshold set in code.");
        presenceDetectionThreshold = DEF_HP_EST_LIM;
        return;
    }

    Serial.println("Checking if Custom HCSR04 HPE Thresholds Exist.");
    
    // Create preferences namespace in read mode.
    preferences.begin(PREF_SENSOR_THDS, true);
    
    // Check for intial exisitence of bme thresholds (if this doesn't exist, this is the first Sentry init).
    bool isInitialBoot = !preferences.isKey(PREF_HPE_THD);

    // Set thresholds to values found in preferences if the key did exist.
    if(isInitialBoot == false) {
        Serial.println("Custom HPE Threshold did exist. Setting custom limit.");
        presenceDetectionThreshold = preferences.getFloat(PREF_HPE_THD, DEF_HP_EST_LIM);
        
    }
    else Serial.println("Custom HPE Threshold did not exist. Retaining default limit.");
    
    // End the preferences namespace and return.
    preferences.end();
}

/**
 * Set thisultrasonic sensors human presence estimation threshold from 
 * the global threshold info packet and update the preferences stored on 
 * sentry to reflect this change.
 * @param preferences Access to Sentry NVM (permanent memory).
 */
void HCSR04::setHpeThresholdAndUpdatePreferences(Preferences preferences) {
    // Grab the new threshold.
    float newLimit = sensorManager->getUserSentryConfigDataPacket()->userPresenceEstimationThreshold;
    Serial.printf("US Threshold [%f -> %f]\n", presenceDetectionThreshold, newLimit);

    // Check to see if update should be made.
    bool limitChanged = (presenceDetectionThreshold != newLimit);
    bool newLimitInBounds = (newLimit <= MAX_HPE) && (newLimit >= MIN_HPE);

    // Set the new HPE limit if required.
    if(limitChanged && newLimitInBounds) {
        presenceDetectionThreshold = newLimit;
        preferences.begin(PREF_SENSOR_THDS, false);
        preferences.putFloat(PREF_HPE_THD, newLimit);
        preferences.end();
    }
    
}

/**
 * Signal that this ultrasonic sensor has passed one or both of its 2 thresholds.
 * @return A byte where the Bit 0 (LSB) represents ths obstacle detection threshold 
 * and Bit 1 represents the presence detection threshold. A bit set to 1 indicates 
 * the distance threshold has been breached and a bit set to 0 indicates the opposite.
 */
char HCSR04::passedThreshold() {
    // Grab the last measured distance.
    char flag = 0x00;
    if(this->active) {
        float distance = pastDistances[distIndex - 1];
        if(distance <= obstacleDetectionThreshold) flag |= OBSTACLE_THRESHOLD_BREACHED;
        if(distance <= presenceDetectionThreshold) flag |= PRESENCE_THRESHOLD_BREACHED;
    }
    return flag;
}

/**
 * Take the avarage of this sensors past distances buffer.
 * @return The average value of this sensors past distances.
 */
float HCSR04::averageBuffer() {
    float average = 0;
    for(int i = 0; i < bufferSize; i++) average += pastDistances[i];
    return average/bufferSize;
}

/**
 * Retrieve this ultrasonic sensors id.
 * @return The id number of this ultrasonic sensor.
 */
int HCSR04::identify() {
    return id;
}

/**
 * Check if this sensor is active or not.
 * @return True if sensor is active, false otherwise.
 */
bool HCSR04::isActive() {
    return active;
}

/**
 * Set this sensors obstacle detection threshold.
 * @param threshold The new threshold to be integrated.
 */
void HCSR04::setObstacleDetectionThreshold(float threshold) {
    obstacleDetectionThreshold = threshold;
}

/**
 * Retrieve this sensors obstacle detection threshold.
 */
float HCSR04::getObstacleDetectionThreshold() {
    return obstacleDetectionThreshold;
}

/**
 * Retrieve this sensors human presence threshold.
 */
float HCSR04::getHpeThreshold() { return presenceDetectionThreshold; }

/**
 * Pulse this ultrasonic sensors trigger pin to initiate measurements.
 */
void HCSR04::pulseTrigger() {
    // Pulse trigger for 10 us.
    digitalWrite(trigger, LOW);
    delayMicroseconds(5);
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);
}

float HCSR04::computeInches() {
    float isrPulseDuration = (isrPulseEnd - isrPulseStart) * 1.0;
    float distanceInInches = (isrPulseDuration/2) / 74;
    return distanceInInches;
}

void HCSR04::setISRStartPulse(ulong start) {
    isrPulseStart = start;
}

void HCSR04::setIRSEndPulse(ulong end) {
    isrPulseEnd = end;
}

float HCSR04::getDistanceReading() { 
    float res = -1;
    if(!active) return res;
    if(distIndex > 0) res = pastDistances[distIndex - 1]; 
    else res = pastDistances[bufferSize - 1]; // Index should get the last element in the buffer.
    return res;
}