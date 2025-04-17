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
    if(pulseFinishedEvent != 0) {
        // Compute distance just measured.
        float inches = computeInches();

        // Store the last average for later comparisons.
        lastBufferAverage = averageBuffer();

        // Update the buffer.
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
    taskENTER_CRITICAL(&preferencesMutex);
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
    taskEXIT_CRITICAL(&preferencesMutex);
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
        Serial.printf("US Temp threshold change accepted.\n");
        presenceDetectionThreshold = newLimit;
        taskENTER_CRITICAL(&preferencesMutex);
        preferences.begin(PREF_SENSOR_THDS, false);
        preferences.putFloat(PREF_HPE_THD, newLimit);
        preferences.end();
        taskEXIT_CRITICAL(&preferencesMutex);
    }
    
}

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
char HCSR04::passedThreshold() {
    char flag = 0x00;

    // Only check thresholds if sensor is active.
    if(this->active == false) return flag;

    // Check obstacle detection threshold. (This is checked against most recent distance instead of the buffers).
    if(getDistanceReading() <= obstacleDetectionThreshold) flag |= OBSTACLE_THRESHOLD_BREACHED;

    // Check human presence estimation threshold.
    float currBufferAvg = averageBuffer();
    float HpeCheck = presenceDetectionThreshold/currBufferAvg - 1;
    if(HpeCheck >= HPE_PERCENT_DIFF/100.0) {
        // Set the bit indicating human presence was detected.
        flag |= PRESENCE_THRESHOLD_BREACHED;

        // Check the difference between current and last buffer averages to determine the strength/confidence of presence.
        float bufferPercentDiff = abs(currBufferAvg - lastBufferAverage)/lastBufferAverage;

        // Greater than a 10% difference between buffers while presence is detected strongly indicates presence (and motion within boundary).
        if(bufferPercentDiff > HPE_STRONG_PERCENT/100.0) {
            flag |= STRONG_PRESENCE_BREACH;
            //Serial.printf("😁diff: %f->Strong Presence Detected->Flag = 0x%x\n", bufferPercentDiff, flag);
        }

        // Less than a 5% difference between buffers while presence is detected weakly indicates presence (and motion within boundary).
        else if(bufferPercentDiff < HPE_WEAK_PERCENT/100.0) {
            flag |= WEAK_PRESENCE_BREACH;
            //Serial.printf("😁diff: %f->Weak Presence Detected->Flag = 0x%x\n", bufferPercentDiff, flag);
        }

        // In between a 5-10% difference between buffers while presence is detected moderately indicates presence (and motion within boundary).
        else {
            flag |= MODERATE_PRESENCE_BREACH;
            //Serial.printf("😁diff: %f->Moderate Presence Detected->Flag = 0x%x\n", bufferPercentDiff, flag);
        }
        
    }

    // Return.
    return flag;
}

/**
 * Take the avarage of this sensors past distances buffer.
 * @return The average value of this sensors past distances.
 */
float HCSR04::averageBuffer() {
    float sum = 0;
    for(int i = 0; i < bufferSize; i++) sum += pastDistances[i];
    return sum/bufferSize;
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

float HCSR04::getLastBufferAverage() { return lastBufferAverage; }