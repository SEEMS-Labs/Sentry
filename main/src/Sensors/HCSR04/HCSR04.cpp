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
 */
float HCSR04::readSensor(TickType_t xMaxBlockTime) {
    
    // Ensure sensor is active before reading.
    if(!active) return -1.0;

    // Pulse trigger for 10 us.
    pulseTrigger();
    
    // Wait for pulse to complete.
    uint32_t pulseFinishedEvent = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);

    // Compute, store at return the distance if pulse didn't time out.
    if(pulseFinishedEvent != 0) {
        float inches = computeInches();
        if(distIndex == bufferSize) distIndex = 0;
        pastDistances[distIndex++] = inches;
        return inches;
    }
    else return -1.0;

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
 * Signal that this ultrasonic sensor has passed one or both of its 2 thresholds.
 * @return A byte where the Bit 7 (MSB) represents ths obstacle detection threshold 
 * and Bit 6 representes the presence detection threshold. A bit set to 1 indicates 
 * the distance threshold has been breached and a bit set to 0 indicates the opposite.
 */
char HCSR04::passedThreshold() {
    // Grab the last measured distance.
    float distance = pastDistances[distIndex];
    char flag = 0x00;
    if(distance <= obstacleDetectionThreshold) flag |= OBSTACLE_THRESHOLD_BREACHED;
    if(distance <= presenceDetectionThreshold) flag |= PRESENCE_THRESHOLD_BREACHED;
    return flag;
}

/**
 * Take the avarage of this sensors past distances buffer.
 * @return The average value of this sensors past distnaces.
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