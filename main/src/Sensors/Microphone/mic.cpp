#include "mic.h"

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

