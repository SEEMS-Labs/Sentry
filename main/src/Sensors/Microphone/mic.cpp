#include "Sensors/Microphone/mic.h"

/**
 * Initializes the sensor pin connections wrt the ESP32 and enables sensor.
 */
void Microphone::init() {
}

/**
 * Poll the sensor and store the data.
 */
float Microphone::readSensor() {

}

/**
 * Mark a sensor as relevant for output collection.
 */
void Microphone::enable() {

}

/**
 * Mark a sensor as irrelevant for output collection.
 */
void Microphone::disable() {

}

/**
 * Signal that a sensor has passed its threshold(s) so that action can be taken.
 * @return True if the sensor has passed its threshold, false othewise.
 */
uint8_t Microphone::passedThreshold() {

}

/**
 * Take the avarage of a buffer that holds past sensor data.
 * @return The average value of a buffer which stores recent past sensor data.
 */
float Microphone::averageBuffer() {

}