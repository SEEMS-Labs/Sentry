#include "mic.h"

/**
 * Defines the sensors pins and connects them to the ESP32, and sets the thresholds of a sensor.
 * @param type the type of sensor being created.
 * @param pins An array of the pin numbers that the sensor is attached to on the ESP32.
 * @param thresholds[] An array holding all the thresholds relating to the Sensor.
 */
void Microphone::init(SensorType type, int pins[], int thresholds[]) {
}

/**
 * Poll the sensor and store the data.
 */
void Microphone::readSensor() {

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
bool Microphone::passedThreshold() {

}

/**
 * Take the avarage of a buffer that holds past sensor data.
 * @return The average value of a buffer which stores recent past sensor data.
 */
float Microphone::averageBuffer() {

}