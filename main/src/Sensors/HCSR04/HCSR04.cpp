#include "HCSR04.h"

/**
 * Defines the sensors pins and connects them to the ESP32, and sets the thresholds of a sensor.
 * @param trigger The trigger pin of this sensor.
 * @param echo The echo pin of this sensor.
 * @param id The unique Id for this ultrasonic sensor.
 */
HCSR04::HCSR04(int trigger, int echo, int id) {
    this->trigger = trigger;
    this->echo = echo;
    this->id = id;
}   

 /**
 * Initializes the sensor pin connections wrt the ESP32.
 */
void init() {
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);
}

/**
 * Poll the sensor and store the data.
 */
void HCSR04::readSensor() {
}

/**
 * Mark a sensor as relevant for output collection.
 */
void HCSR04::enable() {

}

/**
 * Mark a sensor as irrelevant for output collection.
 */
void HCSR04::disable() {

}

/**
 * Signal that a sensor has passed its threshold(s) so that action can be taken.
 * @return True if the sensor has passed its threshold, false othewise.
 */
bool HCSR04::passedThreshold() {

}

/**
 * Take the avarage of a buffer that holds past sensor data.
 * @return The average value of a buffer which stores recent past sensor data.
 */
float HCSR04::averageBuffer() {

}