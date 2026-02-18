#ifndef MAGNETICEN_H
#define MAGNETICEN_H

#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

class MagneticEn {
public:
    MagneticEn();           // Constructor
    int getAngle();         // Function to get the angle

private:
    AS5600 ams5600;         // Instance of AS5600 sensor
    int angle_in = 0;       // Store the last angle value
};

#endif