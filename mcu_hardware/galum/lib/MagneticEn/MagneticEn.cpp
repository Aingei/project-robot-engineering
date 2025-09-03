#include "MagneticEn.h"

MagneticEn::MagneticEn() {}  // Constructor

int MagneticEn::getAngle() {
    // digitalWrite(DIR_PIN, HIGH);
    int in;
    in = map(ams5600.rawAngle(), 0, 4095, 0, 360);  // Mapping raw angle to degrees
    return in;
}