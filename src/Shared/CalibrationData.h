#ifndef CALIBRATION_DATA_H
#define CALIBRATION_DATA_H

#include <Arduino.h>

typedef struct {
    int stepsPerRotation;
    int switcherUpper;
    int switcherLower;
    int filamentUpper1;
    int filamentLower1;
} CalibrationData;
#endif