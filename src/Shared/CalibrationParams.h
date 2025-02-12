#ifndef CALIBRATION_PARAMS_H
#define CALIBRATION_PARAMS_H

#include <Arduino.h>
#include "OtherTypeDefs.h"

typedef struct {
    OnTimer *onTimer;
    uint8_t enPin;
    uint8_t dirPin;
    uint8_t stepPin;
    uint8_t switchPin;
} CalibrationParams;

#endif // CALIBRATION_PARAMS_H
