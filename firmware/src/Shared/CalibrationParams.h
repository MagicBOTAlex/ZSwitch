#ifndef CALIBRATION_PARAMS_H
#define CALIBRATION_PARAMS_H

#include <Arduino.h>
#include "OtherTypeDefs.h"
#include "driver/timer.h"
#include "Shared/Stepper.h"
#include "Shared/CalibrationData.h"

typedef struct {
    OnTimerFunc *onTimer1;
    OnTimerFunc *onTimer2;
#ifdef ESP32
    hw_timer_s *interruptTimer1;
    hw_timer_s *interruptTimer2;
#endif
    Stepper *switcherStepper;
    Stepper *filamentStepper;
    uint8_t switcherPin;
    uint8_t filaPin;
    uint8_t userPin;
    CalibrationData *calibratedData;
} CalibrationParams;

#endif // CALIBRATION_PARAMS_H
