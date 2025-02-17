#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>

struct  Stepper {
    uint8_t enPin;
    uint8_t dirPin;
    uint8_t stepPin;
    bool motorEnabled;
};
#endif