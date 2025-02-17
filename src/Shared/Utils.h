#include "CalibrationData.h"

int getSwitchCrit(int *upper, int *lower){
    int cripPoint = *upper - ((*upper - *lower) / 2);
    return cripPoint;
}

int getClampedSwitchCrit(CalibrationData *caliData, int *upper, int *lower, bool getOppisiteSide = false){
    int critPoint = getSwitchCrit(upper, lower);
    int offCrit = (int)(critPoint + (caliData->stepsPerRotation * 0.5)) % caliData->stepsPerRotation;
    int clamped = ((getOppisiteSide) ? offCrit : critPoint) % caliData->stepsPerRotation;
    return clamped;
}