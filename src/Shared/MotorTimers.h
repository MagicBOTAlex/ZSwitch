#if !defined(MOTOR_TIMERS_H)
#define MOTOR_TIMERS_H

#include "driver/timer.h"
#include "OtherTypeDefs.h"

typedef struct {
    int numTimers;
    hw_timer_t *timers;
    OnTimerFunc *timerFuncs;
} MotorTimers;

#endif // MOTOR_TIMERS_H
