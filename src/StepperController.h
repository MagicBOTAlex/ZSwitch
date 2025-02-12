#ifndef STEPPER_CONTROLLER_H
#define STEPPER_CONTROLLER_H

#include <Arduino.h>
#ifndef ESP32
  #include <Arduino_FreeRTOS.h>
#endif
#include <Shared/ZCommand.h>
#include <Shared/MotorMoveParams.h>
#include <Shared/Stepper.h>
#include <algorithm>

class StepperController
{
private:
    static int NumOfSteppers;
    static Stepper *Steppers;
    static long lastUpdateTime;

    static int getStepperIndex(const char axis);

public:
    static void Init(Stepper *steppers, int numOfSteppers = 3);

    static void MotorTask(void * params);

    static void SetMovementEnabled(const char axis, bool enabled);

    static void SetMoveQueue(const char axis, const int moveTimeMs);
    static void QueueMove(const char axis, const int moveTimeMs);
    static void ClearMoveQueue(const char axis);

    static void SetMotorEnabled(const char axis, bool enabled);
};


#endif