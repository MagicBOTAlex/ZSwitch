#include "StepperController.h"

// Static variable definitions
int StepperController::NumOfSteppers = 0;
Stepper *StepperController::Steppers = nullptr;
CalibrationData *StepperController::CaliData;
MotorTimers *StepperController::StepperTimers;

void StepperController::Init(CalibrationData *caliData, Stepper *steppers, int numOfSteppers)
{
  StepperController::Steppers = steppers;
  StepperController::NumOfSteppers = numOfSteppers;
  StepperController::CaliData = caliData;

  for (size_t i = 0; i < NumOfSteppers; i++)
  {
    pinMode(steppers[i].enPin, OUTPUT);
    pinMode(steppers[i].dirPin, OUTPUT);
    pinMode(steppers[i].stepPin, OUTPUT);
  }
}

void StepperController::MotorTask(void *params)
{
  while (true)
  {
    // Serial.println("Motor loop");
    long sinceLastRunTime = millis() - StepperController::lastUpdateTime;

    for (size_t i = 0; i < (size_t)StepperController::NumOfSteppers; i++)
    {
      Stepper *stepper = &StepperController:: Steppers[i];
      bool isNegative = signbit(stepper->target);

      if (stepper->steppingEnabled && stepper->target != 0)
      {
        if (stepper->target > 0) {
          stepper->target -= std::min((int)sinceLastRunTime, abs(stepper->target));
        } else {
          stepper->target += std::min((int)sinceLastRunTime, abs(stepper->target));
        }
        Serial.println(stepper->target);
        digitalWrite(stepper->dirPin, isNegative);

        if (stepper->disableOnIdle && stepper->target == 0){
          stepper->motorEnabled = false;
        }
      }

      // Update motor enable states
      digitalWrite(stepper->enPin, !stepper->motorEnabled);
    }

    StepperController::lastUpdateTime = millis();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  
}

void StepperController::SetMovementEnabled(const char axis, bool enabled)
{
  int index = getStepperIndex(axis);
  StepperController::Steppers[index].steppingEnabled = enabled;
}

void StepperController::SetMoveQueue(const char axis, int targetStepPos)
{
  int index = getStepperIndex(axis);
  StepperController::Steppers[index].target = targetStepPos;
}

void StepperController::QueueMove(const char axis, int targetStepPos)
{
  int index = getStepperIndex(axis);
  StepperController::Steppers[index].target += targetStepPos;
}

void StepperController::SetMotorEnabled(const char axis, bool enabled)
{
  int index = getStepperIndex(axis);
  StepperController::Steppers[index].motorEnabled = enabled;
}
