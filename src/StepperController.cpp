#include "StepperController.h"

// Static variable definitions
int StepperController::NumOfSteppers = 0;
Stepper *StepperController::Steppers = nullptr;
long StepperController::lastUpdateTime = 0;

int StepperController::getStepperIndex(const char axis)
{
  for (size_t i = 0; i < StepperController::NumOfSteppers; i++)
  {
    if (StepperController::Steppers[i].axis == axis)
    {
      // Serial.print("Found motor: ");
      // Serial.println(i);
      return i;
    }
  }
  return -1;
}

void StepperController::Init(Stepper *steppers, int numOfSteppers)
{
  StepperController::Steppers = steppers;
  StepperController::NumOfSteppers = numOfSteppers;
  StepperController::lastUpdateTime = 0;

  for (size_t i = 0; i < NumOfSteppers; i++)
  {
    pinMode(steppers[i].enPin, OUTPUT);
    pinMode(steppers[i].dirPin, OUTPUT);
    pinMode(steppers[i].stepPin, OUTPUT);
  }

#ifdef ESP32
  // Timer 1 - Equivalent to TIMER1_COMPA_vect
  timer1 = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1us per tick)
  timerAttachInterrupt(timer1, &timer1ISR, true);
  timerAlarmWrite(timer1, 500, true); // 500us interval
  timerAlarmEnable(timer1);

  // Timer 2 - Equivalent to TIMER3_COMPA_vect
  timer2 = timerBegin(1, 80, true); // Timer 1, prescaler 80 (1us per tick)
  timerAttachInterrupt(timer2, &timer2ISR, true);
  timerAlarmWrite(timer2, 500, true); // 500us interval
  timerAlarmEnable(timer2);

  // Timer 3 - Equivalent to TIMER4_COMPA_vect
  timer3 = timerBegin(2, 80, true); // Timer 2, prescaler 80 (1us per tick)
  timerAttachInterrupt(timer3, &timer3ISR, true);
  timerAlarmWrite(timer3, 500, true); // 500us interval
  timerAlarmEnable(timer3);
#else
  cli();
  // Timer 3 (16-bit)
  TCCR3A = 0;
  TCCR3B = (1 << WGM32) | (1 << CS30); // CTC mode, no prescaler
  OCR3A = 499;                         // Compare match value
  TIMSK3 |= (1 << OCIE3A);             // Enable Timer1 Compare A Match Interrupt

  // Timer 4 (16-bit)
  TCCR4A = 0;
  TCCR4B = (1 << WGM42) | (1 << CS40); // CTC mode, no prescaler
  OCR4A = 499;                         // Compare match value
  TIMSK4 |= (1 << OCIE4A);             // Enable Timer3 Compare A Match Interrupt

  // Timer 5 (16-bit)
  TCCR5A = 0;
  TCCR5B = (1 << WGM52) | (1 << CS50); // CTC mode, no prescaler
  OCR5A = 499;                         // Compare match value
  TIMSK5 |= (1 << OCIE5A);             // Enable Timer1 Compare A Match Interrupt
  sei();
#endif
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
      bool isNegative = signbit(stepper->moveQueue);

      if (stepper->steppingEnabled && stepper->moveQueue != 0)
      {
        if (stepper->moveQueue > 0) {
          stepper->moveQueue -= std::min((int)sinceLastRunTime, abs(stepper->moveQueue));
        } else {
          stepper->moveQueue += std::min((int)sinceLastRunTime, abs(stepper->moveQueue));
        }
        Serial.println(stepper->moveQueue);
        digitalWrite(stepper->dirPin, isNegative);

        if (stepper->disableOnIdle && stepper->moveQueue == 0){
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

void StepperController::SetMoveQueue(const char axis, int moveTimeMs)
{
  int index = getStepperIndex(axis);
  StepperController::Steppers[index].moveQueue = moveTimeMs;
}

void StepperController::QueueMove(const char axis, int moveTimeMs)
{
  int index = getStepperIndex(axis);
  StepperController::Steppers[index].moveQueue += moveTimeMs;
}

void StepperController::ClearMoveQueue(const char axis)
{
  int index = getStepperIndex(axis);
  StepperController::Steppers[index].moveQueue = 0;
}

void StepperController::SetMotorEnabled(const char axis, bool enabled)
{
  int index = getStepperIndex(axis);
  StepperController::Steppers[index].motorEnabled = enabled;
}
