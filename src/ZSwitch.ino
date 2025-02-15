#include <Arduino.h>
#ifndef ESP32
#include <Arduino_FreeRTOS.h>
#endif
#include "StepperController.h"
#include "ZCodeParser.h"
#include "Shared/MotorMoveParams.h"
#include "shared/Stepper.h"
#include "shared/OtherTypeDefs.h"
#include "Calibration.h"
#include "driver/timer.h"

#ifdef ESP32
#define CALI_SWITCHER_SWITCH_PIN 21
#define CALI_Fila_SWITCH_PIN 20
// [X, Y, Head]
#define NUM_STEPPERS 2
Stepper steppers[NUM_STEPPERS] = {
    {7, 5, 6, 0, 'x', false, false, true},
    {10, 8, 9, 0, 'y', false, false, true}
    // {30, 34, 36, 0, 'e', false, false, true},
};

#else

#define CALI_SWITCHER_SWITCH_PIN 3
// [X, Y, Head]
#define NUM_STEPPERS 2
Stepper steppers[NUM_STEPPERS] = {
    {38, 55, 54, 0, 'x', false, false, true},
    {56, 61, 60, 0, 'y', false, false, true}
    // {30, 34, 36, 0, 'e', false, false, true},
};
#endif

#pragma region Not gonna mess with this magic

OnTimerFunc onSwitcherTimer;
OnTimerFunc onFilamentTimer;

#ifdef ESP32
// Timer Handles
hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;

void IRAM_ATTR timer1ISR()
{
  if (onSwitcherTimer)
  {
    onSwitcherTimer();
  }
}

void IRAM_ATTR timer2ISR()
{
  if (onFilamentTimer)
  {
    onFilamentTimer();
  }
}
#else

// Timers for stepping the motors
ISR(TIMER3_COMPA_vect)
{
  // if (steppers[0].steppingEnabled && steppers[0].moveQueue != 0) {
  //   digitalWrite(steppers[0].stepPin, !digitalRead(steppers[0].stepPin));
  // }

  if (onSwitcherTimer)
  {
    onSwitcherTimer();
  }
}
ISR(TIMER4_COMPA_vect)
{
  if (steppers[1].steppingEnabled && steppers[1].moveQueue != 0)
  {
    digitalWrite(steppers[1].stepPin, !digitalRead(steppers[1].stepPin));
  }
}
#endif

#pragma endregion

void anotherFunc(void *params)
{

  while (true)
  {
    StepperController::SetMotorEnabled('x', true);
    StepperController::SetMovementEnabled('x', true);
    StepperController::QueueMove('x', -2000);
    Serial.println("Added to queue");
    vTaskDelay(pdMS_TO_TICKS(5000));
  }

  vTaskDelete(NULL);
}

void setup()
{
  Serial.begin(115200);

  // StepperController::Init(steppers, NUM_STEPPERS);

  // xTaskCreate(StepperController::MotorTask, "Motors", 1024, NULL, 5,  NULL);
  // xTaskCreate(anotherFunc, "Other loop", 1024, NULL, 5,  NULL);

#ifdef ESP32
  // Timer setup
  timer1 = timerBegin(0, 8000, true);            // Timer 0, prescaler 8000 (1 tick = 1ms)
  timerAttachInterrupt(timer1, timer1ISR, true); // Attach ISR
  timerAlarmWrite(timer1, 1000, true);
  timerAlarmEnable(timer1);
  timer2 = timerBegin(0, 8000, true);            // Timer 0, prescaler 8000 (1 tick = 1ms)
  timerAttachInterrupt(timer2, timer2ISR, true); // Attach ISR
  timerAlarmWrite(timer2, 1000, true);
  timerAlarmEnable(timer2);
#endif

  CalibrationParams *calibrationParams = (CalibrationParams *)malloc(sizeof(CalibrationParams));
  calibrationParams->onTimer1 = &onSwitcherTimer;
  calibrationParams->switcherStepper = &steppers[0];
  calibrationParams->filamentStepper = &steppers[1];
  calibrationParams->switcherPin = CALI_SWITCHER_SWITCH_PIN;
  calibrationParams->filaPin = CALI_Fila_SWITCH_PIN;
#ifdef ESP32
  calibrationParams->interruptTimer1 = timer1;
#endif

  xTaskCreate(CalibrationTask, "Calibration", 1024 * 2, (void *)calibrationParams, 5, NULL);
}

void loop()
{
  // The RTOS scheduler manages the tasks.
}
