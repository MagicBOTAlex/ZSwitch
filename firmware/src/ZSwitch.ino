#include <Arduino.h>
#include "Shared/MotorMoveParams.h"
#include "shared/Stepper.h"
#include "shared/OtherTypeDefs.h"
#include "Calibration.h"
#include "driver/timer.h"

CalibrationData caliData;

#define CALI_SWITCHER_SWITCH_PIN 1
#define CALI_Fila_SWITCH_PIN 2
#define CALI_USER_BTN_PIN 21
// [X, Y, Head]
#define NUM_STEPPERS 2
Stepper steppers[NUM_STEPPERS] = { // old setup
    {7, 5, 6, false}, // switcher
    {10, 8, 9, false} // fila
};

OnTimerFunc onSwitcherTimer;
OnTimerFunc onFilamentTimer;

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


void setup()
{
  Serial.begin(115200);

  delay(500);

  Serial.println("Fucking work");
  

  // StepperController::Init(steppers, NUM_STEPPERS);

  // xTaskCreate(StepperController::MotorTask, "Motors", 1024, NULL, 5,  NULL);
  // xTaskCreate(anotherFunc, "Other loop", 1024, NULL, 5,  NULL);

#ifdef ESP32
  // Timer setup
  timer1 = timerBegin(0, 8000, true);            // Timer 0, prescaler 8000 (1 tick = 1ms)
  timerAttachInterrupt(timer1, timer1ISR, true); // Attach ISR
  timerAlarmWrite(timer1, 1000, true);
  timerAlarmEnable(timer1);
  timer2 = timerBegin(1, 4000, true);            // Timer 0, prescaler 8000 (1 tick = 1ms)
  timerAttachInterrupt(timer2, timer2ISR, true); // Attach ISR
  timerAlarmWrite(timer2, 1000, true);
  timerAlarmEnable(timer2);
#endif

  CalibrationParams *calibrationParams = (CalibrationParams *)malloc(sizeof(CalibrationParams));
  calibrationParams->onTimer1 = &onSwitcherTimer;
  calibrationParams->onTimer2 = &onFilamentTimer;
  calibrationParams->switcherStepper = &steppers[0];
  calibrationParams->filamentStepper = &steppers[1];
  calibrationParams->switcherPin = CALI_SWITCHER_SWITCH_PIN;
  calibrationParams->filaPin = CALI_Fila_SWITCH_PIN;
  calibrationParams->userPin = CALI_USER_BTN_PIN;
  calibrationParams->calibratedData = &caliData;
#ifdef ESP32
  calibrationParams->interruptTimer1 = timer1;
  calibrationParams->interruptTimer2 = timer2;
#endif

  Serial.println("Starting cali task");
  xTaskCreate(CalibrationTask, "Calibration", 1024 * 4, (void *)calibrationParams, 5, NULL); // Fuck me, everything is going to be in here
}

void loop()
{
  // The RTOS scheduler manages the tasks.
}
