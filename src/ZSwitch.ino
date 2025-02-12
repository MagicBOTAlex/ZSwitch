#include <Arduino.h>
#ifndef ESP32
  #include <Arduino_FreeRTOS.h>
#endif
#include "StepperController.h"
#include "ZCodeParser.h"
#include "Shared/MotorMoveParams.h"
#include "shared/Stepper.h"
#include "InputController.h"

// [X, Y, Head]
#define NUM_STEPPERS 3
Stepper steppers[NUM_STEPPERS] = {
  {38, 55, 54, 0, 'x', false, false, true},
  {56, 61, 60, 0, 'y', false, false, true},
  // {30, 34, 36, 0, 'e', false, false, true},
};

#pragma region Not gonna mess with this magic

#ifdef ESP32
// Timer Handles
hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;
hw_timer_t *timer3 = NULL;

// Interrupt Service Routines for each timer
void IRAM_ATTR timer1ISR()
{
  if (motorsEnabled[0])
  {
    digitalWrite(STEP_PINS[0], !digitalRead(STEP_PINS[0]));
  }
}

void IRAM_ATTR timer2ISR()
{
  if (motorsEnabled[1])
  {
    digitalWrite(STEP_PINS[1], !digitalRead(STEP_PINS[1]));
  }
}

void IRAM_ATTR timer3ISR()
{
  if (motorsEnabled[2])
  {
    digitalWrite(STEP_PINS[2], !digitalRead(STEP_PINS[2]));
  }
}
#else

// Timers for stepping the motors
ISR(TIMER3_COMPA_vect) {
  if (steppers[0].steppingEnabled && steppers[0].moveQueue != 0) {
    digitalWrite(steppers[0].stepPin, !digitalRead(steppers[0].stepPin));
  }
}
ISR(TIMER4_COMPA_vect) {
  if (steppers[1].steppingEnabled && steppers[1].moveQueue != 0) {
    digitalWrite(steppers[1].stepPin, !digitalRead(steppers[1].stepPin));
  }
}
ISR(TIMER5_COMPA_vect) {
  if (steppers[2].steppingEnabled && steppers[2].moveQueue != 0) {
    digitalWrite(steppers[2].stepPin, !digitalRead(steppers[2].stepPin));
  }
}
#endif

#pragma endregion

void anotherFunc(void * params){

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

void setup() {
  Serial.begin(115200);

  StepperController::Init(steppers, NUM_STEPPERS);
  
  xTaskCreate(StepperController::MotorTask, "Motors", 1024, NULL, 5,  NULL);
  // xTaskCreate(anotherFunc, "Other loop", 1024, NULL, 5,  NULL);
  xTaskCreate(InputControlTask, "Input", 1024, NULL, 5,  NULL);
}

void loop() {
  // The RTOS scheduler manages the tasks.
}
