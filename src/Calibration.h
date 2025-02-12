#include "shared/Stepper.h"
#include "shared/CalibrationData.h"
#include "shared/CalibrationParams.h"



bool isCalibrated = false;

void OnTimer3(){
    Serial.println("Somehow working");
}

void CalibrationTask(void *pv){
    CalibrationParams *params = (CalibrationParams*) pv;
    (*(params->onTimer)) = OnTimer3;

    cli();
    // Timer 3 (16-bit)
    TCCR3A = 0;
    TCCR3B = (1 << WGM32) | (1 << CS30); // CTC mode, no prescaler
    OCR3A = 899;                         // Compare match value
    TIMSK3 |= (1 << OCIE3A);             // Enable Timer1 Compare A Match Interrupt
    sei();
    Serial.println("Calibration setup");
}
