#include "shared/Stepper.h"
#include "shared/CalibrationData.h"
#include "shared/CalibrationParams.h"
#ifndef ESP32
  #include <Arduino_FreeRTOS.h>
#endif
#include "driver/timer.h"
#include "CalibrationTimerSetups.h"

#define SWTICH_ON_STATE LOW

int calibrationStage = 0;
CalibrationData caliData;

bool isCalibrated = false;
bool calibrationMotorMoving = false;
CalibrationParams *caliParams;

int switchCritPointCali = 0; // Because we go forward, then backwards

int currentCaliLocation = 0;
int gotoTarget = 0;

bool prevSwitchState = false;
bool getCaliSwitchState(){
    return digitalRead(caliParams->switchPin) == SWTICH_ON_STATE;
}

enum CaliDirection {
    Forward,
    Reverse
};

void caliStep(CaliDirection dir) {
    if (dir == CaliDirection::Forward){
        digitalWrite(caliParams->dirPin, HIGH);
        currentCaliLocation++;
    } else {
        digitalWrite(caliParams->dirPin, LOW);
        currentCaliLocation--;
    }

    digitalWrite(caliParams->stepPin, !digitalRead(caliParams->stepPin));
}

void OnTimer3(){
    portDISABLE_INTERRUPTS();

    // Serial.println("Working");
    
    if (calibrationMotorMoving && caliParams){
        if (calibrationStage == 0) {
            calibrationStage = 2; // skip rotation calibration
            caliData.stepsPerRotation = 3200;

            // Home ig
            if (!getCaliSwitchState() && prevSwitchState){ // Reverse until falling-edge
                switchCritPointCali++;
                Serial.println("Homed!");
            } else { // Rotate until falling-edge
                caliStep(CaliDirection::Reverse);
            }
            
        } else if (calibrationStage == 1) { // Calibrate steps per rotation

            // Fuck switch. it's slower for some reason in c++ anyways
            if (caliData.stepsPerRotation == 0 && !getCaliSwitchState() && prevSwitchState){ // falling-edge
                caliData.stepsPerRotation++;
                prevSwitchState = false;
                caliStep(CaliDirection::Reverse);
                Serial.println("Falling. Start counting...");
            } else if (caliData.stepsPerRotation == 0) { // Rotate until falling-edge
                caliStep(CaliDirection::Reverse);
            }
            
            if (caliData.stepsPerRotation > 0 && !getCaliSwitchState() && prevSwitchState){ // next falling edge
                Serial.println("Falling. Done");
                calibrationStage++;
                Serial.print("Steps per rotation: ");
                Serial.println(caliData.stepsPerRotation);

                currentCaliLocation = 0; // Homed
                
            } else if (caliData.stepsPerRotation > 0) { // Keep rotating
                caliData.stepsPerRotation++;
                caliStep(CaliDirection::Reverse);
            }
        } else if (calibrationStage == 2){

            if (switchCritPointCali == 0 && !getCaliSwitchState()){ // Reverse until unpressed
                switchCritPointCali++;
                Serial.println("Clear!");
            } else if (switchCritPointCali == 0) { // Rotate until falling-edge
                caliStep(CaliDirection::Reverse);
            } else if (switchCritPointCali == 1 && getCaliSwitchState() && !prevSwitchState){ // rising-edge
                caliData.switcherLower = currentCaliLocation;
                switchCritPointCali++;
                Serial.println("rising");
            } else if (switchCritPointCali == 1) { // Rotate until falling-edge
                caliStep(CaliDirection::Forward);
            } else if (switchCritPointCali == 2 && !getCaliSwitchState() && prevSwitchState){ // until unpressed
                Serial.println("Clear!");
                switchCritPointCali++;
                
            } else if (switchCritPointCali == 2) { // Keep rotating
                caliStep(CaliDirection::Forward);
            } else if (switchCritPointCali == 3 && getCaliSwitchState() && !prevSwitchState){ // rising-edge
                Serial.println("rising");
                caliData.switcherUpper = currentCaliLocation;
                calibrationStage++;
                isCalibrated = true;

                Serial.print("Lower: ");
                Serial.println(caliData.switcherLower);
                Serial.print("Upper: ");
                Serial.println(caliData.switcherUpper);
                Serial.print("Crit: ");
                Serial.println(caliData.switcherUpper - ((caliData.switcherUpper-caliData.switcherLower)/2));
                
            } else if (switchCritPointCali == 3) { // Keep rotating
                caliStep(CaliDirection::Reverse);
            }

        }
        prevSwitchState = getCaliSwitchState();
    }
    portENABLE_INTERRUPTS();
}

void motionTimer(){
    if (calibrationMotorMoving && currentCaliLocation % caliData.stepsPerRotation != gotoTarget){
        int offset = (currentCaliLocation-gotoTarget) % caliData.stepsPerRotation;
        if (offset >= 0){
            caliStep(CaliDirection::Reverse);
        } else {
            caliStep(CaliDirection::Forward);
        }
        // Serial.println(offset);
    }
}

void CalibrationTask(void *pv){
    caliParams = (CalibrationParams*) pv;
    Serial.println("Starting calibration");
    Serial.print("dirPin: "); Serial.println(caliParams->dirPin);
    Serial.print("stepPin: "); Serial.println(caliParams->stepPin);
    Serial.print("enPin: "); Serial.println(caliParams->enPin);

    pinMode(caliParams->dirPin, OUTPUT);
    pinMode(caliParams->stepPin, OUTPUT);
    pinMode(caliParams->enPin, OUTPUT);
    pinMode(caliParams->switchPin, (SWTICH_ON_STATE == LOW)? INPUT_PULLUP:INPUT_PULLDOWN);
    
    digitalWrite(caliParams->enPin, LOW);
    
    // vTaskDelay(pdMS_TO_TICKS(1000));
    (*(caliParams->onTimer)) = OnTimer3;
    
#ifdef ESP32
    setupCaliTimer(caliParams->interruptTimer);
#else
    cli();
    setupCaliTimer();
    sei();
#endif
    
    Serial.println("Started motor");
    calibrationMotorMoving = true;

    while (!isCalibrated)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    (*(caliParams->onTimer)) = motionTimer;
    #ifdef ESP32
    timerAlarmWrite(caliParams->interruptTimer, 2, true);
    #else
    OCR3A = 3299;
    #endif

    int cripPoint = caliData.switcherUpper - ((caliData.switcherUpper-caliData.switcherLower)/2);
    while (true)
    {
        gotoTarget = cripPoint;
        vTaskDelay(pdMS_TO_TICKS(1000));
        gotoTarget = (int)(cripPoint + (caliData.stepsPerRotation * 0.5)) % caliData.stepsPerRotation;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    

    vTaskDelete(NULL);
}
