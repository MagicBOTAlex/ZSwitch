#include "shared/Stepper.h"
#include "shared/CalibrationData.h"
#include "shared/CalibrationParams.h"
#ifndef ESP32
#include <Arduino_FreeRTOS.h>
#endif
#include "driver/timer.h"
#include "Shared/Utils.h"
#include "Shared/StepperPositioning.h"

#define SWTICH_ON_STATE LOW
#define SWITCHING_FILA_BTN 20

int calibrationStage = 0;

bool isSwitcherCalibrated = false;
bool calibrationMotorMoving = false;
CalibrationParams *caliParams;

int switchCritPointCali = 0; // Because we go forward, then backwards

int currentSwitcherLocation = 0;
int switcher_gotoTarget = 0;
bool lightlyGrab = false;
bool unGrab = false;

#define NUM_FILAMENT_SLOTS 2
int currentlySelectedFila = 0; // 0 = left, 1 = right
StepperPositioning filaSlots[NUM_FILAMENT_SLOTS];

bool prevSwitchState = false;
bool getCaliSwitchState()
{
    return digitalRead(caliParams->switcherPin) == SWTICH_ON_STATE;
}

void setSwitcherTarget(int newTarget)
{
    switcher_gotoTarget = newTarget;
}

bool currentFilaState = false;

enum CaliDirection
{
    Forward,
    Reverse
};

void caliStep(CaliDirection dir)
{
    if (dir == CaliDirection::Forward)
    {
        digitalWrite(caliParams->switcherStepper->dirPin, HIGH);
        currentSwitcherLocation++;
    }
    else
    {
        digitalWrite(caliParams->switcherStepper->dirPin, LOW);
        currentSwitcherLocation--;
    }

    digitalWrite(caliParams->switcherStepper->stepPin, !digitalRead(caliParams->switcherStepper->stepPin));
}

void SwitcherCaliTimer()
{
    portDISABLE_INTERRUPTS();

    // Serial.println("Working");

    if (calibrationMotorMoving && caliParams)
    {
        if (calibrationStage == 0)
        {
            caliParams->calibratedData->stepsPerRotation = 3200;

            // Home ig
            if (!getCaliSwitchState() && prevSwitchState)
            { // Reverse until falling-edge
                Serial.println("Homed!");
                currentSwitcherLocation = 0;
                calibrationStage = 2;
            }
            else
            { // Rotate until falling-edge
                caliStep(CaliDirection::Reverse);
            }
        }
        else if (calibrationStage == 1)
        { // Calibrate steps per rotation

            // Fuck switch. it's slower for some reason in c++ anyways
            if (caliParams->calibratedData->stepsPerRotation == 0 && !getCaliSwitchState() && prevSwitchState)
            { // falling-edge
                caliParams->calibratedData->stepsPerRotation++;
                prevSwitchState = false;
                caliStep(CaliDirection::Reverse);
                Serial.println("Falling. Start counting...");
            }
            else if (caliParams->calibratedData->stepsPerRotation == 0)
            { // Rotate until falling-edge
                caliStep(CaliDirection::Reverse);
            }

            if (caliParams->calibratedData->stepsPerRotation > 0 && !getCaliSwitchState() && prevSwitchState)
            { // next falling edge
                Serial.println("Falling. Done");
                calibrationStage++;
                Serial.print("Steps per rotation: ");
                Serial.println(caliParams->calibratedData->stepsPerRotation);

                currentSwitcherLocation = 0; // Homed
            }
            else if (caliParams->calibratedData->stepsPerRotation > 0)
            { // Keep rotating
                caliParams->calibratedData->stepsPerRotation++;
                caliStep(CaliDirection::Reverse);
            }
        }
        else if (calibrationStage == 2)
        {

            if (switchCritPointCali == 0 && !getCaliSwitchState())
            { // Reverse until unpressed
                switchCritPointCali++;
                Serial.println("Clear!");
            }
            else if (switchCritPointCali == 0)
            { // Rotate until falling-edge
                caliStep(CaliDirection::Reverse);
            }

            else if (switchCritPointCali == 1 && getCaliSwitchState() && !prevSwitchState)
            { // rising-edge
                caliParams->calibratedData->switcherLower = currentSwitcherLocation;
                switchCritPointCali++;
                Serial.println("rising");
            }
            else if (switchCritPointCali == 1)
            { // Rotate until rising-edge
                caliStep(CaliDirection::Forward);
            }

            else if (switchCritPointCali == 2 && !getCaliSwitchState() && prevSwitchState)
            { // until unpressed
                Serial.println("Clear!");
                switchCritPointCali++;
            }
            else if (switchCritPointCali == 2)
            { // Keep rotating
                caliStep(CaliDirection::Forward);
            }

            else if (switchCritPointCali == 3 && getCaliSwitchState() && !prevSwitchState)
            { // rising-edge
                Serial.println("rising");
                caliParams->calibratedData->switcherUpper = currentSwitcherLocation;
                calibrationStage++;
                isSwitcherCalibrated = true;

                Serial.print("Lower: ");
                Serial.println((*caliParams->calibratedData).switcherLower);
                Serial.print("Upper: ");
                Serial.println((*caliParams->calibratedData).switcherUpper);
                Serial.print("Crit: ");
                Serial.println(getSwitchCrit(&caliParams->calibratedData->switcherUpper, &caliParams->calibratedData->switcherLower));
            }
            else if (switchCritPointCali == 3)
            { // Keep rotating
                caliStep(CaliDirection::Reverse);
            }
        }
        prevSwitchState = getCaliSwitchState();
    }
    portENABLE_INTERRUPTS();
}

// Only for fila cali, just now implimented for switcher cali
// Checks if the current state is rising or falling
// Only works if you keep on calling this
bool prevFilaSwitchState = false;
bool edging(bool *currentState, bool checkFalling = true)
{
    bool isRisingEdge = *currentState == !prevFilaSwitchState;
    bool isFallingEdge = !*currentState == prevFilaSwitchState;
    prevFilaSwitchState = *currentState;
    return (checkFalling) ? isFallingEdge : isRisingEdge;
}

/// @brief Sets the switcher's grab side
/// @return True if grabbed. False if still rotating
bool setSwitcherSide(bool rightSide = false)
{
    int switcherCrit = getClampedSwitchCrit(caliParams->calibratedData, &caliParams->calibratedData->switcherUpper, &caliParams->calibratedData->switcherLower, rightSide);

    if (unGrab){
        switcherCrit = (int)(switcherCrit + (caliParams->calibratedData->stepsPerRotation * (1.0f/2.0f))) % caliParams->calibratedData->stepsPerRotation; // Made error here, but it works better so sure
    } else if (lightlyGrab){
        switcherCrit = (int)(switcherCrit + (caliParams->calibratedData->stepsPerRotation * (1.0f/12.0f))) % caliParams->calibratedData->stepsPerRotation;
    }

    setSwitcherTarget(switcherCrit); // move to correct grab side
    if (switcher_gotoTarget != currentSwitcherLocation)
    {
        // Serial.print("Target: ");
        // Serial.println(switcher_gotoTarget);
        // Serial.print("Current: ");
        // Serial.println(currentSwitcherLocation);
        return false; // Wait until filament is grabbed
    }
    else
    {
        return true;
    }
}

// A basic version of controlling the extrudesion and retraction of both sides
void basicFilaStepCommander(CaliDirection direction, bool isRightSide)
{
    if (!setSwitcherSide(isRightSide))
    {
        return;
    }

    filaSlots[isRightSide].current += (direction == CaliDirection::Forward) ? 1 : -1;
    digitalWrite(caliParams->filamentStepper->dirPin, (direction == CaliDirection::Forward ^ isRightSide) ? LOW : HIGH);
    digitalWrite(caliParams->filamentStepper->stepPin, !digitalRead(caliParams->filamentStepper->stepPin)); // step
    // Serial.println("step");
}

void FilamentPullBackUntilFalling()
{
    if (!setSwitcherSide(currentlySelectedFila))
        return; // Wait for correct side

    currentFilaState = digitalRead(caliParams->filaPin) == SWTICH_ON_STATE;
    if (currentFilaState)
    {
        basicFilaStepCommander(CaliDirection::Reverse, currentlySelectedFila);
    }
}

void FilamentPushUntilRising()
{
    if (!setSwitcherSide(currentlySelectedFila))
        return; // Wait for correct side

    currentFilaState = digitalRead(caliParams->filaPin) == SWTICH_ON_STATE;
    if (!currentFilaState)
    {
        basicFilaStepCommander(CaliDirection::Forward, currentlySelectedFila);
    }
}

void switcherMotionTimer()
{
    if (calibrationMotorMoving && currentSwitcherLocation % caliParams->calibratedData->stepsPerRotation != switcher_gotoTarget)
    {
        int offset = (currentSwitcherLocation - switcher_gotoTarget) % caliParams->calibratedData->stepsPerRotation;
        if (offset >= 0)
        {
            caliStep(CaliDirection::Reverse);
        }
        else
        {
            caliStep(CaliDirection::Forward);
        }
        // Serial.println(offset);
    }
}

void filaMotionTimer()
{
    if (!setSwitcherSide(currentlySelectedFila))
        return;

    int offset = (filaSlots[currentlySelectedFila].current - filaSlots[currentlySelectedFila].target);
    if (!offset)
        return; // Offset 0 then do nothing
    if (offset >= 0)
    {
        basicFilaStepCommander(CaliDirection::Reverse, currentlySelectedFila);
    }
    else
    {
        basicFilaStepCommander(CaliDirection::Forward, currentlySelectedFila);
    }
    //Serial.println(offset);
}

void CalibrationTask(void *pv)
{
    caliParams = (CalibrationParams *)pv;
    Serial.println("Starting calibration");
    Serial.print("dirPin: ");
    Serial.println(caliParams->switcherStepper->dirPin);
    Serial.print("stepPin: ");
    Serial.println(caliParams->switcherStepper->stepPin);
    Serial.print("enPin: ");
    Serial.println(caliParams->switcherStepper->enPin);

    pinMode(caliParams->switcherStepper->dirPin, OUTPUT);
    pinMode(caliParams->switcherStepper->stepPin, OUTPUT);
    pinMode(caliParams->switcherStepper->enPin, OUTPUT);
    pinMode(caliParams->switcherPin, (SWTICH_ON_STATE == LOW) ? INPUT_PULLUP : INPUT_PULLDOWN);

    digitalWrite(caliParams->switcherStepper->enPin, LOW);

    // vTaskDelay(pdMS_TO_TICKS(1000));
    (*(caliParams->onTimer1)) = SwitcherCaliTimer;

#ifdef ESP32
    // That's crazy how much more smoothly it is to design for ESP32
    // Took like 4 hours to find a working example though. Multiple failures like the esp32-c3 not writing to serial and shit
    timerAlarmWrite(caliParams->interruptTimer1, 20, true); // Calibration slow, because too lazy to impliment de-bouncing
#else
    cli();
    // Timer 3 (16-bit)
    TCCR3A = 0;
    TCCR3B = (1 << WGM32) | (1 << CS30); // CTC mode, no prescaler
    OCR3A = 10299;                       // Compare match value
    TIMSK3 |= (1 << OCIE3A);             // Enable Timer1 Compare A Match Interrupt
    sei();
#endif

    // Start switcher cali
    Serial.println("Started motor");
    calibrationMotorMoving = true;

    while (!isSwitcherCalibrated)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    (*(caliParams->onTimer1)) = switcherMotionTimer; // Set to motion func
#ifdef ESP32
    timerAlarmWrite(caliParams->interruptTimer1, 2, true);
#else
    OCR3A = 3299;
#endif

// Start Filament cali
#ifdef ESP32
    // Not adding more support for arduino mega. At least for now
    timerAlarmWrite(caliParams->interruptTimer2, 2, true);
#endif

    // Grab right side and wait for user click
    setSwitcherSide(true);
    Serial.println("Please insert filament and then press user button");
    while (true)
    {
        if (digitalRead(caliParams->userPin) == SWTICH_ON_STATE)
        {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Setup stepper driver
    pinMode(caliParams->filamentStepper->dirPin, OUTPUT);
    pinMode(caliParams->filamentStepper->stepPin, OUTPUT);
    pinMode(caliParams->filamentStepper->enPin, OUTPUT);
    pinMode(caliParams->filaPin, (SWTICH_ON_STATE == LOW) ? INPUT_PULLUP : INPUT_PULLDOWN);

    digitalWrite(caliParams->filamentStepper->enPin, LOW);

    Serial.println("Started fila switch cali");
    (*(caliParams->onTimer2)) = FilamentPullBackUntilFalling; // Start pulling back
    currentFilaState = digitalRead(caliParams->filaPin) == SWTICH_ON_STATE;
    while (currentFilaState) // Wait until filament pulled back
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    caliParams->calibratedData->filamentLower = filaSlots[0].current;
    Serial.print("Lower fila: ");
    Serial.println(caliParams->calibratedData->filamentLower);

    (*(caliParams->onTimer2)) = FilamentPushUntilRising; // Start pushing forward
    while (!currentFilaState)                            // Wait until filament pushed to switch
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    caliParams->calibratedData->filamentUpper = filaSlots[0].current;
    Serial.print("Upper fila: ");
    Serial.println(caliParams->calibratedData->filamentUpper);
    (*(caliParams->onTimer2)) = nullptr;

    for (size_t i = 0; i < NUM_FILAMENT_SLOTS; i++)
    {
        currentlySelectedFila = i;
        Serial.println((currentlySelectedFila) ? "Waiting for filament right load" : "Waiting for filament left load");
        (*(caliParams->onTimer2)) = FilamentPushUntilRising; // Start pushing forward
        while (!currentFilaState)                            // Wait until filament pushed to switch
        {
            // Serial.println("push");
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        (*(caliParams->onTimer2)) = FilamentPullBackUntilFalling; // Start pulling back
        while (currentFilaState)                                  // Wait until filament pulled back
        {
            // Serial.println("pull");
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        filaSlots[currentlySelectedFila].current = caliParams->calibratedData->filamentLower;
        (*(caliParams->onTimer2)) = filaMotionTimer;                                                                                               // Ready moving
        filaSlots[currentlySelectedFila].target = filaSlots[currentlySelectedFila].current - (caliParams->calibratedData->stepsPerRotation * 0.5); // Move half  rotation back to clear for fila 2
        while (filaSlots[currentlySelectedFila].current != filaSlots[currentlySelectedFila].target)                                                // wait for filament finish moved
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        Serial.println((currentFilaState) ? "Right fila ready" : "Left fila ready");
    }

    bool isFirstLoad = true;
    pinMode(SWITCHING_FILA_BTN, (SWTICH_ON_STATE == LOW) ? INPUT_PULLUP : INPUT_PULLDOWN);
    bool prevSwitchingState; // Used to detect falling-edge
    while (true)             // Enter printing loop
    {
        bool switchingBtnState = digitalRead(SWITCHING_FILA_BTN) == SWTICH_ON_STATE || isFirstLoad;
        // Serial.println(switchingBtnState);

        if ((switchingBtnState && !prevFilaSwitchState) || isFirstLoad)
        {
            if(!isFirstLoad)  {
                Serial.println("Switching filament");
                Serial.println("pull");
                vTaskDelay(pdMS_TO_TICKS(4000));
                timerAlarmWrite(caliParams->interruptTimer2, 2, true);
                // Pull
                unGrab = false;
                lightlyGrab = true;
                (*(caliParams->onTimer2)) = FilamentPullBackUntilFalling; // Start pulling back
                vTaskDelay(pdMS_TO_TICKS(2000));
                lightlyGrab = false;
                while (currentFilaState)                                  // Wait until filament pulled back
                {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                filaSlots[currentlySelectedFila].current = caliParams->calibratedData->filamentLower;
                filaSlots[currentlySelectedFila].target = filaSlots[currentlySelectedFila].current - (caliParams->calibratedData->stepsPerRotation * 0.5); // Move half  rotation back to clear for fila 2
                (*(caliParams->onTimer2)) = filaMotionTimer;                                                                                               // Ready moving
                while (filaSlots[currentlySelectedFila].current != filaSlots[currentlySelectedFila].target)                                                // wait for filament finish moved
                {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }
            
            Serial.println("push");
            // Push
            currentlySelectedFila = !currentlySelectedFila;
            (*(caliParams->onTimer2)) = FilamentPushUntilRising; // Start pushing forward
            while (!currentFilaState)                            // Wait until filament pushed to switch
            {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            filaSlots[currentlySelectedFila].current = caliParams->calibratedData->filamentUpper;
            filaSlots[currentlySelectedFila].target = 0;                                                // 0 is the started pull place
            lightlyGrab = false;
            (*(caliParams->onTimer2)) = filaMotionTimer;                                                // Ready moving
            while (filaSlots[currentlySelectedFila].current != filaSlots[currentlySelectedFila].target) // wait for filament finish moved
            {
                vTaskDelay(pdMS_TO_TICKS(100));
            }

            Serial.println("push 2 soft");
            timerAlarmWrite(caliParams->interruptTimer2, 20, true);
            // Push2
            filaSlots[currentlySelectedFila].target = caliParams->calibratedData->stepsPerRotation * 2;                                 
            lightlyGrab = true;
            (*(caliParams->onTimer2)) = filaMotionTimer;                                                // Ready moving
            while (filaSlots[currentlySelectedFila].current != filaSlots[currentlySelectedFila].target) // wait for filament finish moved
            {
                vTaskDelay(pdMS_TO_TICKS(100));
            }

            // Ungrab
            unGrab = true;
            // Push3
            filaSlots[currentlySelectedFila].target++; // Just gonna reuse the push methods                        
            lightlyGrab = true;
            (*(caliParams->onTimer2)) = filaMotionTimer;                                                // Ready moving
            while (filaSlots[currentlySelectedFila].current != filaSlots[currentlySelectedFila].target) // wait for filament finish moved
            {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            isFirstLoad = false;
        }

        prevSwitchingState = switchingBtnState;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // int cripPoint = getSwitchCrit(caliParams->calibratedData->switcherUpper, caliParams->calibratedData->switcherLower);
    // while (true)
    // {
    //     switcher_gotoTarget = cripPoint;
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     switcher_gotoTarget = (int)(cripPoint + (caliParams->calibratedData->stepsPerRotation * 0.5)) % caliParams->calibratedData->stepsPerRotation;
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    vTaskDelete(NULL);
}
