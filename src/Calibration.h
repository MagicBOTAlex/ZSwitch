#include "shared/Stepper.h"
#include "shared/CalibrationData.h"
#include "shared/CalibrationParams.h"
#ifndef ESP32
#include <Arduino_FreeRTOS.h>
#endif
#include "driver/timer.h"

#define SWTICH_ON_STATE LOW

int calibrationStage = 0;
CalibrationData caliData;

bool isSwitcherCalibrated = false;
bool isFilaCalibrated = false;
bool calibrationMotorMoving = false;
CalibrationParams *caliParams;

int switchCritPointCali = 0; // Because we go forward, then backwards

int currentSwitcherLocation = 0;
int switcher_gotoTarget = 0;

int fila1Location = 0;
int fila2Location = 0;

bool prevSwitchState = false;
bool getCaliSwitchState()
{
    return digitalRead(caliParams->switcherPin) == SWTICH_ON_STATE;
}

void setSwitcherTarget(int newTarget){
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

int getSwitchCrit(int *upper, int *lower, bool getOppisiteSide = false){
    int cripPoint = *upper - ((*upper - *lower) / 2);
    int offCrit = (int)(cripPoint + (caliData.stepsPerRotation * 0.5)) % caliData.stepsPerRotation;
    int clamped = ((getOppisiteSide) ? offCrit : cripPoint) % caliData.stepsPerRotation;
    // Serial.print("Opp: ");
    // Serial.println(getOppisiteSide);
    // Serial.print("Upper: ");
    // Serial.println(*upper);
    // Serial.print("lower: ");
    // Serial.println(*lower);
    // Serial.print("Clamped crit: ");
    // Serial.println(clamped);
    return clamped;
}

void SwitcherCaliTimer()
{
    portDISABLE_INTERRUPTS();

    // Serial.println("Working");

    if (calibrationMotorMoving && caliParams)
    {
        if (calibrationStage == 0)
        {
            caliData.stepsPerRotation = 3200;

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
            if (caliData.stepsPerRotation == 0 && !getCaliSwitchState() && prevSwitchState)
            { // falling-edge
                caliData.stepsPerRotation++;
                prevSwitchState = false;
                caliStep(CaliDirection::Reverse);
                Serial.println("Falling. Start counting...");
            }
            else if (caliData.stepsPerRotation == 0)
            { // Rotate until falling-edge
                caliStep(CaliDirection::Reverse);
            }

            if (caliData.stepsPerRotation > 0 && !getCaliSwitchState() && prevSwitchState)
            { // next falling edge
                Serial.println("Falling. Done");
                calibrationStage++;
                Serial.print("Steps per rotation: ");
                Serial.println(caliData.stepsPerRotation);

                currentSwitcherLocation = 0; // Homed
            }
            else if (caliData.stepsPerRotation > 0)
            { // Keep rotating
                caliData.stepsPerRotation++;
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
                caliData.switcherLower = currentSwitcherLocation;
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
                caliData.switcherUpper = currentSwitcherLocation;
                calibrationStage++;
                isSwitcherCalibrated = true;

                Serial.print("Lower: ");
                Serial.println(caliData.switcherLower);
                Serial.print("Upper: ");
                Serial.println(caliData.switcherUpper);
                Serial.print("Crit: ");
                Serial.println(getSwitchCrit(&caliData.switcherUpper, &caliData.switcherLower));
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
bool edging(bool *currentState, bool checkFalling = true){
    bool isRisingEdge = *currentState == !prevFilaSwitchState;
    bool isFallingEdge = !*currentState == prevFilaSwitchState;
    prevFilaSwitchState = *currentState;
    return (checkFalling) ? isFallingEdge : isRisingEdge;
}


/// @brief Sets the switcher's grab side
/// @return True if grabbed. False if still rotating
bool setSwitcherSide(bool rightSide = false) {
    int switcherCrit = getSwitchCrit(&caliData.switcherUpper, &caliData.switcherLower, rightSide);
    setSwitcherTarget(switcherCrit); // move to correct grab side
    if (switcher_gotoTarget != currentSwitcherLocation){
        // Serial.print("Target: ");
        // Serial.println(switcher_gotoTarget);
        // Serial.print("Current: ");
        // Serial.println(currentSwitcherLocation);
        return false; // Wait until filament is grabbed
    } else {
        return true;
    }
}

// A basic version of controlling the extrudesion and retraction of both sides
bool isLeftGrabbed = false;
void basicFilaStepCommander(CaliDirection direction, bool isRightSide) {
    if (!setSwitcherSide(isRightSide)){
        return;
    }

    if (!isRightSide){
        fila1Location += (direction == CaliDirection::Forward) ? 1 : -1;
    } else {
        fila2Location += (direction == CaliDirection::Forward) ? 1 : -1;
    }
    digitalWrite(caliParams->filamentStepper->dirPin, (direction == CaliDirection::Forward) ? LOW : HIGH);
    digitalWrite(caliParams->filamentStepper->stepPin, !digitalRead(caliParams->filamentStepper->stepPin)); // step
    Serial.println("step");
}

int filaPushPullSteps = 0;
void FilamentPullBackUntilFalling()
{
    if (!setSwitcherSide(false)) return; // Wait for correct side
    
    currentFilaState = digitalRead(caliParams->filaPin) == SWTICH_ON_STATE;
    if (!currentFilaState)
    {
        if (filaPushPullSteps == 0) {
            Serial.println("You fucked up. Start with filament inserted to max depth.");
        }
    } else {
        filaPushPullSteps++;
        basicFilaStepCommander(CaliDirection::Reverse, false);
    }
    
}

void FilamentPushUntilRising()
{
    if (!setSwitcherSide(false)) return; // Wait for correct side

    currentFilaState = digitalRead(caliParams->filaPin) == SWTICH_ON_STATE;    
    if (currentFilaState)
    {
        if (filaPushPullSteps == 0) {
            Serial.println("Somethings is wrong...");
        }
    } else {
        filaPushPullSteps++;
        basicFilaStepCommander(CaliDirection::Forward, false);
    }
    
}


void motionTimer() 
{
    if (calibrationMotorMoving && currentSwitcherLocation % caliData.stepsPerRotation != switcher_gotoTarget)
    {
        int offset = (currentSwitcherLocation - switcher_gotoTarget) % caliData.stepsPerRotation;
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

    (*(caliParams->onTimer1)) = motionTimer; // Set to motion func
#ifdef ESP32
    timerAlarmWrite(caliParams->interruptTimer1, 2, true);
#else
    OCR3A = 3299;
#endif

// Start Filament cali
#ifdef ESP32
    // Not adding more support for arduino mega. At least for now
    timerAlarmWrite(caliParams->interruptTimer2, 5, true);
#endif

    // Grab right side and wait for user click
    setSwitcherSide(true); 
    Serial.println("Please insert filament and then press user button");
    while (true)
    {
        if (digitalRead(caliParams->userPin) == SWTICH_ON_STATE){
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
        // Serial.println("Cali lower fila");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    caliData.filamentLower1 = fila1Location;
    Serial.print("Lower fila: ");
    Serial.println(caliData.filamentLower1);
    
    
    (*(caliParams->onTimer2)) = FilamentPushUntilRising; // Start pushing forward
    while (!currentFilaState) // Wait until filament pushed to switch
    {
        // Serial.println("Cali upper fila");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    caliData.filamentUpper1 = fila1Location;
    Serial.print("Upper fila: ");
    Serial.println(caliData.filamentUpper1);
    (*(caliParams->onTimer2)) = nullptr;

    // int cripPoint = getSwitchCrit(&caliData.switcherUpper, &caliData.switcherLower);
    // while (true)
    // {
    //     switcher_gotoTarget = cripPoint;
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     switcher_gotoTarget = (int)(cripPoint + (caliData.stepsPerRotation * 0.5)) % caliData.stepsPerRotation;
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    Serial.println("Cali done");
    vTaskDelete(NULL);
}
