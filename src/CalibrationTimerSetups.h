// Seperate file for easier seperation. It's shit, but I don't care atm lol
#include "driver/timer.h"

#ifdef ESP32
void setupCaliTimer(hw_timer_t *timer){
    // That's crazy how much more smoothly it is to design for ESP32
    // Took like 4 hours to find a working example though. Multiple failures like the esp32-c3 not writing to serial and shit
    timerAlarmWrite(timer, 25, true);
#elif defined(ARDUINO)
void setupCaliTimer(){
    // Timer 3 (16-bit)
    TCCR3A = 0;
    TCCR3B = (1 << WGM32) | (1 << CS30); // CTC mode, no prescaler
    OCR3A = 10299;                         // Compare match value
    TIMSK3 |= (1 << OCIE3A);             // Enable Timer1 Compare A Match Interrupt
#endif
}