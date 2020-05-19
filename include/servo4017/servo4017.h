#ifndef SERVO4017
#define SERVO4017

#include <stdint.h>

typedef struct
{
    const float tickLength_ns;
    const uint8_t updateRate_Hz;
    int64_t extraTime_ticks;
    uint16_t pulseLengths_ticks[9];
} servo4017_t;

void servo4017_setServoValue(servo4017_t *dev, uint8_t servoIdx, uint16_t pulseLength_ms);

void servo4017_initialiseDevice(servo4017_t *dev);

void servo4017_setup(servo4017_t *dev);

#endif