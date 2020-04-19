#ifndef MAINFILE
#define MAINFILE

#include <stdio.h>

static void clock_setup(void);
static void gpio_setup(void);

void handlePacket(uint8_t pckt[]);

#endif