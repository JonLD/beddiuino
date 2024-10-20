/* 09:32 15/03/2023 - change triggering comment */
#ifndef THERMOCOUPLE_H
#define THERMOCOUPLE_H

#include "pindef.h"

#if defined SINGLE_BOARD
#include <Adafruit_MAX31855.h>
extern Adafruit_MAX31855 thermocouple;
#else
#include <max6675.h>
extern MAX6675 thermocouple;
#endif

void thermocoupleInit(void);
float thermocoupleRead(void);

#endif
