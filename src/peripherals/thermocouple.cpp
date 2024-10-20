#include "thermocouple.h"

#if defined SINGLE_BOARD
#include <Adafruit_MAX31855.h>
SPIClass thermoSPI(thermoDI, thermoDO, thermoCLK);
Adafruit_MAX31855 thermocouple(thermoCS, &thermoSPI);
#else
#include <max6675.h>
SPIClass thermoSPI(thermoDI, thermoDO, thermoCLK);
MAX6675 thermocouple(thermoCS, &thermoSPI);
#endif

void thermocoupleInit(void) {
  thermocouple.begin();
}

float thermocoupleRead(void) {
  return thermocouple.readCelsius();
}

