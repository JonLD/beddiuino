#ifndef TOF_H
#define TOF_H

#include <Adafruit_VL53L0X.h>
#include <movingAvg.h>
#include <stdint.h> // for uint16_t, uint32_t

class TOF
{
  public:
    TOF() : tofReading{0u}, mvAvg(4) {}
    void init();
    uint16_t readLvl();
    uint16_t readRangeToPct(uint16_t val);

  private:
    // HardwareTimer* hw_timer;
    // static void TimerHandler10(void);
    uint32_t tofReading;
    Adafruit_VL53L0X tof_sensor;
    movingAvg mvAvg;
};

#endif // TOF_H
