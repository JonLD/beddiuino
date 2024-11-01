#ifndef TOF_H
#define TOF_H

#include <stdint.h> // for uint16_t, uint32_t
#include "../../lib/Common/sensors_state.h"

class TOF {
  public:
    TOF() {
    tofReading = 0u;
  }
    void init(SensorState& sensor);
    uint16_t readLvl();
    uint16_t readRangeToPct(uint16_t val);

  private:
    // HardwareTimer* hw_timer;
    // static void TimerHandler10(void);
    uint32_t tofReading;
    #ifdef TOF_VL53L0X
    Adafruit_VL53L0X tof_sensor;
    movingAvg mvAvg(4);
    #endif // TOF_VL53L0X
};

#endif // TOF_H
