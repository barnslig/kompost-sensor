#ifndef HELTEC_BATTERY_SENSOR_H_
#define HELTEC_BATTERY_SENSOR_H_

#include <Arduino.h>
#include <cstdint>

class HeltecBatterySensor
{
protected:
    uint8_t pinBattery;
    uint8_t pinDrain;

public:
    HeltecBatterySensor(uint8_t pinBattery, uint8_t pinDrain);

    void begin();

    float read();
};

#endif // HELTEC_BATTERY_SENSOR_H_
