#include "HeltecBatterySensor.h"

HeltecBatterySensor::HeltecBatterySensor(uint8_t pinBattery, uint8_t pinDrain) : pinBattery(pinBattery), pinDrain(pinDrain)
{
}

void HeltecBatterySensor::begin()
{
    pinMode(pinDrain, OUTPUT);
}

float HeltecBatterySensor::read()
{
    digitalWrite(pinDrain, LOW);

    delay(10);

    uint32_t mV = analogReadMilliVolts(pinBattery);
    float corr_V = mV * (220 + 100) / 100 * 0.001;

    digitalWrite(pinDrain, HIGH);

    return corr_V;
}