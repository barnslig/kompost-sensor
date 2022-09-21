#include "TemperatureSensor.h"

TemperatureSensor::TemperatureSensor(DallasTemperature *dallasTemp, uint8_t sensorsMax, uint8_t sensorsPrecision) : sensors(dallasTemp), sensorsPrecision(sensorsPrecision)
{
    sensorAddresses = new DeviceAddress[sensorsMax];
}

void TemperatureSensor::begin()
{
    sensors->begin();
    sensorsFound = sensors->getDeviceCount();
    for (int i = 0; i < sensorsFound; i += 1)
    {
        if (!sensors->getAddress(sensorAddresses[i], i))
        {
            // serial.print(F("Unable to find address for sensor "));
            // serial.println(i);
            continue;
        }

        sensors->setResolution(sensorAddresses[i], sensorsPrecision);
    }
}

void TemperatureSensor::update()
{
    sensors->requestTemperatures();
}

uint8_t TemperatureSensor::getDeviceCount()
{
    return sensorsFound;
}

float TemperatureSensor::read(uint8_t deviceIndex)
{
    return sensors->getTempC(sensorAddresses[deviceIndex]);
}