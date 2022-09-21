#ifndef TEMPERATURE_SENSOR_H_
#define TEMPERATURE_SENSOR_H_

#include <cstdint>
#include <DallasTemperature.h>

class TemperatureSensor
{
protected:
    DallasTemperature *sensors;

    uint8_t sensorsPrecision;

    /**
     * Addresses of found temperature sensors
     */
    DeviceAddress *sensorAddresses;

    /**
     * Number of temperature sensors found on the bus
     */
    uint8_t sensorsFound = 0;

public:
    /**
     * Temperatur precision, 9, 10, 11, or 12 bits
     */
    static const uint8_t kDefaultSensorsPrecision = 9;

    /**
     * @param dallasTemp Pointer to the DallasTemperature instance
     * @param sensorsMax Sensors buffer size
     * @param sensorsPrecision Temperatur sensors precision, 9, 10, 11, or 12 bits
     */
    TemperatureSensor(
        DallasTemperature *dallasTemp,
        uint8_t sensorsMax,
        uint8_t sensorsPrecision = kDefaultSensorsPrecision);

    void begin();

    void update();

    uint8_t getDeviceCount();

    float read(uint8_t deviceIndex);
};

#endif // TEMPERATURE_SENSOR_H_
