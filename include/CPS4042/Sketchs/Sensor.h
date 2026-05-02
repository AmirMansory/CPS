#ifndef SENSOR_H
#define SENSOR_H

#include <CPS4042/Hardwares/Sensors/VL530X.h>
#include <CPS4042/Sketchs/AbstractSketch.h>
#include <cstdlib>
#include <ctime>

class Sensor : public AbstractSketch<Sensors::Vl530x>
{
public:
    explicit Sensor(Sensors::Vl530x* node)
        : AbstractSketch<Sensors::Vl530x>{node} {}

    std::int32_t setup(Sensors::Vl530x::Gpio&) override
    {
        std::srand(static_cast<unsigned>(std::time(nullptr)));
        std::cout << "[SETUP] Sensor ready" << std::endl;
        return 0;
    }

    std::int32_t loop(Sensors::Vl530x::Gpio&) override
    {
        if (node()->i2c.isReady())
        {
            uint16_t distance = std::rand() % 4001;   // 0 – 4000 mm
            node()->i2c.setData(distance);
        }
        delay(500);
        return 0;
    }
};

#endif