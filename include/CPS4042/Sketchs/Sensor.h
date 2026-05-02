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

    std::int32_t setup(Sensors::Vl530x::Gpio&) override {
        std::cout << "[DEBUG] Sensor setup." << std::endl;
        std::srand(static_cast<unsigned>(std::time(nullptr)));
        return 0;
    }

    std::int32_t loop(Sensors::Vl530x::Gpio&) override {
        if (node()->i2c.isReady()) {
            std::cout << "[DEBUG] Sensor loop: I2C is ready, generating random distance." << std::endl;
            uint16_t distance = std::rand() % 4001;
            node()->i2c.setData(distance);
        }
        delay(200); // simulate some processing delay

        return 0;
    }
};

#endif