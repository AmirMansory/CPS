#ifndef SENSOR_I2C_H
#define SENSOR_I2C_H

#include <CPS4042/Hardwares/Sensors/VL530X.h>    
#include <CPS4042/Sketchs/AbstractSketch.h>
#include <cstdlib>
#include <ctime>

class Sensor_I2C : public AbstractSketch<Sensors::Vl530x>{
public:
    explicit Sensor_I2C(Sensors::Vl530x* node) : AbstractSketch{node} {}

    std::int32_t setup(Sensors::Vl530x::Gpio&) override{
        std::srand(static_cast<unsigned>(std::time(nullptr)));
        uint16_t distance = std::rand() % 4001;          
        node()->i2c.setData(distance);                    
        std::cout << "[I2C SENSOR] Ready" << std::endl;
        return 0;
    }

    std::int32_t loop(Sensors::Vl530x::Gpio&) override{
        if (node()->i2c.isReady()){
            uint16_t distance = std::rand() % 4001;
            std::cout << "sensing : " << distance << " mm" << std::endl;
            node()->i2c.setData(distance);
        }
        delay(70);
        return 0;
    }
};

#endif