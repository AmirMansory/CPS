#ifndef MICROCONTROLLER_I2C_H
#define MICROCONTROLLER_I2C_H

#include <CPS4042/Hardwares/Boards/Esp8266.h>
#include <CPS4042/Sketchs/AbstractSketch.h>
#include <iostream>
#include <cmath>

class MicroController_I2C : public AbstractSketch<Boards::Esp8266>{
public:
    explicit MicroController_I2C(Boards::Esp8266* node)
        : AbstractSketch<Boards::Esp8266>{node} {}

    std::int32_t setup(Boards::Esp8266::Gpio&) override{
        std::cout << "[I2C] Setup micro" << std::endl;
        return 0;
    }

    std::int32_t loop(Boards::Esp8266::Gpio&) override{
        if (node()->i2c.isDataAvailable()){
            Byte high = node()->i2c.read();
            Byte low  = node()->i2c.read();

            uint16_t distance = (static_cast<uint16_t>(static_cast<uint8_t>(high)) << 8)
                              | static_cast<uint8_t>(low);
            std::cout << ">>> DISTANCE = " << distance << " mm" << std::endl;
        }

        if (node()->i2c.isIdle())
            node()->i2c.write(0x29);  

        static uint32_t tick = 0;
        if (++tick > 50000) tick = 0;
        return 0;
    }
};

#endif