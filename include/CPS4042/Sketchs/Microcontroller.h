#ifndef MICROCONTROLLER_H
#define MICROCONTROLLER_H

#include <CPS4042/Hardwares/Boards/Esp8266.h>
#include <CPS4042/Sketchs/AbstractSketch.h>
#include <iostream>

class MicroController : public AbstractSketch<Boards::Esp8266>
{
public:
    explicit MicroController(Boards::Esp8266* node)
        : AbstractSketch<Boards::Esp8266>{node} {}

    std::int32_t setup(Boards::Esp8266::Gpio&) override
    {
        std::cout << "[SETUP] Microcontroller ready" << std::endl;
        node()->i2c.write(0x29);
        return 0;
    }

    std::int32_t loop(Boards::Esp8266::Gpio&) override
    {

        // ================= I2C part =================
        if (node()->i2c.isDataAvailable()){
            Byte high = node()->i2c.read();
            Byte low  = node()->i2c.read();

            uint16_t distance = (static_cast<uint16_t>(static_cast<uint8_t>(high)) << 8)
                              | static_cast<uint8_t>(low);
            std::cout << ">>> DISTANCE = " << distance << " mm" << std::endl;
        }

        if (node()->i2c.isIdle()){
            node()->i2c.write(0x29);
        }

        // ================= USART part =================
        static uint8_t addr = 0;

        if (!node()->usart.hasResponse() && !node()->usart.isBusy()) {  
            node()->usart.request(addr);
        }

        if (node()->usart.hasResponse()) {
            Byte data = node()->usart.getResponse();
            std::cout << "MCU USART: addr=" << (int)addr << " -> " << (int)data << std::endl;
            addr = (addr + 1) % 256;
            node()->usart.request(addr);
        }   

        delay(500);
        return 0;
    }
};

#endif