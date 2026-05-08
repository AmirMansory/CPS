#ifndef MICROCONTROLLER_USART_H
#define MICROCONTROLLER_USART_H

#include <CPS4042/Hardwares/Boards/Esp8266.h>
#include <CPS4042/Sketchs/AbstractSketch.h>
#include <iostream>

class MicroController_USART : public AbstractSketch<Boards::Esp8266>{
public:
    explicit MicroController_USART(Boards::Esp8266* node)
        : AbstractSketch<Boards::Esp8266>{node} {}

    std::int32_t setup(Boards::Esp8266::Gpio&) override{
        std::cout << "[USART] Setup micro" << std::endl;
        return 0;
    }

    std::int32_t loop(Boards::Esp8266::Gpio&) override
    {
        if (node()->usart.hasDataArrived()){
            Byte data = node()->usart.getReceivedByte();
            std::cout << "MCU: received data=0x" << std::hex
                      << static_cast<int>(static_cast<uint8_t>(data)) << std::dec << std::endl;
        }

        if (!node()->usart.hasRequestLock()){
            static uint8_t addr = 0;
            node()->usart.request(addr);
            std::cout << "MCU: requesting address 0x" << std::hex << (int)addr << std::endl;
            addr++;
            if (addr > 0x0A) addr = 0;   
        }

        static uint32_t tick = 0;
        if (++tick > 50000) tick = 0;
        return 0;
    }
};

#endif