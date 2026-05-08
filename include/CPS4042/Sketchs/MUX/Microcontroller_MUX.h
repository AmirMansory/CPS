#ifndef MICROCONTROLLER_MUX_H
#define MICROCONTROLLER_MUX_H

#include <CPS4042/Hardwares/Boards/Esp8266.h>
#include <CPS4042/Sketchs/AbstractSketch.h>
#include <iostream>

class MicroController_MUX : public AbstractSketch<Boards::Esp8266>{
public:
    explicit MicroController_MUX(Boards::Esp8266* node)
        : AbstractSketch<Boards::Esp8266>{node} {}

    std::int32_t setup(Boards::Esp8266::Gpio&) override{
        std::cout << "[MUX] Setup micro" << std::endl;
        return 0;
    }

    std::int32_t loop(Boards::Esp8266::Gpio&) override
    {
        if (node()->usart.hasDataArrived()){
            Byte data = node()->usart.getReceivedByte();
            uint8_t val = static_cast<uint8_t>(data);
            std::cout << "Channel " << (int)m_currentChannel
                      << " -> value: " << (int)val << std::endl;
            m_waitingForReply = false;
        }

        if (!node()->usart.hasRequestLock() && !m_waitingForReply){
            if (++m_currentChannel > 3) m_currentChannel = 0;
            node()->usart.request(m_currentChannel);   
            std::cout << "MCU: requesting channel " << (int)m_currentChannel << std::endl;
            m_waitingForReply = true;
        }

        static uint32_t tick = 0;
        if (++tick > 50000) tick = 0;
        return 0;
    }

private:
    uint8_t m_currentChannel = 0;
    bool    m_waitingForReply = false;
};

#endif