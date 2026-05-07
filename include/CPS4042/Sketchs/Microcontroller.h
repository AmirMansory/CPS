//Negin
#ifndef MICROCONTROLLER_H
#define MICROCONTROLLER_H

#include <CPS4042/Hardwares/Boards/Esp8266.h>
#include <CPS4042/Sketchs/AbstractSketch.h>
#include <vector>


class MicroController : public AbstractSketch<Boards::Esp8266>
{
public:
    explicit MicroController(Boards::Esp8266* node) :
        AbstractSketch<Boards::Esp8266> {node},
        m_addressIndex(0)
    {
        m_addressesToSend.clear();   // ← inside the body

        // Inside the body, local variable is fine
        const uint8_t temp[] = {
            0x00, 0x01, 0x10, 0x20, 0x30, 0x40, 0x50,
            0x60, 0x70, 0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0
        };

    for (auto val : temp) {
        m_addressesToSend.push_back(static_cast<Byte>(val));
    }
}

    std::int32_t setup(Boards::Esp8266::Gpio& gpio) override
    {
        std::cout << "ESP8266 setup completed. Starting USART communication..." << std::endl;
        return 0;
    }

    std::int32_t loop(Boards::Esp8266::Gpio& gpio) override
    {
        if(m_addressIndex < m_addressesToSend.size() && !m_waitingForResponse)
        {
            Byte address = m_addressesToSend[m_addressIndex];
            std::cout << "Micro: Sending address: 0x" << std::hex << (int)address << std::dec << std::endl;
            gpio.tx.write(address);
            m_waitingForResponse = true;
            m_sendTime = std::chrono::steady_clock::now();
        }
        
        if(m_waitingForResponse && gpio.rx.hasByteToRead())
        {
            Byte response = gpio.rx.read();
            std::cout << "Micro: Received response for address 0x" << std::hex << (int)m_addressesToSend[m_addressIndex] 
                      << " -> 0x" << (int)response << std::dec << std::endl;
            
            m_waitingForResponse = false;
            m_addressIndex++;
        }
        
        if(m_waitingForResponse)
        {
            auto now = std::chrono::steady_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(now - m_sendTime).count() >= 1)
            {
                std::cout << "Micro: Timeout for address 0x" << std::hex << (int)m_addressesToSend[m_addressIndex] << std::dec << std::endl;
                m_waitingForResponse = false;
                m_addressIndex++;
            }
        }
        
        return 0;
    }

private:
    std::vector<Byte> m_addressesToSend;
    int m_addressIndex;
    bool m_waitingForResponse = false;
    std::chrono::steady_clock::time_point m_sendTime;
};

#endif