//Negin

#ifndef USB_H
#define USB_H

#include <CPS4042/Hardwares/Board.h>
#include <CPS4042/Units/BaudRate.h>
#include <CPS4042/Wires/Pin.h>
#include <queue>

namespace Sensors
{

using UsbVoltage = VoltageLevel3_3v;

template <std::uint64_t bar, std::uint64_t btr, typename WorkingVoltageTp>
struct UsbGpio
{
    Pins::Vdd<WorkingVoltageTp> vdd {bar, btr, "Usb::vdd"};
    Pins::Gnd<WorkingVoltageTp> gnd {bar, btr, "Usb::gnd"};
    Pins::Rx<WorkingVoltageTp>  rx  {bar, btr, "Usb::rx"};
    Pins::Tx<WorkingVoltageTp>  tx  {bar, btr, "Usb::tx"};
};

class Usb : public Board<BaudRates::B115200,
                         BitRates::same(BaudRates::B115200),
                         Frequency::Drived, UsbVoltage, UsbGpio>
{
public:
    explicit Usb() : Parent {"Usb::processor"}
    {
        m_processor->installProtocol(&usart);
        std::cout << "one instance of Usb created." << std::endl;
    }

    class USART : public Protocols::AbstractUsart<Usb, Gpio>
    {
    public:
        explicit USART(Usb* b) : 
            Protocols::AbstractUsart<Usb, Gpio> {b},
            m_isSending(false),
            m_isReceiving(false),
            m_bitIndex(0),
            m_currentByte(0),
            m_receivingByte(0)
        {}

        void write(Byte byte) override
        {
            m_txBuffer.push(byte);
        }

        Byte read() override
        {
            if(m_rxBuffer.empty()) return 0;
            Byte b = m_rxBuffer.front();
            m_rxBuffer.pop();
            return b;
        }

        void run(Gpio& gpio) override
        {
            if(!m_txBuffer.empty() && !m_isSending)
            {
                m_currentByte = m_txBuffer.front();
                m_txBuffer.pop();
                m_isSending = true;
                m_bitIndex = 0;
                gpio.tx.write(Bit::Zero);
                return;
            }
            
            if(m_isSending)
            {
                m_bitIndex++;
                
                if(m_bitIndex >= 1 && m_bitIndex <= 8)
                {
                    int bitPos = m_bitIndex - 1;
                    Bit bit = ((m_currentByte >> bitPos) & 1) ? Bit::One : Bit::Zero;
                    gpio.tx.write(bit);
                }
                else if(m_bitIndex == 9)
                {
                    int ones = 0;
                    for(int i = 0; i < 8; i++)
                        if((m_currentByte >> i) & 1) ones++;
                    Bit parity = (ones % 2 == 0) ? Bit::Zero : Bit::One;
                    gpio.tx.write(parity);
                }
                else if(m_bitIndex == 10)
                {
                    gpio.tx.write(Bit::One);
                    m_isSending = false;
                    m_bitIndex = 0;
                }
            }
            
            if(gpio.rx.hasBitToRead())
            {
                Bit bit = gpio.rx.readBit();
                
                if(!m_isReceiving)
                {
                    if(bit == Bit::Zero)
                    {
                        m_isReceiving = true;
                        m_bitIndex = 0;
                        m_receivingByte = 0;
                    }
                }
                else
                {
                    m_bitIndex++;
                    
                    if(m_bitIndex >= 1 && m_bitIndex <= 8)
                    {
                        if(bit == Bit::One)
                            m_receivingByte |= (1 << (m_bitIndex - 1));
                    }
                    else if(m_bitIndex == 9)
                    {
                        // parity - ignore
                    }
                    else if(m_bitIndex == 10)
                    {
                        m_isReceiving = false;
                        m_rxBuffer.push(m_receivingByte);
                        m_bitIndex = 0;
                    }
                }
            }
        }

    private:
        std::queue<Byte> m_txBuffer;
        std::queue<Byte> m_rxBuffer;
        bool m_isSending;
        bool m_isReceiving;
        int m_bitIndex;
        Byte m_currentByte;
        Byte m_receivingByte;
        
    } mutable usart {this};

protected:
    void startModule() override {}
};

} // namespace Sensors

#endif