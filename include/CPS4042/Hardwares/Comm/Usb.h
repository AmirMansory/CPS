#ifndef USB_H
#define USB_H

#include <CPS4042/Hardwares/Board.h>
#include <CPS4042/Protocols/Protocol.h>
#include <CPS4042/Units/BaudRate.h>
#include <CPS4042/Wires/Pin.h>
#include <boost/pfr.hpp>
#include <iostream>
#include <unordered_map>

namespace Sensors
{
using UsbVoltage = VoltageLevel3_3v;

template <std::uint64_t bar, std::uint64_t btr, typename WorkingVoltageTp>
requires std::is_base_of_v<AbstractVoltageLevel, WorkingVoltageTp>
struct UsbGpio
{
public:
    Pins::Vdd<WorkingVoltageTp> vdd {bar, btr, "Usb::vdd"};
    Pins::Gnd<WorkingVoltageTp> gnd {bar, btr, "Usb::gnd"};
    Pins::Rx<WorkingVoltageTp>  rx  {bar, btr, "Usb::rx"};   // READ from this
    Pins::Tx<WorkingVoltageTp>  tx  {bar, btr, "Usb::tx"};   // WRITE to this
    Pins::Scl<WorkingVoltageTp> scl {bar, btr, "Usb::scl"};

};

class Usb : public Board<BaudRates::NotSpecified,
                         BitRates::same(BaudRates::NotSpecified),
                         Frequency::F320khz, UsbVoltage, UsbGpio>
{
public:
    explicit Usb() : Parent {"Usb::Processor"}
    {
        m_processor->installProtocol(&usart);
        std::cout << "[DEBUG] Usb (hard‑disk) created." << std::endl;
    }

    class USART : public Protocols::AbstractUsart<Usb, Gpio>
    {
    public:
        explicit USART(Usb* b) : Protocols::AbstractUsart<Usb, Gpio>{b} {}
        void setStorage(std::unordered_map<Byte, Byte>* map) { m_storage = map; }
        void write(Byte) override {}
        Byte read() override { return 0; }

        void run(Gpio& gpio) override
        {
            switch (m_rxState)
            {
            case RxState::Idle:
                if (gpio.tx.hasBitToRead()){
                    Bit b = gpio.tx.readBit();
                    if (b == Bit::Zero)            
                        m_rxState = RxState::Data;
                }
                break;
            case RxState::Data:
                if (gpio.tx.hasByteToRead()){
                    m_rxByte = gpio.tx.read();
                    m_rxState = RxState::Stop;
                }
                break;
            case RxState::Stop:
                if (gpio.tx.hasBitToRead()){
                    Bit stop = gpio.tx.readBit();
                    if (stop == Bit::One){
                        uint8_t addr = static_cast<uint8_t>(m_rxByte);
                        if (m_storage){
                            auto it = m_storage->find(m_rxByte);
                            if (it != m_storage->end()){
                                m_responseData = it->second;
                                m_responsePending = true;
                                std::cout << "[SLAVE] ✅ Addr 0x" << std::hex << static_cast<int>(addr)
                                        << " → Data 0x" << static_cast<int>(static_cast<uint8_t>(m_responseData))
                                        << std::dec << std::endl;
                            }
                            else{
                                std::cout << "[SLAVE] ⚠️ Addr 0x" << std::hex << static_cast<int>(addr)
                                        << " not found" << std::dec << std::endl;
                            }
                        }
                    }
                    m_rxState = RxState::Idle;    
                }
                break;
            }

            switch (m_txState)
            {
            case TxState::Idle:
                if (m_responsePending)
                    m_txState = TxState::Start;
                break;
            case TxState::Start:
                gpio.rx.write(Bit::Zero);       
                m_txState = TxState::Data;
                break;
            case TxState::Data:
                if (!gpio.rx.hasBitToWrite()){
                    gpio.rx.write(m_responseData);
                    m_txState = TxState::Stop;
                }
                break;
            case TxState::Stop:
                if (!gpio.rx.hasBitToWrite()){
                    gpio.rx.write(Bit::One);       
                    if (m_txStopSent){
                        m_txStopSent = false;
                        m_responsePending = false;
                        m_txState = TxState::Idle;
                        std::cout << "[SLAVE] 📤 Sent" << std::endl;
                    }
                    else{
                        m_txStopSent = true;
                    }
                }
                break;
            }
        }

    private:
        enum class RxState { Idle, Data, Stop };
        enum class TxState { Idle, Start, Data, Stop };
        RxState m_rxState = RxState::Idle;
        TxState m_txState = TxState::Idle;
        Byte    m_rxByte = 0;
        Byte    m_responseData = 0;
        bool    m_responsePending = false;
        bool    m_txStopSent = false;              
        std::unordered_map<Byte, Byte>* m_storage = nullptr;
    }mutable usart{this};

protected:
    void startModule() override {        
        m_gpio.scl.onNextEdge([this](UsbVoltage level) {
            auto bit = Voltage::toBit(level);
            if (bit == Bit::Zero) m_processor->nextCycle(m_gpio);
        });}
};

} // namespace Sensors
#endif
