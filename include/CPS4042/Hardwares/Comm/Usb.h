#ifndef USB_H
#define USB_H

#include <CPS4042/Hardwares/Board.h>
#include <CPS4042/Protocols/Protocol.h>
#include <CPS4042/Units/BaudRate.h>
#include <CPS4042/Wires/Pin.h>

namespace Sensors {

using UsbVoltage = VoltageLevel3_3v;

template <std::uint64_t bar, std::uint64_t btr, typename WorkingVoltageTp>
requires std::is_base_of_v<AbstractVoltageLevel, WorkingVoltageTp>
struct UsbGpio {
    Pins::Vdd<WorkingVoltageTp> vdd {bar, btr, "Usb::vdd"};
    Pins::Gnd<WorkingVoltageTp> gnd {bar, btr, "Usb::gnd"};
    Pins::Rx<WorkingVoltageTp>  rx  {bar, btr, "Usb::rx"};
    Pins::Tx<WorkingVoltageTp>  tx  {bar, btr, "Usb::tx"};
};

class Usb : public Board<BaudRates::NotSpecified,
                         BitRates::same(BaudRates::NotSpecified),
                         Frequency::F320khz, UsbVoltage, UsbGpio>
{
public:
    explicit Usb() : Parent{"Usb::Processor"} {
        m_processor->installProtocol(&usart);
        std::cout << "one instance of Usb created." << std::endl;
    }

    // USART protocol class with state machine
    class USART : public Protocols::AbstractUsart<Usb, Gpio> {
    public:
        explicit USART(Usb* b) : Protocols::AbstractUsart<Usb, Gpio>{b} {}

        void run(Gpio& gpio) override {
            if (m_state == State::IDLE) {
                if (gpio.rx.hasByteToRead()) {
                    Byte addr = gpio.rx.read();
                    std::cout << "Disk: received address " << (int)addr << std::endl;
                    m_reqAddr = addr;
                    m_state = State::SEND_DATA;
                }
            }
            else if (m_state == State::SEND_DATA) {
                // Wait until no pending writes, then send
                if (!gpio.tx.hasBitToWrite() && !gpio.tx.hasByteToWrite()) {
                    Byte data = m_lookup(m_reqAddr);
                    std::cout << "Disk: sending " << (int)data << std::endl;
                    gpio.tx.write(data);
                    m_state = State::IDLE;
                }
            }
        }

        void write(Byte) override {}
        Byte read() override { return 0; }

    private:
        enum class State { IDLE, SEND_DATA };
        State m_state = State::IDLE;
        Byte  m_reqAddr = 0;
        // Sample lookup: just return complement of address (or use the map from HardDisk)
        Byte m_lookup(Byte addr) { return ~addr; }
    } mutable usart{this};

protected:
    void startModule() override {}
};

}   // namespace Sensors
#endif