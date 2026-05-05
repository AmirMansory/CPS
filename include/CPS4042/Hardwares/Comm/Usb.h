#ifndef USB_H
#define USB_H

#include <CPS4042/Hardwares/Board.h>
#include <CPS4042/Protocols/Protocol.h>
#include <CPS4042/Units/BaudRate.h>
#include <CPS4042/Wires/Pin.h>

#include <functional>   // <-- required for std::function

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

        void setLookup(std::function<Byte(Byte)> fn) { m_lookupFn = fn; }

        struct Transaction {
            bool valid   = false;
            Byte address = 0;
            Byte data    = 0;
        };

        Transaction fetchLastTransaction() {
            auto t = m_lastTx;
            m_lastTx.valid = false;
            return t;
        }

        void run(Gpio& gpio) override {
            switch (m_state) {
                case State::IDLE:
                    if (gpio.rx.hasByteToRead()) {
                        m_reqAddr = gpio.rx.read();
                        m_state = State::SEND_DATA;
                    }
                    break;

                case State::SEND_DATA:
                    if (!gpio.tx.hasBitToWrite() && !gpio.tx.hasByteToWrite()) {
                        Byte data = m_lookupFn(m_reqAddr);
                        gpio.tx.write(data);

                        m_lastTx.address = m_reqAddr;
                        m_lastTx.data    = data;
                        m_lastTx.valid   = true;

                        m_state = State::IDLE;
                    }
                    break;
            }
        }

        void write(Byte) override {}
        Byte read() override { return 0; }

    private:
        enum class State { IDLE, SEND_DATA };
        State  m_state = State::IDLE;
        Byte   m_reqAddr = 0;
        Transaction m_lastTx;
        std::function<Byte(Byte)> m_lookupFn = [](Byte b) {
            return static_cast<Byte>(~b);   // explicit cast to avoid narrowing
        };
    } mutable usart{this};

protected:
    void startModule() override {}
};

} // namespace Sensors   // <-- namespace closed

#endif // USB_H