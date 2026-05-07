#ifndef USB_H
#define USB_H

#include <CPS4042/Hardwares/Board.h>
#include <CPS4042/Protocols/Protocol.h>
#include <CPS4042/Units/BaudRate.h>
#include <CPS4042/Wires/Pin.h>
#include <CPS4042/Units/Bit.h>
#include <functional>

namespace Sensors {

using UsbVoltage = VoltageLevel3_3v;

template <std::uint64_t bar, std::uint64_t btr, typename WorkingVoltageTp>
requires std::is_base_of_v<AbstractVoltageLevel, WorkingVoltageTp>
struct UsbGpio {
    Pins::Vdd<WorkingVoltageTp> vdd {bar, btr, "Usb::vdd"};
    Pins::Gnd<WorkingVoltageTp> gnd {bar, btr, "Usb::gnd"};
    Pins::Rx<WorkingVoltageTp>  rx  {bar, btr, "Usb::rx"};   // disk WRITES here
    Pins::Tx<WorkingVoltageTp>  tx  {bar, btr, "Usb::tx"};   // disk READS from here
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

    class USART : public Protocols::AbstractUsart<Usb, Gpio> {
    public:
        explicit USART(Usb* b) : Protocols::AbstractUsart<Usb, Gpio>{b} {}

        void setLookup(std::function<Byte(Byte)> fn) { m_lookupFn = fn; }

        struct Transaction {
            bool valid = false;
            Byte address = 0;
            Byte data = 0;
        };

        Transaction fetchLastTransaction() {
            Transaction t = m_lastTx;
            m_lastTx.valid = false;
            return t;
        }

        void run(Gpio& gpio) override {
            switch (m_state) {
                // ----- Wait for a request (read from TX pin) -----
                case State::WAIT_START:
                    if (gpio.tx.hasBitToRead()) {
                        Bit b = gpio.tx.readBit();
                        if (b == Bit::Zero) {
                            m_rxByte = 0;
                            m_bitIndex = 0;
                            m_state = State::WAIT_DATA;
                        }
                    }
                    break;

                case State::WAIT_DATA:
                    while (gpio.tx.hasBitToRead()) {
                        Bit b = gpio.tx.readBit();
                        if (b == Bit::One) m_rxByte |= (1 << m_bitIndex);
                        ++m_bitIndex;
                        if (m_bitIndex == 8) {
                            m_state = State::WAIT_STOP;
                            break;
                        }
                    }
                    break;

                case State::WAIT_STOP:
                    if (gpio.tx.hasBitToRead()) {
                        Bit b = gpio.tx.readBit();
                        if (b == Bit::One) {
                            // Request received → lookup data and prepare response
                            Byte addr = m_rxByte;
                            Byte data = m_lookupFn(addr);
                            m_txByte = data;
                            m_lastTx.address = addr;
                            m_lastTx.data = data;
                            m_lastTx.valid = true;
                            m_state = State::SEND_START;
                        } else {
                            m_state = State::WAIT_START; // framing error, try again
                        }
                    }
                    break;

                // ----- Send response (write on RX pin) -----
                case State::SEND_START:
                    if (!gpio.rx.hasBitToWrite() && !gpio.rx.hasByteToWrite()) {
                        gpio.rx.write(Bit::Zero);
                        m_state = State::SEND_DATA;
                        m_bitIndex = 0;
                    }
                    break;

                case State::SEND_DATA:
                    if (!gpio.rx.hasBitToWrite() && !gpio.rx.hasByteToWrite()) {
                        gpio.rx.write(takeNthBit(m_txByte, m_bitIndex));
                        ++m_bitIndex;
                        if (m_bitIndex == 8) m_state = State::SEND_STOP;
                    }
                    break;

                case State::SEND_STOP:
                    if (!gpio.rx.hasBitToWrite() && !gpio.rx.hasByteToWrite()) {
                        gpio.rx.write(Bit::One);
                        m_state = State::WAIT_START;   // ready for next request
                    }
                    break;
            }
        }

        void write(Byte) override {}
        Byte read() override { return 0; }

    private:
        enum class State { WAIT_START, WAIT_DATA, WAIT_STOP,
                           SEND_START, SEND_DATA, SEND_STOP };
        State m_state = State::WAIT_START;
        Byte  m_rxByte = 0;
        Byte  m_txByte = 0;
        int   m_bitIndex = 0;
        Transaction m_lastTx;
        std::function<Byte(Byte)> m_lookupFn = [](Byte b) { return static_cast<Byte>(~b); };
    } mutable usart{this};

protected:
    void startModule() override {}
};

} // namespace Sensors
#endif