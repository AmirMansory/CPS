#ifndef VL53_X_H
#define VL53_X_H

#include <CPS4042/Hardwares/Board.h>
#include <CPS4042/Units/BaudRate.h>
#include <CPS4042/Units/Byte.h>
#include <CPS4042/Wires/Pin.h>
#include <boost/pfr.hpp>
#include <iostream>

namespace Sensors
{
using Vl530xVoltage = VoltageLevel3_3v;

template <std::uint64_t bar, std::uint64_t btr, typename WorkingVoltageTp>
requires std::is_base_of_v<AbstractVoltageLevel, WorkingVoltageTp>
struct Vl530xGpio
{
public:
    Pins::Vdd<WorkingVoltageTp> vdd {bar, btr, "Vl530x::vdd"};
    Pins::Gnd<WorkingVoltageTp> gnd {bar, btr, "Vl530x::gnd"};
    Pins::Sda<WorkingVoltageTp> sda {bar, btr, "Vl530x::sda"};
    Pins::Scl<WorkingVoltageTp> scl {bar, btr, "Vl530x::scl"};
};

class Vl530x : public Board<BaudRates::NotSpecified,
                            BitRates::same(BaudRates::NotSpecified),
                            Frequency::Drived, Vl530xVoltage, Vl530xGpio>
{
public:
    static constexpr Byte address = 0x29;

    explicit Vl530x() : Parent {"Vl530x::processor"}
    {
        m_processor->installProtocol(&i2c);
        std::cout << "[DEBUG] Vl530x created." << std::endl;
    }

    class I2C : public Protocols::AbstractI2C<Vl530x, Gpio>
    {
    public:
        explicit I2C(Vl530x* b) : Protocols::AbstractI2C<Vl530x, Gpio>{b} {}

        void init(Byte) override { m_state = State::WaitAddress; }
        void write(Byte) override {}
        Byte read() override { return 0; }

        void setData(uint16_t value)
        {
            uint8_t ub1 = (value >> 8) & 0xFF;
            uint8_t ub2 = value & 0xFF;
            uint8_t maxv = (ub1 > ub2) ? ub1 : ub2;
            uint8_t minv = (ub1 > ub2) ? ub2 : ub1;
            m_checksum = maxv - minv;
            m_b1 = ub1;
            m_b2 = ub2;
            std::cout << ">>> [SLAVE] Data: " << value << " mm"
                      << " (0x" << std::hex << (int)ub1 << ",0x" << (int)ub2
                      << ",CS=0x" << (int)m_checksum << ")" << std::dec << std::endl;
        }

        bool isReady() const { return m_state == State::WaitAddress; }

        void run(Gpio& gpio) override
        {
            switch (m_state) {

            case State::WaitAddress: {
                if (!gpio.sda.hasByteToRead()) return;
                Byte raw = gpio.sda.read();
                Byte receivedAddr = (raw >> 1) & 0x7F;
                bool isRead = (raw & 1) == 1;
                if (receivedAddr == DEVICE_ADDRESS && isRead) {
                    std::cout << ">>> [SLAVE] Address match – ACK" << std::endl;
                    // flush RX before sending ACK
                    while (gpio.sda.hasBitToRead()) gpio.sda.readBit();
                    m_state = State::SendAck;
                }
                break;
            }

            case State::SendAck: {
                if (!gpio.sda.hasBitToWrite()) {
                    gpio.sda.write(Bit::Zero);
                    m_byteIndex = 0; m_bitCounter = 0;
                    m_state = State::SendByte;
                }
                break;
            }

            case State::SendByte: {
                Byte currentByte = (m_byteIndex == 0) ? m_b1 :
                                   (m_byteIndex == 1) ? m_b2 : m_checksum;
                Bit bit = (currentByte >> (7 - m_bitCounter)) & 1 ? Bit::One : Bit::Zero;
                gpio.sda.write(bit);
                if (++m_bitCounter == 8) {
                    // === DISCARD echoed data bits from own RX buffer ===
                    while (gpio.sda.hasBitToRead())
                        gpio.sda.readBit();
                    m_bitCounter = 0; ++m_byteIndex;
                    m_state = State::WaitMasterAck;
                }
                break;
            }

            case State::WaitMasterAck: {
                if (!gpio.sda.hasBitToRead()) return;
                Bit ack = gpio.sda.readBit();
                if (ack == Bit::Zero) {
                    std::cout << "[SLAVE] ACK – continue" << std::endl;
                    if (m_byteIndex < 3)
                        m_state = State::SendByte;
                } else {
                    std::cout << ">>> [SLAVE] NACK – done" << std::endl;
                    // flush everything before returning to idle
                    while (gpio.sda.hasBitToRead()) gpio.sda.readBit();
                    m_state = State::WaitAddress;
                }
                break;
            }

            default: break;
            }
        }

    private:
        enum class State : uint8_t { Idle, WaitAddress, SendAck, SendByte, WaitMasterAck };
        State m_state = State::WaitAddress;
        static constexpr Byte DEVICE_ADDRESS = 0x29;
        uint8_t m_b1 = 0, m_b2 = 0, m_checksum = 0;
        uint8_t m_bitCounter = 0, m_byteIndex = 0;
    } mutable i2c{this};

protected:
    void startModule() override
    {
        m_gpio.scl.onNextEdge([this](Vl530xVoltage level) {
            auto bit = Voltage::toBit(level);
            if (bit == Bit::One) m_processor->nextCycle(m_gpio);
        });
    }
};

} // namespace Sensors
#endif