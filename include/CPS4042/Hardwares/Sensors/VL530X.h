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
        std::cout << "one instance of Vl530x created." << std::endl;
    }

    class I2C : public Protocols::AbstractI2C<Vl530x, Gpio>
    {
    public:
        explicit I2C(Vl530x* b) : Protocols::AbstractI2C<Vl530x, Gpio>{b} {}

        void init(Byte) override { m_state = State::WaitAddress; }
        void write(Byte) override {}
        Byte read() override { return 0; }

        void setData(uint16_t value) {
            std::cout << "[DEBUG] Slave setData: setting distance value " << value << " mm." << std::endl;
            uint8_t ub1 = (value >> 8) & 0xFF;
            uint8_t ub2 = value & 0xFF;
            uint8_t maxv = (ub1 > ub2) ? ub1 : ub2;
            uint8_t minv = (ub1 > ub2) ? ub2 : ub1;
            m_checksum = maxv - minv;
            m_b1 = ub1;
            m_b2 = ub2;
        }

        bool isReady() const { return m_state == State::WaitAddress; }

        void run(Gpio& gpio) override {
            switch (m_state) {

            case State::WaitAddress: {
                std::cout << "[DEBUG] Slave WaitAddress: waiting for address byte." << std::endl;
                if (!gpio.sda.hasByteToRead()) return;
                Byte raw = gpio.sda.read();
                Byte restored = raw;              
                Byte receivedAddr = (restored >> 1) & 0x7F;
                bool isRead = (restored & 1) == 1;
                if (receivedAddr == DEVICE_ADDRESS && isRead) {
                    std::cout << "[DEBUG] Slave WaitAddress: address byte received." << std::endl;
                    m_state = State::SendAck;
                }
                break;
            }


            case State::SendAck: {
                std::cout << "[DEBUG] Slave SendAck: sending ACK bit." << std::endl;
                if (!gpio.sda.hasBitToWrite()) {
                    gpio.sda.write(Bit::Zero);
                    m_byteIndex = 0; m_bitCounter = 0;
                    m_state = State::SendByte;
                }
                break;
            }

            case State::SendByte: {
                std::cout << "[DEBUG] Slave SendByte: sending data byte " << (int)m_byteIndex + 1 << "." << std::endl;
                Byte currentByte;
                if (m_byteIndex == 0) currentByte = m_b1;
                else if (m_byteIndex == 1) currentByte = m_b2;
                else currentByte = m_checksum;
                
                Bit bit = (currentByte >> (7 - m_bitCounter)) & 1 ? Bit::One : Bit::Zero;
                gpio.sda.write(bit);
                
                if (++m_bitCounter == 8) {
                    m_bitCounter = 0; ++m_byteIndex;
                    m_state = State::WaitMasterAck;
                }
                break;
            }
            case State::WaitMasterAck: {
                std::cout << "[DEBUG] Slave WaitMasterAck: waiting for master ACK." << std::endl;
                if (!gpio.sda.hasBitToRead()) return;
                Bit ack = gpio.sda.readBit();
                if (ack == Bit::Zero) {
                    std::cout << "[DEBUG] Slave WaitMasterAck: ACK received from master." << std::endl;
                    if (m_byteIndex < 3) m_state = State::SendByte;
                } else {
                    std::cout << "[DEBUG] Slave WaitMasterAck: NACK received from master, ending transmission." << std::endl;
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
        Byte m_b1 = 0, m_b2 = 0, m_checksum = 0;
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
