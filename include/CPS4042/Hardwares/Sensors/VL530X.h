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

        void init(Byte address) override {m_state = State::WaitAddress; }
        void write(Byte) override {}
        Byte read() override { return 0; }

        void setData(uint16_t value)
        {
            uint8_t ub1 = (value >> 8) & 0xFF;
            uint8_t ub2 = value & 0xFF;
            m_checksum = abs(ub1 - ub2);
            m_b1 = ub1;
            m_b2 = ub2;
        }

        bool isReady() const { return m_state == State::WaitAddress;}

        void run(Gpio& gpio) override
        {
            switch (m_state) {

            case State::Idle:
                raw = 0;
                receivedAddr = 0;
                isRead = {false};
                m_state = State::WaitAddress;
            // ---- wait for I2C address from master ----
            case State::WaitAddress: {
                if (!gpio.sda.hasByteToRead()) return;
                raw = gpio.sda.read();
                receivedAddr = (raw >> 1) & 0x7F;
                isRead = (raw & 1) == 1;
                
                printf("[SLAVE] Address byte received: 0x%02X (addr=0x%02X, %s)\n",
                    raw, receivedAddr, isRead ? "READ" : "WRITE");
                
                if (receivedAddr == DEVICE_ADDRESS && isRead) {
                    printf("[SLAVE] ✅ Address match (0x%02X) – will ACK\n", DEVICE_ADDRESS);
                    m_state = State::SendAck;
                } else {
                    printf("[SLAVE] ❌ Address mismatch (expected 0x%02X) – ignoring\n", DEVICE_ADDRESS);
                    m_state = State::Idle;
                    raw = 0;
                    receivedAddr = 0;
                    isRead = {false};
                }
                break;
            }

            // ---- send ACK bit to master and flush any pending data ----
            case State::SendAck: {
                printf("[SLAVE] Entering SendAck state\n");
                gpio.sda.write(Bit::One);
                m_state = State::SendByte;

                break;
            }

            // ---- send the three data bytes (b1, b2, checksum) ----
            case State::SendByte: {
                if (!gpio.sda.hasBitToWrite()) {
                    printf("[SLAVE] 📤 Sending bytes: b1=0x%02X, b2=0x%02X, cs=0x%02X\n",
                        m_b1, m_b2, m_checksum);
                    gpio.sda.write({m_b1, m_b2, m_checksum});
                    m_state = State::WaitNack;
                } else {
                    printf("[SLAVE] Cannot send bytes – SDA not ready for write\n");
                }
                break;
            }

            case State::WaitNack:
                if (!gpio.sda.hasBitToRead()) return;
                nack = gpio.sda.readBit();
                if (nack == Bit::One)
                    std::cout << "Slave wait nack" << std::endl;
                    m_state = State::Idle;
                while(gpio.sda.hasBitToRead())
                    gpio.sda.readBit();

            default: break;
            }
        }

    private:
        enum class State : uint8_t {Idle, WaitAddress, SendAck, SendByte, WaitNack};
        State m_state = State::WaitAddress;
        static constexpr Byte DEVICE_ADDRESS = 0x29;
        Byte m_b1 = 0, m_b2 = 0, m_checksum = 0;
        Byte raw = 0, receivedAddr = 0;
        bool isRead = {false};
        Bit nack = Bit::One;
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


#endif // VL53_X_H