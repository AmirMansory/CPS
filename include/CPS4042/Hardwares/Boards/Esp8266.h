#ifndef ESP8266_H
#define ESP8266_H

#include <CPS4042/Hardwares/Board.h>
#include <CPS4042/Protocols/Protocol.h>
#include <CPS4042/Units/BaudRate.h>
#include <CPS4042/Units/Byte.h>
#include <CPS4042/Wires/Pin.h>
#include <boost/pfr.hpp>
#include <iostream>

namespace Boards
{
using Esp8266Voltage = VoltageLevel3_3v;

template <BaudRate BR, BitRate BTR, typename WorkingVoltageTp>
requires std::is_base_of_v<AbstractVoltageLevel, WorkingVoltageTp>
struct Esp8266Gpio
{
public:
    Pins::Vdd<WorkingVoltageTp>     vdd1 {BR, BTR, "Esp8266::vdd1"};
    Pins::Gnd<WorkingVoltageTp>     gnd1 {BR, BTR, "Esp8266::gnd1"};
    Pins::Vdd<WorkingVoltageTp>     vdd2 {BR, BTR, "Esp8266::vdd2"};
    Pins::Gnd<WorkingVoltageTp>     gnd2 {BR, BTR, "Esp8266::gnd2"};
    Pins::Vdd<WorkingVoltageTp>     vdd3 {BR, BTR, "Esp8266::vdd3"};
    Pins::Gnd<WorkingVoltageTp>     gnd3 {BR, BTR, "Esp8266::gnd3"};
    Pins::Rx<WorkingVoltageTp>      rx {BR, BTR, "Esp8266::rx"};
    Pins::Tx<WorkingVoltageTp>      tx {BR, BTR, "Esp8266::tx"};
    Pins::Sda<WorkingVoltageTp>     sda {BR, BTR, "Esp8266::sda"};
    Pins::Scl<WorkingVoltageTp>     scl {BR, BTR, "Esp8266::scl"};
    Pins::Digital<WorkingVoltageTp> d0 {BR, BTR, "Esp8266::d0"};
    Pins::Digital<WorkingVoltageTp> d1 {BR, BTR, "Esp8266::d1"};
    Pins::Digital<WorkingVoltageTp> d2 {BR, BTR, "Esp8266::d2"};
    Pins::Digital<WorkingVoltageTp> d3 {BR, BTR, "Esp8266::d3"};
    Pins::Digital<WorkingVoltageTp> d4 {BR, BTR, "Esp8266::d4"};
    Pins::Digital<WorkingVoltageTp> d5 {BR, BTR, "Esp8266::d5"};
    Pins::Analog<WorkingVoltageTp>  a0 {BR, BTR, "Esp8266::a1"};
};

class Esp8266 : public Board<BaudRates::B115200, BitRates::same(BaudRates::B115200),
                             Frequency::F320khz, Esp8266Voltage, Esp8266Gpio>
{
public:
    explicit Esp8266() : Parent {"Esp8266::Processor"}
    {
        m_processor->communicationClockChanged.connect(
          [this](Bit edge) { m_gpio.scl.nextEdge(edge); });
        m_processor->installProtocol(&i2c);
        m_processor->installProtocol(&usart);
    }

    class I2C : public Protocols::AbstractI2C<Esp8266, Gpio>
    {
    public:
        explicit I2C(Esp8266* b) : Protocols::AbstractI2C<Esp8266, Gpio>{b} {}

        void init(Byte) override {}

        void write(Byte address) override
        {
            if (m_state != State::Idle && m_state != State::Done) return;

            // Clean start: flush all echoed bits from previous transaction
            while (m_board->m_gpio.sda.hasBitToRead())
                m_board->m_gpio.sda.readBit();

            std::cout << ">>> [MASTER] Start -> addr 0x"
                      << std::hex << (int)(uint8_t)address << std::dec << std::endl;

            m_slaveAddress = address;
            Byte addr = (address << 1) | 0x01;
            m_board->m_gpio.sda.write(addr);
            m_state = State::SendAddress;
            m_transactionActive = true;
            resetTransaction();
        }

        Byte read() override
        {
            if (m_buffer.empty()) return 0;
            Byte v = m_buffer.front(); m_buffer.pop();
            return v;
        }

        bool isIdle() const { return m_state == State::Idle || m_state == State::Done; }

        void run(Gpio& gpio) override
        {
            if (!m_transactionActive) return;

            if (m_state != State::Idle && m_state != State::Done)
            {
                if (++m_timeoutCounter > TIMEOUT_CYCLES) {
                    std::cout << "[MASTER] Timeout" << std::endl;
                    m_state = State::Error;
                }
            }

            switch (m_state) {

            // ---- send address byte ----
            case State::SendAddress:
                if (m_board->m_gpio.sda.hasBitToWrite() > 0) return;
                m_state = State::WaitAck;
                m_timeoutCounter = 0;
                break;

            // ---- wait for slave ACK ----
            case State::WaitAck: {
                if (!m_board->m_gpio.sda.hasBitToRead()) return;
                Bit ack = m_board->m_gpio.sda.readBit();
                if (ack == Bit::Zero) {
                    std::cout << ">>> [MASTER] ACK received" << std::endl;
                    m_state = State::ReadByte;
                    m_timeoutCounter = 0;
                } else {
                    std::cout << "[MASTER] NACK - abort" << std::endl;
                    m_state = State::Error;
                }
                break;
            }

            // ---- read one data byte (bit by bit) ----
            case State::ReadByte: {
                if (!gpio.sda.hasBitToRead()) return;
                Bit b = gpio.sda.readBit();
                m_currentByte = (m_currentByte << 1) | (b == Bit::One);
                if (++m_bitCounter == 8) {
                    std::cout << ">>> [MASTER] Byte: 0x"
                              << std::hex << (int)(uint8_t)m_currentByte << std::dec << std::endl;
                    storeByte(m_currentByte);
                    m_bitCounter = 0; m_currentByte = 0;
                    if (++m_byteCounter < PACKET_SIZE)
                        m_state = State::SendAck;
                    else
                        m_state = State::SendNack;
                }
                break;
            }

            // ---- send ACK and then flush its echo ----
            case State::SendAck:
                gpio.sda.write(Bit::Zero);
                m_state = State::FlushAckEcho;
                break;

            // ---- swallow the echoed ACK bit before reading next byte ----
            case State::FlushAckEcho:
                if (m_board->m_gpio.sda.hasBitToRead()) {
                    m_board->m_gpio.sda.readBit();  // discard echoed ACK
                    m_state = State::ReadByte;
                }
                break;

            // ---- send NACK ----
            case State::SendNack:
                gpio.sda.write(Bit::One);
                m_state = State::Validate;
                break;

            // ---- validate checksum ----
            case State::Validate: {
                uint8_t ub1 = static_cast<uint8_t>(m_b1);
                uint8_t ub2 = static_cast<uint8_t>(m_b2);
                uint8_t ucs = static_cast<uint8_t>(m_checksum);
                uint8_t maxv = (ub1 > ub2) ? ub1 : ub2;
                uint8_t minv = (ub1 > ub2) ? ub2 : ub1;

                if (maxv - minv == ucs) {
                    std::cout << ">>> [MASTER] Checksum OK" << std::endl;
                    m_buffer.push(m_b1);
                    m_buffer.push(m_b2);
                } else {
                    std::cout << "[MASTER] Checksum FAIL (expected 0x"
                              << std::hex << (int)(maxv - minv) << ")" << std::dec << std::endl;
                }
                m_state = State::Done;
                m_transactionActive = false;
                break;
            }

            // ---- error / done ----
            case State::Error:
                m_state = State::Done;
                m_transactionActive = false;
                break;

            case State::Done:
                // Final flush of any leftover echoes
                while (m_board->m_gpio.sda.hasBitToRead())
                    m_board->m_gpio.sda.readBit();
                m_transactionActive = false;
                break;

            default: break;
            }
        }

    private:
        enum class State : uint8_t {
            Idle, SendAddress, WaitAck, ReadByte,
            SendAck, FlushAckEcho, SendNack, Validate, Done, Error
        };
        State m_state = State::Idle;
        Byte m_slaveAddress = 0x00;
        static constexpr uint8_t PACKET_SIZE = 3;
        static constexpr uint32_t TIMEOUT_CYCLES = 2000;
        uint32_t m_timeoutCounter = 0;
        uint8_t m_bitCounter = 0, m_byteCounter = 0;
        Byte m_currentByte = 0;
        Byte m_b1 = 0, m_b2 = 0, m_checksum = 0;
        bool m_transactionActive = false;

        void resetTransaction() {
            m_timeoutCounter = m_bitCounter = m_byteCounter = 0;
            m_currentByte = 0;
        }

        void storeByte(Byte b) {
            if (m_byteCounter == 0) m_b1 = b;
            else if (m_byteCounter == 1) m_b2 = b;
            else m_checksum = b;
        }
    } mutable i2c{this};

    class USART : public Protocols::AbstractUsart<Esp8266, Gpio>
    {
    public:
        explicit USART(Esp8266* b) : Protocols::AbstractUsart<Esp8266, Gpio>{b} {}
        void write(Byte) override {}
        Byte read() override { return 0; }
        void run(Gpio&) override {}
    } mutable usart{this};

protected:
    void startModule() override {}
};

} // namespace Boards
#endif