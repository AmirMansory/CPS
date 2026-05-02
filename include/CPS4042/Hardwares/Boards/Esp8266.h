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
        std::cout << "[DEBUG] Esp8266 created." << std::endl;
    }

    class I2C : public Protocols::AbstractI2C<Esp8266, Gpio>
    {
    public:
        explicit I2C(Esp8266* b) : Protocols::AbstractI2C<Esp8266, Gpio>{b} {}

        void init(Byte) override {
            std::cout << "[DEBUG] Master I2C init called (ignored)." << std::endl;
        }

        void write(Byte address) override {
            std::cout << "[DEBUG] Master write called with address 0x"<< std::hex << (int)(uint8_t)address << std::dec << std::endl;

            if (m_state != State::Idle && m_state != State::Done) {
                std::cout << "[DEBUG] Master busy, write ignored." << std::endl;
                return;
            }

            
            std::cout << "[DEBUG] Master-Write-flush before writing" << std::endl;
            while (m_board->m_gpio.sda.hasBitToRead()) {
                m_board->m_gpio.sda.readBit();
            }
            
            m_slaveAddress = address;
            Byte addr = (address << 1) | 0x01;
            std::cout << "[DEBUG] Sending addr byte 0x" << std::hex << (int)(uint8_t)addr << std::dec << std::endl;
            m_board->m_gpio.sda.write(addr);
            setState(State::SendAddress);
            m_transactionActive = true;
            resetTransaction();
        }

        Byte read() override {
            if (m_buffer.empty()) return 0;
            Byte v = m_buffer.front(); m_buffer.pop();
            std::cout << "[DEBUG] Master read byte 0x" << std::hex << (int)(uint8_t)v << std::dec << std::endl;
            return v;
        }

        bool isIdle() const {
            return m_state == State::Idle || m_state == State::Done;
        }

        void run(Gpio& gpio) override {
            if (!m_transactionActive) return;

            if (m_state != State::Idle && m_state != State::Done) {
                if (++m_timeoutCounter > TIMEOUT_CYCLES) {
                    std::cout << "[DEBUG] Master TIMEOUT in state " << (int)m_state << std::endl;
                    setState(State::Error);
                }
            }

            switch (m_state) {
            case State::SendAddress:
                std::cout << "[DEBUG] Master SendAddress: waiting for SCL to be high." << std::endl;
                if (m_board->m_gpio.sda.hasBitToWrite() > 0) return;
                setState(State::WaitAck);
                m_timeoutCounter = 0;
                break;

            case State::WaitAck: {
                std::cout << "[DEBUG] Master WaitAck: waiting for ACK bit." << std::endl;
                if (!m_board->m_gpio.sda.hasBitToRead()) return;
                Bit ack = m_board->m_gpio.sda.readBit();
                if (ack == Bit::Zero) {
                    setState(State::ReadByte);
                    m_timeoutCounter = 0;
                } else {
                    std::cout << "[DEBUG] Master WaitAck: NACK received, aborting." << std::endl;
                    setState(State::Error);
                }
                break;
            }

            case State::ReadByte: {
                std::cout << "[DEBUG] Master ReadByte: waiting for data bit." << std::endl;
                if (!gpio.sda.hasBitToRead()) return;
                Bit b = gpio.sda.readBit();
                m_currentByte = (m_currentByte << 1) | (b == Bit::One);
                if (++m_bitCounter == 8) {
                    std::cout << "[DEBUG] Master ReadByte: got full byte 0x"
                              << std::hex << (int)(uint8_t)m_currentByte << std::dec << std::endl;
                    storeByte(m_currentByte);
                    m_bitCounter = 0; m_currentByte = 0;
                    if (++m_byteCounter < PACKET_SIZE) {
                        setState(State::SendAck);
                    } else {
                        setState(State::SendNack);
                    }
                }
                break;
            }

            case State::SendAck:{
                std::cout << "[DEBUG] Master SendAck: sending ACK bit." << std::endl;
                gpio.sda.write(Bit::Zero);
                setState(State::ReadByte);
                break;
            }


            case State::SendNack:{
                std::cout << "[DEBUG] Master SendNack: sending NACK bit." << std::endl;
                gpio.sda.write(Bit::One);
                setState(State::Validate);
                break;
            }

            case State::Validate: {
                std::cout << "[DEBUG] Master Validate: validating received data." << std::endl;
                uint8_t ub1 = static_cast<uint8_t>(m_b1);
                uint8_t ub2 = static_cast<uint8_t>(m_b2);
                uint8_t uchecksum = static_cast<uint8_t>(m_checksum);
                uint8_t maxv = (ub1 > ub2) ? ub1 : ub2;
                uint8_t minv = (ub1 > ub2) ? ub2 : ub1;
                uint8_t expected = maxv - minv;
                
                std::cout << "[DEBUG] Master Validate: high=0x" << std::hex << (int)ub1
                          << " low=0x" << (int)ub2 << " checksum=0x" << (int)uchecksum
                          << " expected=0x" << (int)expected << std::dec << std::endl;
                          
                if (expected == uchecksum) {
                    std::cout << "[DEBUG] Checksum valid! Storing data." << std::endl;
                    m_buffer.push(m_b1);
                    m_buffer.push(m_b2);
                } else {
                    std::cout << "[DEBUG] Checksum INVALID!" << std::endl;
                }
                setState(State::Done);
                m_transactionActive = false;
                break;
            }

            case State::Error:{
                std::cout << "[DEBUG] Master ERROR: transaction failed." << std::endl;
                setState(State::Done);
                m_transactionActive = false;
                break;
            }

            case State::Done:{
                std::cout << "[DEBUG] Master DONE: transaction complete. "
                "Flush any leftover bits in the SDA buffer before next transaction" << std::endl;
                while (m_board->m_gpio.sda.hasBitToRead()) {
                    m_board->m_gpio.sda.readBit();   // discard stale bits
                }
                m_transactionActive = false;
                break;
            }
            default: break;
            }
        }

    private:
        enum class State : uint8_t {
            Idle = 0, SendAddress = 1, WaitAck = 2, ReadByte = 3,
            SendAck = 4, SendNack = 5, Validate = 6, Done = 7, Error = 8
        };

        void setState(State s) { m_state = s; }

        State m_state = State::Idle;
        Byte m_slaveAddress = 0x00;
        static constexpr uint8_t PACKET_SIZE = 3;
        static constexpr uint32_t TIMEOUT_CYCLES = 1000;
        uint32_t m_timeoutCounter = 0;
        uint8_t m_bitCounter = 0, m_byteCounter = 0;
        Byte m_currentByte = 0;
        Byte m_b1 = 0, m_b2 = 0, m_checksum = 0;
        bool m_transactionActive = false;

        void resetTransaction() {
            std::cout << "[DEBUG] Master resetTransaction: resetting transaction state." << std::endl;
            m_timeoutCounter = m_bitCounter = m_byteCounter = 0;
            m_currentByte = 0;
        }

        void storeByte(Byte b) {
            std::cout << "[DEBUG] Master storeByte: storing byte 0x" << std::hex << (int)(uint8_t)b << std::dec << std::endl;
            if (m_byteCounter == 0) m_b1 = b;
            else if (m_byteCounter == 1) m_b2 = b;
            else m_checksum = b;
        }
    } mutable i2c{this};

    class USART : public Protocols::AbstractUsart<Esp8266, Gpio>
    {
    public:
        explicit USART(Esp8266* b) : Protocols::AbstractUsart<Esp8266, Gpio>{b} {}
        void write(Byte byte) override {}
        Byte read() override { return 0; }
        void run(Gpio& gpio) override {}
    } mutable usart{this};

protected:
    void startModule() override {}
};

} // namespace Boards
#endif
