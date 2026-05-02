#ifndef ESP8266_H
#define ESP8266_H

#include <CPS4042/Hardwares/Board.h>
#include <CPS4042/Protocols/Protocol.h>
#include <CPS4042/Units/BaudRate.h>
#include <CPS4042/Wires/Pin.h>
#include <boost/pfr.hpp>

namespace Boards
{

using Esp8266Voltage = VoltageLevel3_3v;

template <BaudRate BR, BitRate BTR, typename WorkingVoltageTp>
requires std::is_base_of_v<AbstractVoltageLevel, WorkingVoltageTp>
struct Esp8266Gpio
{
public:
    Pins::Vdd<WorkingVoltageTp>     vdd1 {BR, BTR, "Esp8266::vdd1"};    // Pin 0
    Pins::Gnd<WorkingVoltageTp>     gnd1 {BR, BTR, "Esp8266::gnd1"};    // Pin 1

    Pins::Vdd<WorkingVoltageTp>     vdd2 {BR, BTR, "Esp8266::vdd2"};    // Pin 2
    Pins::Gnd<WorkingVoltageTp>     gnd2 {BR, BTR, "Esp8266::gnd2"};    // Pin 3

    Pins::Vdd<WorkingVoltageTp>     vdd3 {BR, BTR, "Esp8266::vdd3"};    // Pin 4
    Pins::Gnd<WorkingVoltageTp>     gnd3 {BR, BTR, "Esp8266::gnd3"};    // Pin 5

    Pins::Rx<WorkingVoltageTp>      rx {BR, BTR, "Esp8266::rx"};        // Pin 6
    Pins::Tx<WorkingVoltageTp>      tx {BR, BTR, "Esp8266::tx"};        // Pin 7

    Pins::Sda<WorkingVoltageTp>     sda {BR, BTR, "Esp8266::sda"};      // Pin 8
    Pins::Scl<WorkingVoltageTp>     scl {BR, BTR, "Esp8266::scl"};      // Pin 9

    Pins::Digital<WorkingVoltageTp> d0 {BR, BTR, "Esp8266::d0"};    // Pin 10
    Pins::Digital<WorkingVoltageTp> d1 {BR, BTR, "Esp8266::d1"};    // Pin 11
    Pins::Digital<WorkingVoltageTp> d2 {BR, BTR, "Esp8266::d2"};    // Pin 12
    Pins::Digital<WorkingVoltageTp> d3 {BR, BTR, "Esp8266::d3"};    // Pin 13
    Pins::Digital<WorkingVoltageTp> d4 {BR, BTR, "Esp8266::d4"};    // Pin 14
    Pins::Digital<WorkingVoltageTp> d5 {BR, BTR, "Esp8266::d5"};    // Pin 15
    Pins::Analog<WorkingVoltageTp>  a0 {BR, BTR, "Esp8266::a1"};    // Pin 16
};

class Esp8266
    : public Board<BaudRates::B115200, BitRates::same(BaudRates::B115200),
                   Frequency::F320khz, Esp8266Voltage, Esp8266Gpio>
{
public:
    explicit Esp8266() :
        Parent {"Esp8266::Processor"}
    {
        m_processor->communicationClockChanged.connect(
          [this](Bit edge) { m_gpio.scl.nextEdge(edge); });

        m_processor->installProtocol(&i2c);
        m_processor->installProtocol(&usart);

        std::cout << "one instance of Esp8266" << " created." << std::endl;
    };

    class I2C : public Protocols::AbstractI2C<Esp8266, Gpio>
    {
    private: 
        static constexpr uint8_t PAKET_SIZE = 2; // Define the expected packet size
        static constexpr uint32_t TIMEOUT_CYCLES = 10000; // Timeout cycles for I2C transactions

        enum class State {
            Idle,
            SendAddress,
            WaitAck,
            ReadByte,
            SendAck,
            SendNack,
            Validate,
            Done,
            Error
        };

        State m_state = State::Idle;
        Byte m_slaveAddress = 0x00;

        uint32_t m_timeoutCounter = 0;
        uint8_t m_bitCounter = 0;
        uint8_t m_byteCounter = 0;
        Byte m_currentByte = 0x00;
        Byte m_b1 = 0x00, m_b2 = 0x00, m_checksum = 0x00;
        bool m_transactionActive = false;

    public:
        explicit I2C(Esp8266* b) :
            Protocols::AbstractI2C<Esp8266, Gpio> {b}
        {}

        
        void
        init(Byte address) override
        {}

        void
        write(Byte address) override{
            if (m_state != State::Idle && m_state != State::Done) {
                std::cerr << "I2C: Cannot start new transaction, bus is busy." << std::endl;
                return;
            }
            m_slaveAddress = address;
            Byte addr = (address << 1) | 0x01; // LSB = 1 for read
            m_board->m_gpio.sda.write(addr);
            m_state = State::SendAddress;
            m_transactionActive = true;
            resetTransaction();

        }

        Byte
        read() override{
            if (m_buffer.empty()) {
                std::cerr << "I2C: No data to read." << std::endl;
                return 0x00;
            }
            Byte v = m_buffer.front();
            m_buffer.pop();
            return v;
        }

        bool isIdle() const {


            return m_state == State::Idle || m_state == State::Done;
        }

        void
        run(Gpio& gpio) override{
            if (!m_transactionActive) {
                return;
            }
            if (m_state != State::Idle && m_state != State::Done) {
                m_timeoutCounter++;
                if (m_timeoutCounter > TIMEOUT_CYCLES) {
                    std::cerr << "I2C: Transaction timeout." << std::endl;
                    m_state = State::Error;
                    resetTransaction();
                    return;
                }
            }
            switch (m_state)
            {
            case State::SendAddress:{
                if (m_board->m_gpio.sda.hasBitToWrite() > 0) {
                    return;     
                }
                m_state = State::WaitAck;
                m_timeoutCounter = 0;
                break;
            }
            case State::WaitAck:{
                if (m_board->m_gpio.sda.hasBitToRead()) {
                    return;
                }
                Bit ack = m_board->m_gpio.sda.readBit();
                if (ack == Bit::Zero) {
                    std::cerr << "I2C: ACK received for address." << std::endl;
                    m_state = State::ReadByte;
                    m_timeoutCounter = 0;
                    return;
                } else {
                    std::cerr << "I2C: NACK received for address." << std::endl;
                    m_state = State::Error;
                    resetTransaction();
                }
                break;
            }
            case State::ReadByte:{
                if (!m_board->m_gpio.sda.hasBitToRead()) {
                    return;
                }
                Bit b = m_board->m_gpio.sda.readBit();
                m_currentByte = (m_currentByte << 1) | (b == Bit::One ? 1 : 0);
                m_bitCounter++;
                if (m_bitCounter == 8) {
                    storeByte(m_currentByte);
                    m_bitCounter = 0;
                    m_currentByte = 0x00;
                    m_byteCounter++;
                    if (m_byteCounter < PAKET_SIZE) {
                        m_state = State::SendAck;
                    } else {
                        m_state = State::SendNack;
                    }
                }
                break;
            }
            case State::SendAck:{
                m_board->m_gpio.sda.write(Bit::Zero);
                m_state = State::ReadByte;
                m_timeoutCounter = 0;
                break;
            }
            case State::SendNack:{
                m_board->m_gpio.sda.write(Bit::One);
                m_state = State::Validate;
                m_timeoutCounter = 0;
                break;
            }
            case State::Validate:{
                Byte maxv = (m_b1 > m_b2) ? m_b1 : m_b2;
                Byte minv = (m_b1 > m_b2) ? m_b2 : m_b1;
                Byte calculatedChecksum = maxv - minv;
                if (calculatedChecksum == m_checksum) {
                    std::cout << "I2C: Packet received successfully. Data: " 
                              << std::hex << static_cast<int>(m_b1) << " " 
                              << std::hex << static_cast<int>(m_b2) << std::dec 
                              << ", Checksum: " << std::hex << static_cast<int>(m_checksum) 
                              << std::dec << std::endl;
                    m_buffer.push(m_b1);
                    m_buffer.push(m_b2);
                } else {
                    std::cerr << "I2C: Checksum mismatch. Received: " 
                              << std::hex << static_cast<int>(m_checksum) 
                              << ", Calculated: " << std::hex << static_cast<int>(calculatedChecksum) 
                              << std::dec << std::endl;
                    m_state = State::Error;
                }
                resetTransaction();
                break;
            }
            default:
                break;
            }

        }

    private: 
        void resetTransaction() {
            m_timeoutCounter = 0;
            m_bitCounter = 0;
            m_byteCounter = 0;
            m_currentByte = 0x00;
        }

        void storeByte(Byte byte) {
            if (m_byteCounter == 0) {
                m_b1 = byte;
            } else if (m_byteCounter == 1) {
                m_b2 = byte;
            } else {
                m_checksum = byte;
            }


        }

    } mutable i2c {this};

    class USART : public Protocols::AbstractUsart<Esp8266, Gpio>
    {
    public:
        explicit USART(Esp8266* b) :
            Protocols::AbstractUsart<Esp8266, Gpio> {b}
        {}

        void
        write(Byte byte) override
        {}

        Byte
        read() override
        {
            return 0;
        }

        void
        run(Gpio& gpio) override
        {}

    } mutable usart {this};

protected:
    inline void
    startModule() override
    {}
};
}    // namespace Boards

#endif    // ESP8266_H
