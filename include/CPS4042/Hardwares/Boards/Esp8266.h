#ifndef ESP8266_H
#define ESP8266_H

#include <CPS4042/Hardwares/Board.h>
#include <CPS4042/Protocols/Protocol.h>
#include <CPS4042/Units/BaudRate.h>
#include <CPS4042/Units/Byte.h>
#include <CPS4042/Wires/Pin.h>
#include <boost/pfr.hpp>
#include <iostream>
#include <bitset>
#include <queue>
#include <cmath>

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
    }

    class I2C : public Protocols::AbstractI2C<Esp8266, Gpio>
    {
    public:
        explicit I2C(Esp8266* b) : Protocols::AbstractI2C<Esp8266, Gpio>{b} {}

        void init(Byte address) override {
            if (m_state != State::WaitAck && m_state != State::Done) return;
            std::cout << "[MASTER] Microcontroller ready --> proceed" << std::endl;
            write(address);
        }

        void write(Byte address) override{
            if (m_board->m_gpio.sda.hasBitToWrite()) return;
            
            uint8_t u_addr = static_cast<uint8_t>(address);
            std::cout << ">>> [MASTER] Start -> addr 0x" << std::hex << (int)u_addr << std::dec << std::endl;
            
            m_slaveAddress = address;
            Byte addr = (address << 1) | 0x01;
            m_board->m_gpio.sda.write(addr);
            
            m_transactionActive = true;
            resetTransaction();
            m_state = State::WaitAck;
        }

        Byte read() override{
            if (m_buffer.empty()) return 0;
            Byte v = m_buffer.front(); 
            m_buffer.pop();
            return v;
        }

        bool isIdle() const { return (m_state == State::Done); }
        bool isDataAvailable() const { return !m_buffer.empty(); }

        void run(Gpio& gpio) override
        {
            if (!m_transactionActive) return;
            if (m_state != State::Done){
                if (++m_timeoutCounter > TIMEOUT_CYCLES) {
                    std::cout << "[MASTER] ❌ Timeout" << std::endl;
                    m_state = State::Error;
                }
            }

            switch (m_state) {

            case State::WaitAck: {
                if (!gpio.sda.hasBitToRead()) break;
                Bit ack = gpio.sda.readBit();
                if (ack == Bit::One) {
                    printf("[MASTER] ✅ ACK received from slave\n");
                    m_state = State::ReadByte;
                    m_timeoutCounter = 0;
                }
                break;
            }

            case State::ReadByte: {
                if (!gpio.sda.hasByteToRead()) break;

                if (m_byteCounter == 0){
                    m_b1 = gpio.sda.read();
                    m_byteCounter++;
                } else if(m_byteCounter == 1){
                    m_b2 = gpio.sda.read();
                    m_byteCounter++;
                } else if(m_byteCounter == 2){
                    m_checksum = gpio.sda.read();
                    m_byteCounter++;

                    uint8_t u_b1 = static_cast<uint8_t>(m_b1);
                    uint8_t u_b2 = static_cast<uint8_t>(m_b2);
                    uint8_t u_cs = static_cast<uint8_t>(m_checksum);

                    printf("[MASTER] 📥 Received bytes: b1=0x%02X, b2=0x%02X, cs=0x%02X\n", u_b1, u_b2, u_cs);
                    
                    // فرمول ریاضی: $cs = |b_1 - b_2|$
                    uint8_t calc_cs = static_cast<uint8_t>(std::abs(static_cast<int>(u_b1) - static_cast<int>(u_b2)));
                    
                    if (calc_cs == u_cs) {
                        printf("[MASTER] ✅ Checksum OK (0x%02X)\n", calc_cs);
                        m_buffer.push(m_b1);
                        m_buffer.push(m_b2);
                        m_state = State::Done;
                    } else {
                        printf("[MASTER] ❌ Checksum mismatch! calc=0x%02X, recv=0x%02X\n", calc_cs, u_cs);
                        m_state = State::Error;
                    }                    
                }
                break;
            }

            case State::Error:
                printf("[MASTER] ⚠️ Entered Error state – resetting transaction\n");
                m_transactionActive = false;
                m_state = State::Done;
                break;

            case State::Done:             
                m_transactionActive = false;
                break;

            default: break;
            }
        }

    private:
        enum class State : uint8_t {
            WaitAck, ReadByte, SendAck, SendNack, Done, Error
        };
        State m_state = State::Done;
        Byte m_slaveAddress = 0x00; 
        static constexpr uint32_t TIMEOUT_CYCLES = 2000;
        uint32_t m_timeoutCounter = 0;
        uint8_t m_byteCounter = 0;
        Byte m_b1 = 0, m_b2 = 0, m_checksum = 0;
        bool m_transactionActive = false;
        std::queue<Byte> m_buffer;

        void resetTransaction() {
            m_timeoutCounter = 0;
            m_b1 = 0; m_b2 = 0; m_checksum = 0;
            m_byteCounter = 0;
        }
    } mutable i2c{this};

    // ... (بخش USART بدون تغییر)
protected:
    void startModule() override {}
};
} // namespace Boards
#endif
