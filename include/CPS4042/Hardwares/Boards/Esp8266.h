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

        void init(Byte address) override {
            if (m_state != State::WaitAck) return;
            std::cout << "Microcontroller ready  --> procced" << std::endl;
            write(address);
        }

        void write(Byte address) override{
            if (m_board->m_gpio.sda.hasBitToWrite()) return;
            std::cout << ">>> [MASTER] Start -> addr 0x"
                      << std::hex << (int)(uint8_t)address << std::dec << std::endl;
            m_slaveAddress = address;
            Byte addr = (address << 1) | 0x01;
            m_board->m_gpio.sda.write(addr);
            m_transactionActive = true;
            resetTransaction();
        }

        Byte read() override{
            if (m_buffer.empty()) return 0;
            Byte v = m_buffer.front(); m_buffer.pop();
            return v;
        }

        bool isIdle() const { return (m_state == State::Done);}

        void run(Gpio& gpio) override
        {
            if (!m_transactionActive) return;
            if (m_state != State::Done){
                if (++m_timeoutCounter > TIMEOUT_CYCLES) {
                    std::cout << "[MASTER] Timeout" << std::endl;
                    m_state = State::Error;
                }
            }

            switch (m_state) {


            case State::WaitAck: {
                Bit ack = gpio.sda.readBit();
                if (ack == Bit::One) {
                    printf("[MASTER] ✅ ACK received from slave\n");
                    m_state = State::ReadByte;
                    m_timeoutCounter = 0;
                } else {
                    printf("[MASTER] ⏳ waiting ...\n");

                }
                break;
            }

            // ---- read three data bytes (b1, b2, checksum) ----
            case State::ReadByte: {
                ++m_byteCounter;
                if (m_byteCounter==1){
                    m_b1 = gpio.sda.read();
                }else if(m_byteCounter==2){
                    m_b2 = gpio.sda.read();
                }else if(m_byteCounter==3){
                    m_checksum = gpio.sda.read();
                }else{
                    printf("[MASTER] 📥 Received bytes: b1=0x%02X (%d), b2=0x%02X (%d), cs=0x%02X (%d)\n",
                        m_b1, m_b1, m_b2, m_b2, m_checksum, m_checksum);
                    uint8_t calc_cs = abs(m_b1 - m_b2);
                    if (calc_cs == abs(m_checksum)) {
                        printf("[MASTER] ✅ Quick checksum OK (0x%02X == 0x%02X)\n", calc_cs, m_checksum);
                        m_state = State::Done;
                    } else {
                        printf("[MASTER] ❌ Quick checksum mismatch! calc=0x%02X, recv=0x%02X\n", calc_cs, m_checksum);
                        m_state = State::Error;  // as per original
                    }                    
                }
                break;
            }

            // ---- error state ----
            case State::Error:
                printf("[MASTER] ⚠️ Entered Error state – resetting transaction\n");
                m_state = State::Done;
                break;

            // ---- done state: flush any remaining bits ----
            case State::Done:
                printf("[MASTER] 🏁 Transaction finished – flushing leftover bits\n");              
                m_transactionActive = false;
                m_state = State::WaitAck;
                break;

            default: break;
            }
        }

    private:
        enum class State : uint8_t {
            WaitAck, ReadByte,
            SendAck, SendNack, Done, Error
        };
        State m_state = State::WaitAck;
        Byte m_slaveAddress = 0x00; 
        static constexpr uint8_t PACKET_SIZE = 3;
        static constexpr uint32_t TIMEOUT_CYCLES = 2000;
        uint32_t m_timeoutCounter = 0;
        uint8_t m_byteCounter = 0;
        Byte m_b1 = 0, m_b2 = 0, m_checksum = 0;
        bool m_transactionActive = false;

        void resetTransaction() {
            m_timeoutCounter = 0;
            m_b1 = 0, m_b2 = 0, m_checksum = 0;
            m_byteCounter = 0;
        }


    } mutable i2c{this};

   class USART : public Protocols::AbstractUsart<Esp8266, Gpio>
    {
    public:
        explicit USART(Esp8266* b) : Protocols::AbstractUsart<Esp8266, Gpio>{b} {}

        void request(Byte addr) {
            if (m_state != State::IDLE) return;
            std::cout << "request function in master address : " << (int)addr << std::endl;
            m_txByte = addr;
            request_lock = {true};
            m_state = State::SEND_START;
          
        }

        bool hasRequestLock() const { return request_lock; }
        bool hasDataArrived() const {return data_arrived;}
        Byte getReceivedByte() const { return m_rxByte; }


        void run(Gpio& gpio) override {
            // ----- Transmitter -----
            switch (m_state) {
                case State::IDLE:
                    request_lock = {false};
                    break;

                case State::SEND_START:
                    if (!gpio.tx.hasBitToWrite() && !gpio.tx.hasByteToWrite()) {
                        gpio.tx.write(Bit::Zero);
                        std::cout << "send start state in master" << std::endl;
                        m_state = State::SEND_DATA;
                    }
                    break;

                case State::SEND_DATA:
                    if (!gpio.tx.hasBitToWrite() && !gpio.tx.hasByteToWrite()) {
                        gpio.tx.write(m_txByte);  
                        std::cout << "send data state in master address : " << (int)m_txByte << std::endl;
                        m_state = State::SEND_STOP;
                    }
                    break;

                case State::SEND_STOP:
                    if (!gpio.tx.hasBitToWrite() && !gpio.tx.hasByteToWrite()) {
                        gpio.tx.write(Bit::One);
                        std::cout << "send stop state in master address : " << std::endl;
                        m_state = State::GET_DATA;      // now wait for response
                    }
                    break;

                case State::GET_DATA:{
                    if (gpio.rx.hasByteToRead()){
                        m_rxByte = gpio.rx.read();
                        data_arrived = {true};
                        m_state = State::IDLE;
                    }
                    break;
                }

            }
        }

        void write(Byte) override {}
        Byte read() override { return 0; }

    private:
        enum class State { IDLE, SEND_START, SEND_DATA, SEND_STOP, GET_DATA};
        State m_state = State::IDLE;
        Byte  m_txByte = 0;
        Byte  m_rxByte = 0;
        bool request_lock = {false};
        bool data_arrived = {false};
    } mutable usart{this};

protected:
    void startModule() override {}
};

} // namespace Boards


#endif

