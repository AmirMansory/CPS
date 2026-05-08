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
        m_processor->installProtocol(&usart);      
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

            while(m_board->m_gpio.sda.hasBitToRead())
                    m_board->m_gpio.sda.read();
            
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
                        m_state = State::SendNack;
                    } else {
                        printf("[MASTER] ❌ Checksum mismatch! calc=0x%02X, recv=0x%02X\n", calc_cs, u_cs);
                        m_state = State::SendNack;
                    }                    
                }
                break;
            }

            case State::SendNack: {
                if (gpio.sda.hasBitToWrite()) break;
                gpio.sda.write(Bit::One); // ارسال سیگنال پایان تراکنش (NACK)
                m_state = State::Done;
                break;
            }


            case State::Error:
                printf("[MASTER] ⚠️ Entered Error state – resetting transaction\n");
                m_state = State::Done;
                break;

            case State::Done:             
                if(gpio.sda.hasBitToWrite()) return;
                while(gpio.sda.hasBitToRead())
                    gpio.sda.read();
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

    class USART : public Protocols::AbstractUsart<Esp8266, Gpio>
    {
    public:
        explicit USART(Esp8266* b) : Protocols::AbstractUsart<Esp8266, Gpio>{b} {}

        void write(Byte) override {}
        Byte read() override { return 0; }

        void request(Byte address)
        {
            if (m_txState != TxIdle || m_requestPending) return;
            m_addressToSend = address;
            m_requestPending = true;
            m_txState = TxStart;
            std::cout << "[MASTER] ➡️ Request addr 0x"
                    << std::hex << static_cast<int>(static_cast<uint8_t>(address)) << std::dec << std::endl;
        }

        bool hasRequestLock() const { return m_requestPending; }
        bool hasDataArrived() const { return m_dataReady; }
        Byte getReceivedByte() { m_dataReady = false; return m_receivedData; }

        void run(Gpio& gpio) override
        {
            // ---- TRANSMITTER ----
            switch (m_txState)
            {
            case TxIdle: break;
            case TxStart:
                gpio.tx.write(Bit::Zero);        
                m_txState = TxData;
                break;
            case TxData:
                if (!gpio.tx.hasBitToWrite()){
                    gpio.tx.write(m_addressToSend); 
                    m_txState = TxStop;
                }
                break;
            case TxStop:
                if (!gpio.tx.hasBitToWrite()){
                    gpio.tx.write(Bit::One);   
                    if (m_txStopSent){
                        m_txStopSent = false;
                        m_requestPending = false;
                        m_txState = TxIdle;
                    }
                    else{
                        m_txStopSent = true;
                    }
                }
                break;
            }

            // ---- RECEIVER ----
            switch (m_rxState)
            {
            case RxIdle:
                if (gpio.rx.hasBitToRead()){
                    Bit b = gpio.rx.readBit();
                    if (b == Bit::Zero)            
                        m_rxState = RxData;
                }
                break;
            case RxData:
                if (gpio.rx.hasByteToRead()){
                    m_rxByte = gpio.rx.read();
                    m_rxState = RxStop;
                }
                break;
            case RxStop:
                if (gpio.rx.hasBitToRead()){
                    Bit stop = gpio.rx.readBit();
                    if (stop == Bit::One){
                        m_receivedData = m_rxByte;
                        m_dataReady = true;
                        std::cout << "[MASTER] ✅ Received data 0x"
                                << std::hex << static_cast<int>(static_cast<uint8_t>(m_rxByte)) << std::dec << std::endl;
                    }
                    else{
                        std::cout << "[MASTER] ❌ Bad stop bit" << std::endl;
                    }
                    m_rxState = RxIdle;
                }
                break;
            }
        }

    private:
        enum TxState { TxIdle, TxStart, TxData, TxStop };
        enum RxState { RxIdle, RxData, RxStop };

        TxState m_txState = TxIdle;
        RxState m_rxState = RxIdle;

        Byte    m_addressToSend = 0;
        bool    m_requestPending = false;
        bool    m_txStopSent = false;    

        Byte    m_rxByte = 0;
        Byte    m_receivedData = 0;
        bool    m_dataReady = false;
    }mutable usart{this};


protected:
    void startModule() override {        
        
        m_gpio.scl.onNextEdge([this](Esp8266Voltage level) {
            auto bit = Voltage::toBit(level);
            if (bit == Bit::One) m_processor->nextCycle(m_gpio);
        });
    }
};
} // namespace Boards

#endif
