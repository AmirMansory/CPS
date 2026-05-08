#ifndef I2CMUX_H
#define I2CMUX_H

#include <CPS4042/Hardwares/Board.h>
#include <CPS4042/Protocols/Protocol.h>
#include <CPS4042/Units/BaudRate.h>
#include <CPS4042/Wires/Pin.h>
#include <boost/pfr.hpp>
#include <iostream>

namespace Hardware
{
using MuxVoltage = VoltageLevel3_3v;

template <std::uint64_t bar, std::uint64_t btr, typename WorkingVoltageTp>
requires std::is_base_of_v<AbstractVoltageLevel, WorkingVoltageTp>
struct I2CMuxGpio
{
public:
    Pins::Vdd<WorkingVoltageTp> vdd   {bar, btr, "Mux::vdd"};
    Pins::Gnd<WorkingVoltageTp> gnd   {bar, btr, "Mux::gnd"};
    Pins::Rx<WorkingVoltageTp>  rx    {bar, btr, "Mux::rx"};   
    Pins::Tx<WorkingVoltageTp>  tx    {bar, btr, "Mux::tx"};  
    Pins::Sda<WorkingVoltageTp> sda0 {bar, btr, "Mux::sda0"};
    Pins::Scl<WorkingVoltageTp> scl0 {bar, btr, "Mux::scl0"};
    Pins::Sda<WorkingVoltageTp> sda1 {bar, btr, "Mux::sda1"};
    Pins::Scl<WorkingVoltageTp> scl1 {bar, btr, "Mux::scl1"};
    Pins::Sda<WorkingVoltageTp> sda2 {bar, btr, "Mux::sda2"};
    Pins::Scl<WorkingVoltageTp> scl2 {bar, btr, "Mux::scl2"};
    Pins::Sda<WorkingVoltageTp> sda3 {bar, btr, "Mux::sda3"};
    Pins::Scl<WorkingVoltageTp> scl3 {bar, btr, "Mux::scl3"};
};

class I2CMux : public Board<BaudRates::NotSpecified,
                            BitRates::same(BaudRates::NotSpecified),
                            Frequency::F320khz, MuxVoltage, I2CMuxGpio>
{
public:
    class USART;
    class I2CChannel;

    explicit I2CMux() : Parent {"I2CMux::Processor"}{
        m_processor->communicationClockChanged.connect(
          [this](Bit edge) {
              m_gpio.scl0.nextEdge(edge);
              m_gpio.scl1.nextEdge(edge);
              m_gpio.scl2.nextEdge(edge);
              m_gpio.scl3.nextEdge(edge);
          });

        m_processor->installProtocol(&usart);
        for (auto& ch : m_i2cChannels)
            m_processor->installProtocol(&ch);

        std::cout << "[MUX] Board created, 4 I2C channels ready." << std::endl;
    }

    class USART : public Protocols::AbstractUsart<I2CMux, Gpio>{
    public:
        explicit USART(I2CMux* b) : Protocols::AbstractUsart<I2CMux, Gpio>{b} {}

        void write(Byte) override {}
        Byte read() override { return 0; }

        void run(Gpio& gpio) override{
            switch (m_rxState){
            case RxIdle:
                if (gpio.tx.hasBitToRead()) {
                    Bit b = gpio.tx.readBit();
                    if (b == Bit::Zero)            
                        m_rxState = RxData;
                }
                break;
            case RxData:
                if (gpio.tx.hasByteToRead()) {
                    m_rxByte = gpio.tx.read();     
                    m_rxState = RxStop;
                }
                break;
            case RxStop:
                if (gpio.tx.hasBitToRead()) {
                    Bit stop = gpio.tx.readBit();
                    if (stop == Bit::One) {
                        uint8_t channel = static_cast<uint8_t>(m_rxByte);
                        if (channel < 4) {
                            std::cout << "[MUX] ➡️ Channel " << (int)channel << " selected" << std::endl;
                            m_board->m_i2cChannels[channel].startTransaction();
                        }
                    }
                    m_rxState = RxIdle;
                }
                break;
            }

            if (m_txPending){
                switch (m_txState)
                {
                case TxIdle:
                    m_txByteIdx = 0;
                    m_txState = TxStart;
                    break;
                case TxStart:
                    gpio.rx.write(Bit::Zero);          
                    m_txState = TxData;
                    break;
                case TxData:
                    if (!gpio.rx.hasBitToWrite()) {     
                        gpio.rx.write(m_txBuffer[m_txByteIdx]); 
                        m_txState = TxStop;
                    }
                    break;
                case TxStop:
                    if (!gpio.rx.hasBitToWrite()) {     
                        gpio.rx.write(Bit::One);        
                        if (m_txStopSent) {
                            m_txStopSent = false;
                            m_txByteIdx++;
                            if (m_txByteIdx >= m_txByteCount) {
                                m_txPending = false;
                                m_txState = TxIdle;
                                std::cout << "[MUX] ✅ Data forwarded" << std::endl;
                            } else {
                                m_txState = TxStart;    
                            }
                        } else {
                            m_txStopSent = true;
                        }
                    }
                    break;
                }
            }
        }

        void setResponseBytes(Byte b) {
            m_txBuffer[0] = b;
            m_txByteCount = 1;
            m_txPending = true;
        }

    private:
        enum RxState { RxIdle, RxData, RxStop };
        enum TxState { TxIdle, TxStart, TxData, TxStop };

        RxState m_rxState = RxIdle;
        TxState m_txState = TxIdle;
        Byte    m_rxByte = 0;

        bool    m_txPending = false;
        Byte    m_txBuffer[3] = {};     
        int     m_txByteIdx = 0;
        int     m_txByteCount = 1;
        bool    m_txStopSent = false;
    } mutable usart{this};

    class I2CChannel : public Protocols::AbstractI2C<I2CMux, Gpio>
    {
    public:
        I2CChannel(I2CMux* b, int channelIndex)
            : Protocols::AbstractI2C<I2CMux, Gpio>{b}, m_channelIndex(channelIndex) {}

        void init(Byte) override {}
        void write(Byte) override {}
        Byte read() override { return 0; }

        void startTransaction() {
            if (m_state != State::Idle) return;
            m_state = State::SendAddress;
            m_bitCount = 0;
        }

        void run(Gpio& gpio) override
        {
            if (m_state == State::Idle) return;

            Pins::Sda<MuxVoltage>& sda = getSda(gpio, m_channelIndex);

            switch (m_state)
            {
            case State::Idle: break;

            case State::SendAddress:
                if (!sda.hasBitToWrite()) {
                    Byte addr = (0x29 << 1) | 0x01;
                    sda.write(addr);
                    m_state = State::WaitAck;
                }
                break;

            case State::WaitAck:
                if (sda.hasBitToRead()) {
                    Bit ack = sda.readBit();
                    if (ack == Bit::One) {
                        m_state = State::ReadData;
                    } else {
                        m_state = State::Idle;  
                    }
                }
                break;

            case State::ReadData:
                if (sda.hasByteToRead()) {
                    m_dataByte = sda.read();         
                    m_state = State::SendNack;
                }
                break;

            case State::SendNack:
                if (!sda.hasBitToWrite()) {
                    sda.write(Bit::One);             
                    m_state = State::Idle;
                    m_board->usart.setResponseBytes(m_dataByte);
                }
                break;
            }
        }

    private:
        Pins::Sda<MuxVoltage>& getSda(Gpio& gpio, int channel) {
            switch (channel) {
                case 0: return gpio.sda0;
                case 1: return gpio.sda1;
                case 2: return gpio.sda2;
                default: return gpio.sda3;
            }
        }

        enum State { Idle, SendAddress, WaitAck, ReadData, SendNack };
        State m_state = State::Idle;
        int   m_channelIndex = 0;
        int   m_bitCount = 0;
        Byte  m_dataByte = 0;
    };

    I2CChannel m_i2cChannels[4] = {
        I2CChannel(this, 0),
        I2CChannel(this, 1),
        I2CChannel(this, 2),
        I2CChannel(this, 3)
    };

protected:
    void startModule() override {}
};

} // namespace Hardware

#endif