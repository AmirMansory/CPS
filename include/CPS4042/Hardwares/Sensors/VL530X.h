#ifndef VL53_X_H
#define VL53_X_H

#include <CPS4042/Hardwares/Board.h>
#include <CPS4042/Units/BaudRate.h>
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
    Pins::Vdd<WorkingVoltageTp> vdd {bar, btr, "Vl530x::vdd"};    // Pin 0
    Pins::Gnd<WorkingVoltageTp> gnd {bar, btr, "Vl530x::gnd"};    // Pin 1
    Pins::Sda<WorkingVoltageTp> sda {bar, btr, "Vl530x::sda"};    // Pin 2
    Pins::Scl<WorkingVoltageTp> scl {bar, btr, "Vl530x::scl"};    // Pin 3
};

class Vl530x : public Board<BaudRates::NotSpecified,
                            BitRates::same(BaudRates::NotSpecified),
                            Frequency::Drived, Vl530xVoltage, Vl530xGpio>
{
public:
    inline static constexpr Byte address = 0x29;

    explicit Vl530x() :
        Parent {"Vl530x::processor"}
    {
        m_processor->installProtocol(&i2c);

        std::cout << "one instance of Vl530x" << " created." << std::endl;
    }

    class I2C : public Protocols::AbstractI2C<Vl530x, Gpio>
    {
    private:
        enum class State {
            Idle,
            WaitAddress,
            SendAck,
            SendByte,
            WaitMasterAck
        };

        State m_state = State::WaitAddress;
        static constexpr Byte DEVICE_ADDRESS = 0x29;
        Byte m_b1 = 0, m_b2 = 0;
        Byte m_checksum = 0;
        uint8_t m_bitCounter = 0;
        uint8_t m_byteIndex = 0;


    public:
        explicit I2C(Vl530x* b) :
            Protocols::AbstractI2C<Vl530x, Gpio> {b}
        {}

        void
        init(Byte) override{
            m_state = State::WaitAddress;
            m_b1 = 0;
            m_b2 = 0;
            m_checksum = 0;
            m_bitCounter = 0;
            m_byteIndex = 0;
        }

        void
        write(Byte byte) override
        {}

        Byte
        read() override
        {
            return 0;
        }

        void setMeasurementData(uint16_t value){
            m_b1 = (value >> 8) & 0xFF;
            m_b2 = value & 0xFF;
            Byte maxv = (m_b1 > m_b2) ? m_b1 : m_b2;
            Byte minv = (m_b1 < m_b2) ? m_b1 : m_b2;
            m_checksum = maxv - minv;
        }

        bool isReady() const{
            return m_state == State::WaitAddress;
        }

        void
        run(Gpio& gpio) override{
            switch(m_state)
            {
                case State::WaitAddress:{
                    if (!gpio.sda.hasByteToRead()) return;
                    Byte raw = gpio.sda.read();
                    Byte receivedAddr = (raw >> 1) & 0x7F;
                    bool isRead = (raw & 0x1) == 1;
                    if (receivedAddr == DEVICE_ADDRESS && isRead){
                        m_state = State::SendAck;
                    }
                    break;
                }
                case State::SendAck:{
                    if (!gpio.sda.hasBitToWrite()){
                        gpio.sda.write(Bit::Zero);
                        m_byteIndex = 0;
                        m_bitCounter = 0;
                        m_state = State::SendByte;
                    }
                    break;
                }
                case State::SendByte:{
                    Byte currentByte;
                    if (m_byteIndex == 0) currentByte = m_b1;
                    else if (m_byteIndex == 1) currentByte = m_b2;
                    else currentByte = m_checksum;

                    Bit bit = (((currentByte >> (7 - m_bitCounter)) & 1) ? Bit::One : Bit::Zero);
                    gpio.sda.write(bit);

                    m_bitCounter++;
                    if (m_bitCounter == 8){
                        gpio.sda.write(Bit::Z);
                        m_bitCounter = 0;
                        m_byteIndex++;
                        m_state = State::WaitMasterAck;
                    }
                    break;
                }
                case State::WaitMasterAck:{
                    if (!gpio.sda.hasBitToRead()) return;
                    Bit ack = gpio.sda.readBit();
                    if (ack == Bit::Zero){
                        if (m_byteIndex < 3){
                            m_state = State::SendByte;
                        }
                    }else{
                        m_state = State::WaitAddress;
                    }
                    break;
                }
                default:
                    break;
            }
        }

    } mutable i2c {this};

protected:
    inline void
    startModule() override
    {
        m_gpio.scl.onNextEdge([this](Vl530xVoltage level) {
            auto bit = Voltage::toBit(level);

            if(bit == Bit::One)    // positive edge
            {
                m_processor->nextCycle(m_gpio);
            }
        });
    }
};

}    // namespace Sensors

#endif    // VL53_X_H
