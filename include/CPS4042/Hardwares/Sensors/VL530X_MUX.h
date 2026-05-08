#ifndef VL53_X_MUX_H
#define VL53_X_MUX_H

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
        std::cout << "[VL530X MUX] created." << std::endl;
    }
   
    class I2C : public Protocols::AbstractI2C<Vl530x, Gpio>
    {
    public:
        explicit I2C(Vl530x* b) : Protocols::AbstractI2C<Vl530x, Gpio>{b} {}

        void init(Byte address) override { m_state = State::WaitAddress; }
        void write(Byte) override {}
        Byte read() override { return 0; }

        // Set a single byte to be sent when master reads
        void setData(uint8_t value)
        {
            m_dataByte = static_cast<Byte>(value);
        }

        bool isReady() const { return m_state == State::WaitAddress; }

        void run(Gpio& gpio) override
        {
            switch (m_state) {

            case State::Idle:
                raw = 0;
                receivedAddr = 0;
                isRead = false;
                while (gpio.sda.hasBitToRead()) gpio.sda.readBit();
                m_state = State::WaitAddress;
                break;

            case State::WaitAddress: {
                if (!gpio.sda.hasByteToRead()) return;
                raw = gpio.sda.read();
                
                uint8_t u_raw = static_cast<uint8_t>(raw);
                receivedAddr = (u_raw >> 1) & 0x7F;
                isRead = (u_raw & 1) == 1;
                
                if (receivedAddr == DEVICE_ADDRESS && isRead) {
                    m_state = State::SendAck;
                } else {
                    m_state = State::Idle;
                }
                break;
            }

            case State::SendAck: {
                gpio.sda.write(Bit::One);       
                m_state = State::SendData;
                break;
            }

            case State::SendData: {
                if (!gpio.sda.hasBitToWrite()) {   
                    gpio.sda.write(m_dataByte);   
                    m_state = State::WaitNack;
                }
                break;
            }

            case State::WaitNack: {
                if (!gpio.sda.hasBitToRead()) return;
                Bit nack = gpio.sda.readBit();
                if (nack == Bit::One) {
                    while (gpio.sda.hasBitToRead()) gpio.sda.readBit();
                    m_state = State::Idle;
                }
                break;
            }

            default: break;
            }
        }

    private:
        enum class State : uint8_t { Idle, WaitAddress, SendAck, SendData, WaitNack };
        State m_state = State::WaitAddress;
        static constexpr uint8_t DEVICE_ADDRESS = 0x29;

        Byte m_dataByte = 0;          
        Byte raw = 0; 
        uint8_t receivedAddr = 0;
        bool isRead = false;
    } mutable i2c{this};

protected:
    void startModule() override{
        m_gpio.scl.onNextEdge([this](Vl530xVoltage level) {
            auto bit = Voltage::toBit(level);
            if (bit == Bit::One) m_processor->nextCycle(m_gpio);   
        });
    }
};

} // namespace Sensors

#endif // VL53_X_MUX_H