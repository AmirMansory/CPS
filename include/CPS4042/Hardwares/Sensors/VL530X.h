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
    // ==================== بخش اضافه شده ====================
    private:
        Byte m_myAddress;           // آدرس خود سنسور (0x29)
        Byte m_txBuffer[3];         // بافر داده‌ای که باید فرستاده شود (2 بایت داده + 1 بایت checksum)
        int m_txIndex;              // اندیس فعلی در بافر ارسال
        int m_bytesToSend;          // تعداد بایت‌هایی که باید فرستاده شوند (معمولاً 3)
        bool m_addressed;           // آیا آدرس سنسور توسط مستر دیده شده؟
        Byte m_receivedAddress;     // آدرس دریافتی از مستر
        
        enum State { IDLE, ADDRESS_RECEIVED, SEND_DATA, SEND_ACK } m_state;
    // ====================================================

    public:
        explicit I2C(Vl530x* b) :
            Protocols::AbstractI2C<Vl530x, Gpio> {b},
            // ==================== بخش اضافه شده ====================
            m_myAddress(Vl530x::address),
            m_txIndex(0),
            m_bytesToSend(0),
            m_addressed(false),
            m_receivedAddress(0),
            m_state(IDLE)
            // ====================================================
        {}

        void
        init(Byte address) override
        // ==================== بخش اضافه شده ====================
        {
            m_myAddress = address;
            std::cout << "[VL530X I2C] Initialized with own address: 0x" 
                      << std::hex << (int)address << std::dec << std::endl;
        }
        // ====================================================

        void
        write(Byte byte) override
        // ==================== بخش اضافه شده ====================
        {
            // سنسور داده از مستر دریافت می‌کند (معمولاً آدرس یا فرمان)
            std::cout << "[VL530X I2C] Received byte from master: 0x" 
                      << std::hex << (int)byte << std::dec << std::endl;
            
            if (!m_addressed) {
                m_receivedAddress = byte;
                // چک می‌کنیم آیا آدرس دریافتی با آدرس خودمان یکی است؟
                if (byte == m_myAddress) {
                    m_addressed = true;
                    m_state = ADDRESS_RECEIVED;
                    std::cout << "[VL530X I2C] ✓ My address matched! (0x" 
                              << std::hex << (int)byte << std::dec << ")" << std::endl;
                }
            }
        }
        // ====================================================

        Byte
        read() override
        // ==================== بخش اضافه شده ====================
        {
            // مستر از سنسور می‌خواند - داده آماده شده را برمی‌گردانیم
            if (m_state == SEND_DATA && m_txIndex < m_bytesToSend) {
                Byte data = m_txBuffer[m_txIndex];
                std::cout << "[VL530X I2C] Sending byte " << m_txIndex + 1 
                          << ": " << (int)data << std::endl;
                m_txIndex++;
                return data;
            }
            return 0;
        }
        // ====================================================

        void
        run(Gpio& gpio) override
        // ==================== بخش اضافه شده ====================
        {
            switch (m_state) {
                case ADDRESS_RECEIVED:
                    // آدرس خود را دیدیم، حالا باید ACK بفرستیم و آماده ارسال داده شویم
                    std::cout << "[VL530X I2C] Address matched, preparing to send data..." << std::endl;
                    m_state = SEND_ACK;
                    break;
                    
                case SEND_ACK:
                    // ACK ارسال شد (در این شبیه‌ساز، فقط وضعیت را ثبت می‌کنیم)
                    std::cout << "[VL530X I2C] Sending ACK to master" << std::endl;
                    m_state = SEND_DATA;
                    m_txIndex = 0;
                    break;
                    
                case SEND_DATA:
                    // در حال ارسال داده به مستر هستیم
                    if (m_txIndex >= m_bytesToSend) {
                        // همه داده‌ها فرستاده شد
                        std::cout << "[VL530X I2C] All data sent, returning to IDLE" << std::endl;
                        m_state = IDLE;
                        m_addressed = false;
                        m_bytesToSend = 0;
                    }
                    break;
                    
                default:
                    break;
            }
        }
        // ====================================================

        // ==================== توابع کمکی اضافه شده ====================
        // تابع برای تنظیم داده‌ای که باید به مستر فرستاده شود
        void setDataToSend(const Byte data[3]) {
            m_txBuffer[0] = data[0];
            m_txBuffer[1] = data[1];
            m_txBuffer[2] = data[2];
            m_bytesToSend = 3;
            m_txIndex = 0;
            std::cout << "[VL530X I2C] Data prepared for transmission" << std::endl;
        }
        
        // تابع برای تنظیم داده با استفاده از uint16_t (عدد 0-4000)
        void prepareDataFromValue(uint16_t value) {
            uint8_t byte1 = (value >> 8) & 0xFF;   // بایت بزرگتر
            uint8_t byte2 = value & 0xFF;          // بایت کوچکتر
            uint8_t checksum = (byte1 > byte2) ? (byte1 - byte2) : (byte2 - byte1);
            
            m_txBuffer[0] = byte1;
            m_txBuffer[1] = byte2;
            m_txBuffer[2] = checksum;
            m_bytesToSend = 3;
            m_txIndex = 0;
            
            std::cout << "[VL530X] Prepared data from value " << value 
                      << " -> bytes: (" << (int)byte1 << ", " << (int)byte2 
                      << "), checksum: " << (int)checksum << std::endl;
        }
        
        // بررسی اینکه آیا سنسور آدرس دیده یا نه
        bool isAddressed() const {
            return m_addressed;
        }
        
        // ریست کردن وضعیت سنسور
        void reset() {
            m_state = IDLE;
            m_addressed = false;
            m_txIndex = 0;
            m_bytesToSend = 0;
        }
        // ====================================================

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