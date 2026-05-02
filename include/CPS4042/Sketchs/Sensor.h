#ifndef SENSOR_H
#define SENSOR_H

#include <CPS4042/Hardwares/Sensors/VL530X.h>
#include <CPS4042/Sketchs/AbstractSketch.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <ctime>  // ==================== اضافه شده برای time(nullptr)

class Sensor : public AbstractSketch<Sensors::Vl530x>
{
// ==================== بخش اضافه شده ====================
private:
    boost::random::mt19937 m_gen;                     // تولید کننده اعداد تصادفی (Mersenne Twister)
    boost::random::uniform_int_distribution<> m_dist; // توزیع یکنواخت اعداد بین 0 تا 4000
    uint16_t m_lastValue;                             // آخرین مقداری که تولید شده است
    int m_sendCounter;                                // شمارنده تعداد دفعات ارسال داده
// ====================================================

public:
    explicit Sensor(Sensors::Vl530x* node) :
        AbstractSketch<Sensors::Vl530x> {node}
        // ==================== بخش اضافه شده ====================
        : m_gen(static_cast<unsigned int>(std::time(nullptr))),  // seed تصادفی بر اساس زمان فعلی
          m_dist(0, 4000),                                        // محدوده اعداد: 0 تا 4000
          m_lastValue(0),
          m_sendCounter(0)
        // ====================================================
    {}

    std::int32_t
    setup(Sensors::Vl530x::Gpio& gpio) override
    // ==================== بخش اضافه شده ====================
    {
        std::cout << "\n========== VL530X SENSOR SETUP ==========" << std::endl;
        std::cout << "vl530x setup completed." << std::endl;
        
        // مقداردهی اولیه پروتکل I2C با آدرس خود سنسور (0x29)
        // آدرس سنسور به صورت ثابت در کلاس Vl530x تعریف شده است
        node()->i2c.init(Sensors::Vl530x::address);
        
        std::cout << "I2C initialized with own address: 0x" 
                  << std::hex << (int)Sensors::Vl530x::address << std::dec << std::endl;
        std::cout << "Random number range: 0 to 4000" << std::endl;
        std::cout << "==========================================" << std::endl;
        
        return 0;
    }
    // ====================================================

    std::int32_t
    loop(Sensors::Vl530x::Gpio& gpio) override
    // ==================== بخش اضافه شده ====================
    {
        // 1. تولید عدد تصادفی بین 0 تا 4000
        m_lastValue = m_dist(m_gen);
        m_sendCounter++;
        
        // 2. تبدیل عدد 16 بیتی به دو بایت مجزا
        //    byte1 = بایت پر ارزش (بایت اول / High Byte)
        //    byte2 = بایت کم ارزش (بایت دوم / Low Byte)
        uint8_t byte1 = (m_lastValue >> 8) & 0xFF;   // شیفت 8 بیت به راست برای گرفتن بایت اول
        uint8_t byte2 = m_lastValue & 0xFF;          // ماسک کردن 8 بیت آخر برای گرفتن بایت دوم
        
        // 3. محاسبه checksum = تفاوت بایت بزرگتر و کوچکتر
        //    طبق صورت سوال: "بایت سوم حاصل تفریق بایت بزرگتر عدد و بایت کوچکتر باشد"
        uint8_t checksum = (byte1 > byte2) ? (byte1 - byte2) : (byte2 - byte1);
        
        // 4. آماده کردن داده برای ارسال از طریق I2C
        //    داده شامل 3 بایت است: [byte1, byte2, checksum]
        Byte dataToSend[3] = { byte1, byte2, checksum };
        node()->i2c.setDataToSend(dataToSend);
        
        // 5. چاپ اطلاعات برای دیباگ
        std::cout << "\n[SENSOR #" << m_sendCounter << "] Activity:" << std::endl;
        std::cout << "  Generated random value: " << m_lastValue << std::endl;
        std::cout << "  -> Byte1 (high byte):  " << (int)byte1 << std::endl;
        std::cout << "  -> Byte2 (low byte):   " << (int)byte2 << std::endl;
        std::cout << "  -> Checksum:           " << (int)checksum << std::endl;
        std::cout << "  Data ready for I2C transmission. Waiting for master request..." << std::endl;
        
        // 6. تاخیر 2 ثانیه‌ای بین هر بار تولید داده
        //    طبق صورت سوال: سنسور هر بار یک عدد تصادفی جدید تولید می‌کند
        delay(2000);
        
        return 0;
    }
    // ====================================================

    // ==================== توابع کمکی اضافه شده (برای دیباگ و دسترسی) ====================
    // دریافت آخرین مقدار تولید شده
    uint16_t getLastValue() const {
        return m_lastValue;
    }
    
    // دریافت شمارنده تعداد ارسال‌ها
    int getSendCounter() const {
        return m_sendCounter;
    }
    // ====================================================
};

#endif // SENSOR_H