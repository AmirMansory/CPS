#ifndef MICROCONTROLLER_H
#define MICROCONTROLLER_H

#include <CPS4042/Hardwares/Boards/Esp8266.h>
#include <CPS4042/Sketchs/AbstractSketch.h>
#include <CPS4042/Utils/ByteStream.h>
#include <CPS4042/Utils/Wave.h>
#include <bitset>

class MicroController : public AbstractSketch<Boards::Esp8266>
{
// ==================== بخش اضافه شده ====================
private:
    uint8_t m_rxBuffer[3];      // بافر برای ذخیره 3 بایت دریافتی از سنسور
    int m_rxIndex;              // اندیس فعلی در بافر دریافت
    bool m_waitingForData;      // وضعیت: آیا منتظر دریافت داده از سنسور هستیم؟
    int m_requestCounter;       // شمارنده درخواست‌ها (برای دیباگ)
// ====================================================

public:
    explicit MicroController(Boards::Esp8266* node) :
        AbstractSketch<Boards::Esp8266> {node},
        // ==================== بخش اضافه شده ====================
        m_rxIndex(0),
        m_waitingForData(false),
        m_requestCounter(0)
        // ====================================================
    {}

    std::int32_t
    setup(Boards::Esp8266::Gpio& gpio) override
    // ==================== بخش اضافه شده ====================
    {
        std::cout << "\n========== ESP8266 SETUP ==========" << std::endl;
        std::cout << "esp8266 setup completed." << std::endl;
        
        // مقداردهی اولیه I2C با آدرس سنسور VL530X (0x29)
        node()->i2c.init(Sensors::Vl530x::address);
        
        std::cout << "I2C initialized with sensor address: 0x" 
                  << std::hex << (int)Sensors::Vl530x::address << std::dec << std::endl;
        std::cout << "===================================\n" << std::endl;
        
        return 0;
    }
    // ====================================================

    std::int32_t
    loop(Boards::Esp8266::Gpio& gpio) override
    // ==================== بخش اضافه شده ====================
    {
        if (!m_waitingForData) {
            // شروع درخواست داده جدید از سنسور
            m_requestCounter++;
            std::cout << "\n[REQUEST #" << m_requestCounter << "] Asking sensor for data..." << std::endl;
            
            // ارسال آدرس سنسور برای شروع ارتباط
            node()->i2c.write(Sensors::Vl530x::address);
            
            m_waitingForData = true;
            m_rxIndex = 0;
            
            // پاک کردن بافر
            for (int i = 0; i < 3; i++) {
                m_rxBuffer[i] = 0;
            }
        } 
        else {
            // در حال دریافت داده از سنسور هستیم
            // توجه: در این شبیه‌ساز، داده‌ها از طریق تابع read دریافت می‌شوند
            // و وضعیت توسط توابع کمکی I2C مدیریت می‌شود
            
            // چک می‌کنیم که آیا داده‌ای از سنسور رسیده است؟
            // (این بخش بستگی به نحوه پیاده‌سازی شبیه‌ساز دارد)
            
            // اگر همه 3 بایت را دریافت کردیم، داده را بررسی کن
            if (m_rxIndex >= 3) {
                // بررسی checksum
                uint8_t byte1 = m_rxBuffer[0];
                uint8_t byte2 = m_rxBuffer[1];
                uint8_t receivedChecksum = m_rxBuffer[2];
                
                uint8_t calculatedChecksum = (byte1 > byte2) ? (byte1 - byte2) : (byte2 - byte1);
                
                std::cout << "\n[RECEIVED DATA]" << std::endl;
                std::cout << "  Byte 1 (high): " << (int)byte1 << std::endl;
                std::cout << "  Byte 2 (low):  " << (int)byte2 << std::endl;
                std::cout << "  Checksum (recv): " << (int)receivedChecksum << std::endl;
                std::cout << "  Checksum (calc): " << (int)calculatedChecksum << std::endl;
                
                if (receivedChecksum == calculatedChecksum) {
                    uint16_t value = (byte1 << 8) | byte2;
                    std::cout << "\n✓✓✓ VALID DATA ✓✓✓" << std::endl;
                    std::cout << "  Sensor value: " << value << std::endl;
                } else {
                    std::cout << "\n✗✗✗ CHECKSUM ERROR ✗✗✗" << std::endl;
                    std::cout << "  Data corrupted!" << std::endl;
                }
                
                std::cout << "-----------------------------------" << std::endl;
                
                // آماده برای درخواست بعدی
                m_waitingForData = false;
            }
        }
        
        // کمی تاخیر برای جلوگیری از busy-loop
        delay(500);
        
        return 0;
    }
    // ====================================================
    
    // ==================== توابع کمکی اضافه شده ====================
    // تابع برای دریافت داده از سنسور (توسط I2C صدا زده می‌شود)
    void onDataReceived(Byte data) {
        if (m_waitingForData && m_rxIndex < 3) {
            m_rxBuffer[m_rxIndex] = data;
            std::cout << "[Micro] Received byte " << m_rxIndex + 1 
                      << ": " << (int)data << std::endl;
            m_rxIndex++;
        }
    }
    
    // بررسی اینکه آیا منتظر داده هستیم
    bool isWaitingForData() const {
        return m_waitingForData;
    }
    
    // دریافت اندیس فعلی بافر
    int getRxIndex() const {
        return m_rxIndex;
    }
    // ====================================================
};

#endif    // MICROCONTROLLER_H