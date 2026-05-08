#ifndef SENSOR_MUX_H
#define SENSOR_MUX_H

#include <CPS4042/Hardwares/Sensors/VL530X_MUX.h>   // 1‑byte sensor
#include <CPS4042/Sketchs/AbstractSketch.h>
#include <cstdlib>
#include <ctime>

class Sensor_MUX : public AbstractSketch<Sensors::Vl530x>{
    int m_id;   // 0..3 to differentiate channels
public:
    explicit Sensor_MUX(Sensors::Vl530x* node, int id) 
        : AbstractSketch{node}, m_id(id) {}

    std::int32_t setup(Sensors::Vl530x::Gpio&) override{
        // Different seed per sensor
        std::srand(static_cast<unsigned>(std::time(nullptr)) + m_id * 100);
        generateAndSetData();
        std::cout << "[MUX SENSOR " << m_id << "] Ready" << std::endl;
        return 0;
    }

    std::int32_t loop(Sensors::Vl530x::Gpio&) override{
        if (node()->i2c.isReady()){
            generateAndSetData();
        }
        delay(50);
        return 0;
    }

private:
    void generateAndSetData(){
        uint8_t value;
        switch (m_id){
            case 0:  value = std::rand() % 21;            
                     break;
            case 1:  value = 50 + (std::rand() % 51);     
                     break;
            case 2:  value = 'A' + (std::rand() % 26);    
                     break;
            default: value = std::rand() % 256;           
        }
        node()->i2c.setData(value);   // one‑byte mode
        std::cout << "[SENSOR " << m_id << "] new data: " 
                  << static_cast<int>(value);
        if (m_id == 2) std::cout << " ('" << static_cast<char>(value) << "')";
        std::cout << std::endl;
    }
};

#endif