#ifndef MICROCONTROLLER_H
#define MICROCONTROLLER_H

#include <CPS4042/Hardwares/Boards/Esp8266.h>
#include <CPS4042/Sketchs/AbstractSketch.h>
#include <CPS4042/Utils/ByteStream.h>
#include <CPS4042/Utils/Wave.h>
#include <bitset>

class MicroController : public AbstractSketch<Boards::Esp8266>
{
public:
    explicit MicroController(Boards::Esp8266* node) :
        AbstractSketch<Boards::Esp8266> {node}
    {}

    std::int32_t
    setup(Boards::Esp8266::Gpio& gpio) override
    {
        std::cout << "esp8266 setup completed." << std::endl;
        node()->i2c.write(0x29);
        delay(500);
        return 0;
    }

    std::int32_t
    loop(Boards::Esp8266::Gpio& gpio) override
    {
        if (node()->i2c.isDataAvailable()){
            Byte high = node()->i2c.read();
            Byte low = node()->i2c.read();
            uint16_t distance = (static_cast<uint16_t>(high) << 8) | low;
            std::cout << "Distance: " << distance << " mm" << std::endl;
            node()->i2c.write(0x29);

        }
        delay(100);
        return 0;
    }
};


#endif    // MICROCONTROLLER_H
