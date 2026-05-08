#ifndef I2CMULTIPLEXER_H
#define I2CMULTIPLEXER_H

#include <CPS4042/Hardwares/Comm/I2CMux.h>
#include <CPS4042/Sketchs/AbstractSketch.h>

class I2CMultiplexer : public AbstractSketch<Hardware::I2CMux>
{
public:
    explicit I2CMultiplexer(Hardware::I2CMux* node) : AbstractSketch<Hardware::I2CMux>{node} {}

    std::int32_t setup(Hardware::I2CMux::Gpio&) override {
        std::cout << "I2CMux setup completed." << std::endl;
        return 0;
    }

    std::int32_t loop(Hardware::I2CMux::Gpio&) override {
        return 0;
    }
};

#endif