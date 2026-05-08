#include <CPS4042/main.h>
#include <CPS4042/Hardwares/Boards/Esp8266.h>
#include <CPS4042/Hardwares/Sensors/VL530X_MUX.h>         
#include <CPS4042/Sketchs/MUX/Microcontroller_MUX.h>
#include <CPS4042/Sketchs/MUX/Sensor_MUX.h>
#include <CPS4042/Wires/Link.h>
#include <memory>
#include <CPS4042/Hardwares/Comm/I2CMux.h>
#include <CPS4042/Sketchs/MUX/I2CMultiplexer.h>

int main()
{
    Boards::Esp8266    esp8266;
    Hardware::I2CMux   mux;
    Sensors::Vl530x    sensor[4];      


    auto linkVdd = std::make_shared<Link>();
    auto linkGnd = std::make_shared<Link>();
    auto linkTx  = std::make_shared<Link>();  
    auto linkRx  = std::make_shared<Link>();   

    CPS_SET_OBJECT_NAME(esp8266);
    CPS_SET_OBJECT_NAME(mux);
    CPS_SET_OBJECT_NAME_PTR(linkVdd);
    CPS_SET_OBJECT_NAME_PTR(linkGnd);
    CPS_SET_OBJECT_NAME_PTR(linkTx);
    CPS_SET_OBJECT_NAME_PTR(linkRx);

  
    esp8266.gpio().vdd2.attachLink(linkVdd);
    esp8266.gpio().gnd2.attachLink(linkGnd);
    mux.gpio().vdd.attachLink(linkVdd);
    mux.gpio().gnd.attachLink(linkGnd);

    esp8266.gpio().tx.attachLink(linkTx);
    mux.gpio().tx.attachLink(linkTx);          
    esp8266.gpio().rx.attachLink(linkRx);
    mux.gpio().rx.attachLink(linkRx);          

    auto linkSda[4] = { std::make_shared<Link>(), std::make_shared<Link>(),
                        std::make_shared<Link>(), std::make_shared<Link>() };
    auto linkScl[4] = { std::make_shared<Link>(), std::make_shared<Link>(),
                        std::make_shared<Link>(), std::make_shared<Link>() };

    for (int i = 0; i < 4; ++i)
    {
        sensor[i].gpio().vdd.attachLink(linkVdd);
        sensor[i].gpio().gnd.attachLink(linkGnd);

        switch (i) {
            case 0: mux.gpio().sda0.attachLink(linkSda[i]); mux.gpio().scl0.attachLink(linkScl[i]); break;
            case 1: mux.gpio().sda1.attachLink(linkSda[i]); mux.gpio().scl1.attachLink(linkScl[i]); break;
            case 2: mux.gpio().sda2.attachLink(linkSda[i]); mux.gpio().scl2.attachLink(linkScl[i]); break;
            case 3: mux.gpio().sda3.attachLink(linkSda[i]); mux.gpio().scl3.attachLink(linkScl[i]); break;
        }

        sensor[i].gpio().sda.attachLink(linkSda[i]);
        sensor[i].gpio().scl.attachLink(linkScl[i]);
    }

    MicroController_MUX  mc(&esp8266);
    I2CMultiplexer       muxSketch(&mux);
    Sensor_MUX           s0(&sensor[0], 0); 
    Sensor_MUX           s1(&sensor[1], 1);   
    Sensor_MUX           s2(&sensor[2], 2);   
    Sensor_MUX           s3(&sensor[3], 3);   

    muxSketch.start();
    mc.start();
    s0.start(); s1.start(); s2.start(); s3.start();

    return Application::exec();
}