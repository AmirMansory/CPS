#include <CPS4042/main.h>
#include <CPS4042/Hardwares/Boards/Esp8266.h>
#include <CPS4042/Hardwares/Sensors/VL530X.h>
#include <CPS4042/Sketchs/Microcontroller.h>
#include <CPS4042/Sketchs/Sensor.h>
#include <CPS4042/Wires/Link.h>
#include <memory>

int main()
{
    Boards::Esp8266 esp8266;
    Sensors::Vl530x  vl530x;

    auto linkVdd = std::make_shared<Link>();
    auto linkGnd = std::make_shared<Link>();
    auto linkScl = std::make_shared<Link>();
    auto linkSda = std::make_shared<Link>();

    CPS_SET_OBJECT_NAME(esp8266);
    CPS_SET_OBJECT_NAME(vl530x);
    CPS_SET_OBJECT_NAME_PTR(linkVdd);
    CPS_SET_OBJECT_NAME_PTR(linkGnd);
    CPS_SET_OBJECT_NAME_PTR(linkScl);
    CPS_SET_OBJECT_NAME_PTR(linkSda);

    esp8266.gpio().vdd1.attachLink(linkVdd);
    esp8266.gpio().gnd1.attachLink(linkGnd);
    esp8266.gpio().scl.attachLink(linkScl);
    esp8266.gpio().sda.attachLink(linkSda);

    vl530x.gpio().vdd.attachLink(linkVdd);
    vl530x.gpio().gnd.attachLink(linkGnd);
    vl530x.gpio().scl.attachLink(linkScl);
    vl530x.gpio().sda.attachLink(linkSda);

    MicroController mc(&esp8266);
    Sensor          sensor(&vl530x);

    // IMPORTANT: start sensor first so it can listen to SCL edges
    sensor.start();
    mc.start();

    return Application::exec();
}