#include <CPS4042/main.h>
#include <CPS4042/Hardwares/Boards/Esp8266.h>
#include <CPS4042/Hardwares/Comm/Usb.h>            
#include <CPS4042/Sketchs/USART/Microcontroller_USART.h>
#include <CPS4042/Sketchs/USART/HardDisk.h>
#include <CPS4042/Wires/Link.h>
#include <memory>

int main()
{
    Boards::Esp8266 esp8266;
    Sensors::Usb    usb;

    auto linkTx  = std::make_shared<Link>();   
    auto linkRx  = std::make_shared<Link>();   
    auto linkVdd = std::make_shared<Link>();
    auto linkGnd = std::make_shared<Link>();

    CPS_SET_OBJECT_NAME(esp8266);
    CPS_SET_OBJECT_NAME(usb);
    CPS_SET_OBJECT_NAME_PTR(linkTx);
    CPS_SET_OBJECT_NAME_PTR(linkRx);
    CPS_SET_OBJECT_NAME_PTR(linkVdd);
    CPS_SET_OBJECT_NAME_PTR(linkGnd);

    // Power
    esp8266.gpio().vdd2.attachLink(linkVdd);
    esp8266.gpio().gnd2.attachLink(linkGnd);
    usb.gpio().vdd.attachLink(linkVdd);
    usb.gpio().gnd.attachLink(linkGnd);

    esp8266.gpio().tx.attachLink(linkTx);
    usb.gpio().tx.attachLink(linkTx);           // slave reads from its TX
    esp8266.gpio().rx.attachLink(linkRx);
    usb.gpio().rx.attachLink(linkRx);           // slave writes on its RX

    MicroController_USART micro(&esp8266);
    HardDisk              disk(&usb);

    CPS_SET_OBJECT_NAME(micro);
    CPS_SET_OBJECT_NAME(disk);

    micro.start();
    disk.start();

    return Application::exec();
}