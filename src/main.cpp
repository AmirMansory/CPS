
//Negin
#include <CPS4042/Sketchs/HardDisk.h>
#include <CPS4042/Sketchs/Microcontroller.h>
#include <CPS4042/Units/Bit.h>
#include <CPS4042/Units/Byte.h>
#include <CPS4042/Wires/Pin.h>
#include <CPS4042/main.h>

std::int32_t main()
{
    Boards::Esp8266 esp8266;
    Sensors::Usb usb; 
    
    auto linkRed    = std::make_shared<Link>();
    auto linkBlack  = std::make_shared<Link>();
    
    CPS_SET_OBJECT_NAME(esp8266);
    CPS_SET_OBJECT_NAME(usb);
    
    CPS_SET_OBJECT_NAME_PTR(linkRed);
    CPS_SET_OBJECT_NAME_PTR(linkBlack);
    
    esp8266.gpio().vdd1.attachLink(linkRed);
    usb.gpio().vdd.attachLink(linkRed);
    
    esp8266.gpio().gnd1.attachLink(linkBlack);
    usb.gpio().gnd.attachLink(linkBlack);
    
    auto usartLinkTx = std::make_shared<Link>();
    auto usartLinkRx = std::make_shared<Link>();
    CPS_SET_OBJECT_NAME_PTR(usartLinkTx);
    CPS_SET_OBJECT_NAME_PTR(usartLinkRx);
    
    esp8266.gpio().tx.attachLink(usartLinkTx);
    usb.gpio().rx.attachLink(usartLinkTx);
    
    esp8266.gpio().rx.attachLink(usartLinkRx);
    usb.gpio().tx.attachLink(usartLinkRx);
    
    MicroController micro(&esp8266);
    HardDisk hardDisk(&usb);
    
    micro.start();
    hardDisk.start();
    
    return Application::exec();
}