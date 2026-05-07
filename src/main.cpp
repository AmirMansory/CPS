
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
  
  mc.start();


  
  sensor.start();
 




  
    

  return Application::exec();
}

// #include <CPS4042/Sketchs/Microcontroller.h>
// #include <CPS4042/Sketchs/HardDisk.h>
// #include <CPS4042/Units/Bit.h>
// #include <CPS4042/Units/Byte.h>
// #include <CPS4042/Wires/Pin.h>
// #include <CPS4042/main.h>

// // I2C header not needed anymore
// // #include <CPS4042/Sketchs/Sensor.h>

// std::int32_t main()
// {
//     Boards::Esp8266 esp8266;
//     Sensors::Usb    usb;



//     // Links for USART
//     auto linkTx = std::make_shared<Link>();   // MCU.TX -> USB.RX
//     auto linkRx = std::make_shared<Link>();   // USB.TX -> MCU.RX
//     auto linkVdd = std::make_shared<Link>();
//     auto linkGnd = std::make_shared<Link>();

//     CPS_SET_OBJECT_NAME(esp8266);
//     CPS_SET_OBJECT_NAME(usb);
//     CPS_SET_OBJECT_NAME_PTR(linkTx);
//     CPS_SET_OBJECT_NAME_PTR(linkRx);
//     CPS_SET_OBJECT_NAME_PTR(linkVdd);
//     CPS_SET_OBJECT_NAME_PTR(linkGnd);

//     // Power connections (can use unused pins)
//     esp8266.gpio().vdd2.attachLink(linkVdd);
//     esp8266.gpio().gnd2.attachLink(linkGnd);
//     usb.gpio().vdd.attachLink(linkVdd);
//     usb.gpio().gnd.attachLink(linkGnd);



//     // Attach Esp8266 first to set baudrate
//     esp8266.gpio().tx.attachLink(linkTx);
//     usb.gpio().tx.attachLink(linkTx);
//     esp8266.gpio().rx.attachLink(linkRx);
//     usb.gpio().rx.attachLink(linkRx);
//       // disk TX → MCU RX

//     // Sketches
//     MicroController micro(&esp8266);
//     HardDisk        disk(&usb);

//     CPS_SET_OBJECT_NAME(micro);
//     CPS_SET_OBJECT_NAME(disk);

//     micro.start();
//     disk.start();

//     return Application::exec();
// }