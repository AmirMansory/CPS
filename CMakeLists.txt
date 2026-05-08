cmake_minimum_required(VERSION 3.19)
project(CPS4042 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------------------------------------
# Common simulation core headers (used by all three executables)
# ------------------------------------------------------------
set(CORE_HEADERS
    include/CPS4042/main.h
    include/CPS4042/Globals/Object.h
    include/CPS4042/Protocols/Protocol.h
    include/CPS4042/Wires/Link.h
    include/CPS4042/Wires/Pin.h
    include/CPS4042/Hardwares/Transmitter.h
    include/CPS4042/Hardwares/Board.h
    include/CPS4042/Hardwares/Processor.h
    include/CPS4042/Units/Bit.h
    include/CPS4042/Units/Byte.h
    include/CPS4042/Units/Voltage.h
    include/CPS4042/Units/Frequency.h
    include/CPS4042/Units/BaudRate.h
    include/CPS4042/Utils/Queue.h
    include/CPS4042/Utils/Wave.h
    include/CPS4042/Utils/ByteStream.h
    include/CPS4042/Hardwares/Boards/Esp8266.h
    include/CPS4042/Sketchs/AbstractSketch.h
)

# Common include directories (boost + local include)
set(COMMON_INCLUDES
    ${CMAKE_SOURCE_DIR}/include
    C:/boost/boost_1_84_0
)

# ------------------------------------------------------------
# Target 1 : Direct I2C (3‑byte sensor)
# ------------------------------------------------------------
add_executable(CPS4042_I2C
    src/main_i2c.cpp
    ${CORE_HEADERS}
    include/CPS4042/Hardwares/Sensors/VL530X.h      # original 3‑byte sensor
    include/CPS4042/Sketchs/I2C/Microcontroller_I2C.h
    include/CPS4042/Sketchs/I2C/Sensor_I2C.h
)
target_include_directories(CPS4042_I2C PRIVATE ${COMMON_INCLUDES})

# ------------------------------------------------------------
# Target 2 : Full‑Duplex USART
# ------------------------------------------------------------
add_executable(CPS4042_USART
    src/main_usart.cpp
    ${CORE_HEADERS}
    include/CPS4042/Hardwares/Comm/Usb.h
    include/CPS4042/Sketchs/USART/Microcontroller_USART.h
    include/CPS4042/Sketchs/USART/HardDisk.h
)
target_include_directories(CPS4042_USART PRIVATE ${COMMON_INCLUDES})

# ------------------------------------------------------------
# Target 3 : I2C Multiplexer (4 sensors, 1‑byte each)
# ------------------------------------------------------------
add_executable(CPS4042_MUX
    src/main_mux.cpp
    ${CORE_HEADERS}
    include/CPS4042/Hardwares/Comm/I2CMux.h
    include/CPS4042/Hardwares/Sensors/VL530X_MUX.h  # 1‑byte sensor
    include/CPS4042/Sketchs/MUX/Microcontroller_MUX.h
    include/CPS4042/Sketchs/MUX/Sensor_MUX.h
    include/CPS4042/Sketchs/MUX/I2CMultiplexer.h
)
target_include_directories(CPS4042_MUX PRIVATE ${COMMON_INCLUDES})

# ------------------------------------------------------------
# Install rules (optional)
# ------------------------------------------------------------
include(GNUInstallDirs)

install(TARGETS CPS4042_I2C CPS4042_USART CPS4042_MUX
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)