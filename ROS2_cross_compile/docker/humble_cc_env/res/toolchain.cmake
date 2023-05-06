# https://github.com/cyberbotics/epuck_ros2/blob/master/installation/cross_compile/toolchain.cmake

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_LIBRARY_ARCHITECTURE arm-linux-gnueabihf)
set(CMAKE_CROSSCOMPILING 1)

set(CMAKE_C_COMPILER /bin/arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER /bin/arm-linux-gnueabihf-g++)
set(CMAKE_SYSROOT /opt/rootfs)

# https://github.com/eProsima/Fast-DDS/issues/1262
set(CMAKE_CXX_FLAGS "-latomic")

set(CMAKE_FIND_ROOT_PATH /opt/ros2_humble/install)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(PYTHON_SOABI cpython-38-arm-linux-gnueabihf)
set(PYTHON_EXECUTABLE /opt/rootfs/usr/bin/python3.8)
set(CMAKE_THREADS_PTHREAD_ARG 0)
set(CMAKE_C_FLAGS -Wno-psabi)
set(CMAKE_CXX_FLAGS -Wno-psabi)

# https://github.com/foonathan/memory/pull/60
set(CMAKE_CROSSCOMPILING_EMULATOR /usr/bin/qemu-arm-static)