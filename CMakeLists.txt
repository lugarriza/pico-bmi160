cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
project(test_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# rest of your project

add_executable(bmi160_i2c
    bmi160_i2c.c
)
# pico_enable_stdio_usb(hello_world 1)
pico_enable_stdio_uart(bmi160_i2c 1)

add_subdirectory(bmi160)



# target_include_directories(bmi160_i2c PRIVATE "${PROJECT_SOURCE_DIR}/../../bmi160")

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(
		 bmi160_i2c
		 pico_stdlib
		 hardware_i2c
		 bmi160
		 )

# create map/bin/hex/uf2 file in addition to ELF.
# pico_add_extra_outputs(bmi160_i2c_i2c)
