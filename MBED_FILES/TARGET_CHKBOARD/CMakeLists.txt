add_library(mbed-chkboard INTERFACE)

target_sources(mbed-chkboard
    INTERFACE
        PeripheralPins.c
)

target_include_directories(mbed-chkboard
    INTERFACE
        .
)

target_link_libraries(mbed-chkboard INTERFACE mbed-stm32f407xe)