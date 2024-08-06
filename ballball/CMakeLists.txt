

add_executable(main
        main.cpp
        servo.cpp
        dhcpserver/dhcpserver.c
        dnsserver/dnsserver.c
        )

target_include_directories(main PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        ${CMAKE_CURRENT_LIST_DIR}/dhcpserver
        ${CMAKE_CURRENT_LIST_DIR}/dnsserver
        )

# pull in common dependencies and additional pwm hardware support
target_link_libraries(main
        pico_stdlib
        hardware_pwm
        hardware_clocks
        hardware_irq
        hardware_i2c
        hardware_uart
        hardware_adc
        pico_multicore
        pico_cyw43_arch_lwip_threadsafe_background
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(main )

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

# add url via pico_set_program_url
example_auto_set_url(main )
