

add_executable(main
        main.cpp
        servo.cpp
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
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(main )

# add url via pico_set_program_url
example_auto_set_url(main )

