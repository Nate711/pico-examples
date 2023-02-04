add_executable(spi_slave
        spi_slave.cc
        )

# Pull in basic dependencies
target_link_libraries(spi_slave pico_stdlib hardware_spi)


pico_enable_stdio_usb(spi_slave 1)
pico_enable_stdio_uart(spi_slave 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(spi_slave)

# add url via pico_set_program_url
example_auto_set_url(spi_slave)


add_executable(spi_slave_blocking
        spi_slave_blocking.cc
        )

# Pull in basic dependencies
target_link_libraries(spi_slave_blocking pico_stdlib hardware_spi)


pico_enable_stdio_usb(spi_slave_blocking 1)
pico_enable_stdio_uart(spi_slave_blocking 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(spi_slave_blocking)

# add url via pico_set_program_url
example_auto_set_url(spi_slave_blocking)