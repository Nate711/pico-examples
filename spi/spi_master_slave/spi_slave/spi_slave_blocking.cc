// Copyright (c) 2021 Michael Stoops. All rights reserved.
// Portions copyright (c) 2021 Raspberry Pi (Trading) Ltd.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
// following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
//    disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
//    following disclaimer in the documentation and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
//    products derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
// USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// SPDX-License-Identifier: BSD-3-Clause
//
// Example of an SPI bus slave using the PL022 SPI interface

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#define BUF_LEN 8

// I don't think SPI freq means anything for a slave - haven't found difference
// #define SPI_FREQ 10000000
#define SPI_FREQ 1000000

uint16_t out_buf[BUF_LEN], in_buf[BUF_LEN];
uint8_t in_buf8[BUF_LEN * 2];
int i = 0;

void printbuf8(uint8_t buf[], size_t len)
{
    for (size_t i = 0; i < len; ++i)
    {
        printf("%02x ", buf[i]);
    }
}

void printbuf16(uint16_t buf[], size_t len)
{
    for (size_t i = 0; i < len; ++i)
    {
        printf("%02x ", buf[i]);
    }
}

void buf16_to_buf8(uint16_t buf16[], uint8_t buf8[], size_t len_buf16)
{
    for (size_t i = 0; i < len_buf16; i++)
    {
        buf8[2 * i] = static_cast<uint8_t>((buf16[i] >> 8) & 0xff);
        buf8[2 * i + 1] = static_cast<uint8_t>(buf16[i] & 0xff);
    }
}

int main()
{
    // Enable UART so we can print
    stdio_init_all();
#if !defined(spi_default) || !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || !defined(PICO_DEFAULT_SPI_RX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)
#warning spi/spi_slave example requires a board with SPI pins
    puts("Default SPI pins were not defined");
#else

    printf("SPI slave example\n");

    // Enable SPI 0 at 1 MHz and connect to GPIOs
    spi_init(spi_default, SPI_FREQ);
    spi_set_slave(spi_default, true);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI);

    spi_set_format(spi0, 16, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    // Initialize output buffer with uint16_t 8 to 0
    for (size_t i = 0; i < BUF_LEN; ++i)
    {
        out_buf[i] = BUF_LEN - i - 1;
    }

    // gpio_set_irq_enabled_with_callback(PICO_DEFAULT_SPI_CSN_PIN, GPIO_IRQ_EDGE_FALL, true, &cs_low_callback);
    int count = 0;
    while (1)
    {
        spi_write16_read16_blocking(spi_default, out_buf, in_buf, BUF_LEN);
        buf16_to_buf8(in_buf, in_buf8, BUF_LEN);

        // Write to stdio whatever came in on the MOSI line.
        // printf("SPI slave says: read page %d from the MOSI line:\n", i);
        if (count % 100 == 0)
        {
            printf("baud_rate: %d\n", spi_get_baudrate(spi_default));
            printf("(blocking) Page %d from the MOSI line: ", i++);
            printbuf8(in_buf8, BUF_LEN * 2);
            printf("\twrote: ");
            printbuf16(out_buf, BUF_LEN);
            putchar('\n');
        }
        count++;
    }
    return 0;
#endif
}
