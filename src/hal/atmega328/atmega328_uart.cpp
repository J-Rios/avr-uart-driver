
/**
 * @file    atmega328_uart.cpp
 * @author  Jose Miguel Rios Rubio <jrios.github@gmail.com>
 * @date    26-01-2022
 * @version 1.0.0
 *
 * @section DESCRIPTION
 *
 * UART driver implementation for the following AVR devices:
 * ATmega48A/48PA/88A/88PA/168A/168PA/328/328P
 *
 * @section LICENSE
 *
 * Copyright (c) 2020 Jose Miguel Rios Rubio. All right reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */

/*****************************************************************************/

/* Include Guards */

#if defined(__AVR_ATmega48__)    || defined(__AVR_ATmega48A__)   || \
    defined(__AVR_ATmega48PA__)  || defined(__AVR_ATmega48PB__)  || \
    defined(__AVR_ATmega48P__)   || defined(__AVR_ATmega88__)    || \
    defined(__AVR_ATmega88A__)   || defined(__AVR_ATmega88P__)   || \
    defined(__AVR_ATmega88PA__)  || defined(__AVR_ATmega88PB__)  || \
    defined(__AVR_ATmega168__)   || defined(__AVR_ATmega168A__)  || \
    defined(__AVR_ATmega168P__)  || defined(__AVR_ATmega168PA__) || \
    defined(__AVR_ATmega168PB__) || defined(__AVR_ATmega328__)   || \
    defined(__AVR_ATmega328P__)

/*****************************************************************************/

/* Libraries */

// Header Interface
#include "atmega328_uart.h"

// Device/Framework Headers
#include <avr/io.h>

/*****************************************************************************/

/* Constants & Defines */

// Last available UART on this device
#define LAST_UART (AVR_NUM_UARTS - 1)

/*****************************************************************************/

/* Public Methods */

/**
 * @details
 * The constructor of the class. Takes the CPU Frequency that will be used by
 * the driver.
 */
AvrUart::AvrUart(const uint32_t f_cpu)
{
    _f_cpu = f_cpu;
    for (uint8_t i = 0; i < AVR_NUM_UARTS; i++)
        _uart_configured[i] = false;
}

/**
 * @details
 * The destructor of the class.
 */
AvrUart::~AvrUart()
{}

/**
 * @details
 * This function configure an UART to be used at specified speed.
 */
bool AvrUart::setup(const uint8_t uart_n,
        const uint32_t baud_rate, const bool internal_res_pullup)
{
    // Ignore if invalid UART number provided
    if (uart_n > LAST_UART)
        return false;

    // Set Baud Rate
    UBRR0H = (uint8_t)(((_f_cpu / (16L * baud_rate)) - 1) >> 8);
    UBRR0L = (uint8_t)((_f_cpu / (16L * baud_rate)) - 1);

    // Enable Tx & Rx
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);

    // Set frame format: 8 data, 2 stop bit
    UCSR0C = (1 << USBS0) | (3 << UCSZ00);

    // Set internal pull-up resistor on RXD0 pin to supress line noise
    if (internal_res_pullup)
    {
        DDRD &= ~_BV(PIND0);
        PORTD |= _BV(PIND0);
    }

    _uart_configured[uart_n] = true;

    return true;
}

/**
 * @details
 * This function write to the corresponding UART Serial interface the provided
 * byte of data.
 */
bool AvrUart::write(const uint8_t uart_n, const uint8_t write_byte)
{
    // Ignore if invalid UART number provided or UART is not configured yet
    if (uart_n > LAST_UART)
        return false;
    if (!_uart_configured[uart_n])
        return false;

    // Wait until transmit buffer is empty
    // Make sure any previous transmision has been completed
    while (!is_uart_tx_buffer_available(uart_n));

    // Add data byte into transmit buffer
    UDR0 = write_byte;

    return true;
}

/**
 * @details
 * This function checks if some data has been received and stored in UART
 * receive buffer, and get-return one byte from the received buffer.
 */
bool AvrUart::read(const uint8_t uart_n, uint8_t* read_byte)
{
    // Ignore if invalid UART number provided or UART is not configured yet
    if (uart_n > LAST_UART)
        return false;
    if (!_uart_configured[uart_n])
        return false;

    // Ignore if receive buffer is empty
    if (!is_uart_rx_data_available(uart_n))
        return false;

    // Read byte from receive buffer
    *read_byte = UDR0;
    return true;
}

/**
 * @details
 * This function reads data from UART receive buffer until it is empty.
 */
bool AvrUart::flush_rx(const uint8_t uart_n)
{
    uint8_t read_byte = 0xFF;

    // Ignore if invalid UART number provided or UART is not configured yet
    if (uart_n > LAST_UART)
        return false;
    if (!_uart_configured[uart_n])
        return false;

    // While there is data in receive buffer, read it
    while (is_uart_rx_data_available(uart_n))
        read_byte = UDR0;
    return read_byte;
}

/**
 * @details
 * This function checks the corresponding flag of data received and available
 * to be read from receive buffer of UART from the corresponding register.
 */
bool AvrUart::is_uart_rx_data_available(const uint8_t uart_n)
{
    // Ignore if invalid UART number provided or UART is not configured yet
    if (uart_n > LAST_UART)
        return false;
    if (!_uart_configured[uart_n])
        return false;

    return (UCSR0A & (1 << RXC0));
}

/**
 * @details
 * This function checks the corresponding flag of register regarding data
 * currently being transmitted, so transmission buffer of UART is busy and
 * can't be used right now.
 */
bool AvrUart::is_uart_tx_buffer_available(const uint8_t uart_n)
{
    // Ignore if invalid UART number provided or UART is not configured yet
    if (uart_n > LAST_UART)
        return false;
    if (!_uart_configured[uart_n])
        return false;

    return (UCSR0A & (1 << UDRE0));
}

/*****************************************************************************/

/* Private Methods */

/*****************************************************************************/

#endif /* defined(__AVR_ATmega48__) || ... || defined(__AVR_ATmega328P__) */
