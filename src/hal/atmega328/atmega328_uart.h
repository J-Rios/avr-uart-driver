
/**
 * @file    atmega328_uart.h
 * @author  Jose Miguel Rios Rubio <jrios.github@gmail.com>
 * @date    26-01-2022
 * @version 1.0.0
 *
 * @section DESCRIPTION
 *
 * UART driver interface for the following AVR devices:
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

/* Include Guard */

#ifndef ATMEGA328_UART_H_
#define ATMEGA328_UART_H_

/*****************************************************************************/

/* Libraries */

// Standard C/C++ libraries
#include <stdint.h>
#include <stdbool.h>

/*****************************************************************************/

/* Constants & Defines */

// Number of UARTs on this device (just 1, UART0)
#define AVR_NUM_UARTS 1

// UARTs number defines
#define UART0 0

/*****************************************************************************/

/* Data Types */

/*****************************************************************************/

/* Class Interface */

class AvrUart
{
    public:

        /**
         * @brief  Constructor. Takes UART number to use.
         * @param  f_cpu CPU Frequency (same as Fosc).
         */
        AvrUart(const uint32_t f_cpu);

        /**
         * @brief  Destructor.
         */
        ~AvrUart();

        /**
         * @brief  Setup the UART.
         * @param  uart_n UART number to use (0 - UART0; 1 - UART1...).
         * @param  baud_rate UART communication speed.
         */
        bool setup(const uint8_t uart_n, const uint32_t baud_rate,
                const bool internal_res_pullup=true);

        /**
         * @brief  UART send a byte.
         * @param  uart_n UART number to use (0 - UART0; 1 - UART1...).
         * @param  write_byte Byte of data to send.
         */
        bool write(const uint8_t uart_n, const uint8_t write_byte);

        /**
         * @brief  UART read received byte.
         * @param  uart_n UART number to use (0 - UART0; 1 - UART1...).
         * @param  read_byte Pointer to store received byte.
         * @return Read result (success - true; fail - false).
         */
        bool read(const uint8_t uart_n, uint8_t* read_byte);

        /**
         * @brief  Clear UART receive buffer.
         * @param  uart_n UART number to use (0 - UART0; 1 - UART1...).
         */
        bool flush_rx(const uint8_t uart_n);

        /**
         * @brief  Check if there is any data available to be read from UART
         * receive buffer.
         * @param  uart_n UART number to use (0 - UART0; 1 - UART1...).
         * @return If there is data available to bead (true or false).
         */
        bool is_uart_rx_data_available(const uint8_t uart_n);

        /**
         * @brief  Check if transmission buffer is available to transmit data.
         * @param  uart_n UART number to use (0 - UART0; 1 - UART1...).
         * @return If data can be send (true) or transmission is currently
         * busy (false).
         */
        bool is_uart_tx_buffer_available(const uint8_t uart_n);

    private:

        uint32_t _f_cpu;
        bool _uart_configured[AVR_NUM_UARTS];
};

/*****************************************************************************/

#endif /* ATMEGA328_UART_H_ */
