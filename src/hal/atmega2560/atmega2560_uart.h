
/**
 * @file    atmega2560_uart.h
 * @author  Jose Miguel Rios Rubio <jrios.github@gmail.com>
 * @date    02-02-2022
 * @version 1.1.1
 *
 * @section DESCRIPTION
 *
 * UART driver interface for the following AVR devices:
 * ATmega640/1280/1281/2560/2561
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

#if defined(__AVR_ATmega640__)   || defined(__AVR_ATmega1280__)  || \
    defined(__AVR_ATmega1281__)  || defined(__AVR_ATmega2560__)  || \
    defined(__AVR_ATmega2561__)

#ifndef ATMEGA2560_UART_H_
#define ATMEGA2560_UART_H_

/*****************************************************************************/

/* Libraries */

// Standard C/C++ libraries
#include <stdint.h>
#include <stdbool.h>

// Device/Framework Headers
#include <avr/interrupt.h>

/*****************************************************************************/

/* Driver Configuration Options */

// Buffer size to store received UART data (bytes)
// Modify this according to your project requirement
#if !defined(AVRUART_RX_BUFFER_SIZE)
    #define AVRUART_RX_BUFFER_SIZE 64
#endif

/*****************************************************************************/

/* Constants & Defines */

// Number of UARTs on this device (UART0, UART1, UART2, UART3)
#define AVR_NUM_UARTS 4

// UARTs number defines
#define UART0 0
#define UART1 1
#define UART2 2
#define UART3 3

/*****************************************************************************/

/* Data Types */

/*****************************************************************************/

/* Extern ISRs Definitions */

extern "C"
{
    /* Check ISR(X) Macro definition */

    // USART0 data received ISR
	void USART0_RX_vect(void) __attribute__ ((signal));

    // USART1 data received ISR
	void USART1_RX_vect(void) __attribute__ ((signal));

    // USART2 data received ISR
	void USART2_RX_vect(void) __attribute__ ((signal));

    // USART3 data received ISR
	void USART3_RX_vect(void) __attribute__ ((signal));
};

/*****************************************************************************/

/* Class Interface */

class AvrUart
{
    /**
     * @brief  Set USART0 receive data Interrupt Service Rutine as a friend
     * function that can access AvrUart private elements.
     */
    friend void USART0_RX_vect(void);

    /**
     * @brief  Set USART1 receive data Interrupt Service Rutine as a friend
     * function that can access AvrUart private elements.
     */
    friend void USART1_RX_vect(void);

    /**
     * @brief  Set USART2 receive data Interrupt Service Rutine as a friend
     * function that can access AvrUart private elements.
     */
    friend void USART2_RX_vect(void);

    /**
     * @brief  Set USART3 receive data Interrupt Service Rutine as a friend
     * function that can access AvrUart private elements.
     */
    friend void USART3_RX_vect(void);

    public:

        /**
         * @brief  Constructor. Takes UART number to use.
         * @param  uart_num UART number to use (0 - UART0; 1 - UART1...).
         * @param  freq_cpu CPU Frequency (same as Fosc).
         */
        AvrUart(const uint8_t uart_num, const uint32_t freq_cpu);

        /**
         * @brief  Destructor.
         */
        ~AvrUart();

        /**
         * @brief  Setup the UART.
         * @param  baud_rate UART communication speed.
         */
        bool setup(const uint32_t baud_rate,
                const bool internal_res_pullup=true);

        /**
         * @brief  UART send a byte.
         * @param  write_byte Byte of data to send.
         */
        bool write(const uint8_t write_byte);

        /**
         * @brief  UART read received byte.
         * @param  read_byte Pointer to store received byte.
         * @return Read result (success - true; fail - false).
         */
        bool read(uint8_t* read_byte);

        /**
         * @brief  Clear UART receive buffer.
         */
        bool flush_rx();

        /**
         * @brief  Get the number of bytes received and available to be read
         * from UART received data buffer.
         * @return The number of bytes available to be read from RX buffer.
         */
        uint16_t num_rx_data_available();

    private:

        uint32_t f_cpu;
        uint8_t uart_n;
        bool uart_configured;
        volatile uint16_t rx_buffer_head;
        volatile uint16_t rx_buffer_tail;
        volatile uint8_t rx_buffer[AVRUART_RX_BUFFER_SIZE];

        /**
         * @brief  Check if there is any data available to be read from UART
         * receive buffer.
         * @return If there is data available to bead (true or false).
         */
        bool is_uart_rx_data_available();

        /**
         * @brief  Check if transmission buffer is available to transmit data.
         * @return If data can be send (true) or transmission is currently
         * busy (false).
         */
        bool is_uart_tx_buffer_available();
};

/*****************************************************************************/

#endif /* ATMEGA2560_UART_H_ */

#endif /* defined(__AVR_ATmega640__) || ... || defined(__AVR_ATmega2561__) */
