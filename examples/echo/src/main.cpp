
/**
 * @file    main.cpp
 * @author  Jose Miguel Rios Rubio <jrios.github@gmail.com>
 * @date    26-01-2022
 * @version 1.0.0
 *
 * @section DESCRIPTION
 *
 * Echo example main file of AVR-UART Driver.
 *
 * @section LICENSE
 *
 * Copyright (c) 2021 Jose Miguel Rios Rubio. All right reserved.
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

/* Libraries */

// Standard Headers (libc)
#include <inttypes.h>

// Project Headers
#include "constants.h"
#include "avr_uart.h"

/*****************************************************************************/

/* Main Function */

int main(void)
{
    AvrUart Serial(F_CPU);

    // Initilize UART0
    Serial.setup(UART_NUM, UART_BAUD_RATE);

    while (1)
    {
        // Echo received data from UART
        uint8_t read_byte = 0xFF;
        if (Serial.read(UART_NUM, &read_byte))
            Serial.write(UART_NUM, read_byte);
    }
}

/*****************************************************************************/
