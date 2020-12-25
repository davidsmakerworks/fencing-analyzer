/*
 * Fencing Action Analyzer
 * Copyright (c) 2020 David Rice
 * 
 * Routines for initializing and working with peripherals
 * 
 * SET/CLR/INV registers are used when appropriate to avoid
 * R-M-W hazards on MIPS architecture, but "bits" structures
 * are used for readability when no R-M-W hazard exists.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef PERIPHERALS_H
#define	PERIPHERALS_H

#ifdef	__cplusplus
extern "C" {
#endif

/* Initialize I/O ports */
void init_ports(void);

/* Initialize PPS pin settings */
void init_pps(void);

/* Initialize SPI2 for inertial module */
void init_spi2(void);

/* Initialize UART2 for Bluetooth data */
void init_uart2(void);

/* Transfer one 8-bit value on SPI2 */
uint8_t spi2_transfer(uint8_t data);

/* Transmit one 8-bit value on UART2 */
void uart2_transmit(uint8_t data);

/* Receive one 8-bit value from UART2 */
uint8_t uart2_get_byte(void);

#ifdef	__cplusplus
}
#endif

#endif	/* PERIPHERALS_H */

