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

#include <xc.h>

#include <stdint.h>

#include "system.h"
#include "peripherals.h"

/* Initialize I/O ports */
void init_ports(void) {
    /* Disable analog on all ports */
    ANSELA = 0x00000000;
    ANSELB = 0x00000000;
    ANSELC = 0x00000000;
    
    /* Set all output latches low except:
     * RA1/RP2 - U2TX
     * RB10/RP17 - SCK2OUT
     * RB13 - CSN */
    LATA = _LATA_LATA1_MASK;
    LATB = _LATB_LATB10_MASK |
           _LATB_LATB13_MASK;
    LATC = 0x00000000;
    
    /* Set all ports to output except: 
     * RA0/RP1 - U2RX
     * RB12/RP12 - SDI2 
     * RB14 - INT2 
     * RB15 - INT1 */
    TRISA = _TRISA_TRISA0_MASK;
    TRISB = _TRISB_TRISB12_MASK |
            _TRISB_TRISB14_MASK |
            _TRISB_TRISB15_MASK;
    TRISC = 0x00000000;
}

/* Initialize PPS pin settings */
void init_pps(void) {
    /* Unlock system registers */
    unlock_sysreg();
    
    /* UART2 RX on RP1 */
    RPINR9bits.U2RXR = 1;
    
    /* UART2 TX on RP2 */
    RPOR0bits.RP2R = 1;
    
    /* SCK2OUT on RP17 */
    RPOR4bits.RP17R = 4;
    
    /* SDO2 on RP18 */
    RPOR4bits.RP18R = 3;
    
    /* SDI2 on RP12 */
    RPINR11bits.SDI2R = 12;
    
    /* Lock system registers */
    lock_sysreg();
}

/* Initialize SPI2 for inertial module */
void init_spi2(void) {
    /* Set SPI2 master mode */
    SPI2CONbits.MSTEN = 1;
    
    /* SPI2 clock is idle-high, active-low */
    SPI2CONbits.CKP = 1;
    
    /* SPI2 samples at middle of data output time */
    SPI2CONbits.SMP = 0;
    
    /* SPI2 output data changes on idle-to-active transition */
    SPI2CONbits.CKE = 0;
    
    /* Fsck = PBCLK / (2 * (SPI2BRG + 1)) = 1 MHz when PBCLK = 16 MHz */
    SPI2BRG = 7;
    
    /* Activate SPI2 module */
    SPI2CONbits.ON = 1;
}

/* Initialize UART2 for Bluetooth data */
void init_uart2(void) {
    /* UART2 clock source is PBCLK */
    U2MODEbits.CLKSEL = 0b00;
    
    /* Enable high baud rate mode */
    U2MODEbits.BRGH = 1;
    
    /* Set 8 data bits, no parity */
    U2MODEbits.PDSEL = 0b00;
    
    /* Set 1 stop bit */
    U2MODEbits.STSEL = 1;

    /* Baud Rate = PBCLK / (4 * U2BRG + 1)) = 57971 at PBCLK = 16 MHz 
     * Closest standard baud rate: 57600 */
    U2BRG = 69;
    
    U2STAbits.UTXEN = 1;
    U2STAbits.URXEN = 1;
    
    /* Activate UART2 module */
    U2MODEbits.ON = 1;
}

/* Transfer one 8-bit value on SPI2 */
uint8_t spi2_transfer(uint8_t data) {
    /* Block if buffer is full */
    while (SPI2STATbits.SPITBF);
    
    SPI2BUF = data;
    
    /* Spin until transfer is complete */
    while (!SPI2STATbits.SPIRBF);
    
    data = SPI2BUF;
    
    return data;
}

/* Transmit one 8-bit value on UART2 */
void uart2_transmit(uint8_t data) {
    /* Block if buffer is full */
    while (U2STAbits.UTXBF);
    
    U2TXREG = data;  
}

/* Receive one 8-bit value from UART2 */
uint8_t uart2_get_byte(void) {
    uint8_t data;
    
    data = U2RXREG;
    
    return data;
}
