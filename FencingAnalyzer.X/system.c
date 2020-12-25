/*
 * Fencing Action Analyzer
 * Copyright (c) 2020 David Rice
 * 
 * System operations and initialization routines
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

#include "system.h"

/* Initialize oscillator module for SYSCLK = 16 MHz
 *  
 * This should check CLKSTAT at appropriate points to ensure that components
 * are stable but the CLKSTAT register does not appear to work 
 * on the PIC32MM
 */
void init_osc(void) {
    /* Unlock system registers */
    unlock_sysreg();
    
    /* Set PLL multiplication factor 4 and division factor 2 
     * With FRC of 8 MHz this results in SYSCLK = (8 * 4) / 2 = 16 MHz */
    SPLLCONbits.PLLODIV = 0b001;
    SPLLCONbits.PLLMULT = 0b0000010;
    
    /* Switch SYSCLK to system PLL */
    OSCCONbits.NOSC = 0b001;
    
    /* Initialize oscillator switch */
    OSCCONbits.OSWEN = 1;
    
    /* Wait for oscillator switch to complete */
    while (OSCCONbits.OSWEN);
    
    /* Lock system registers */
    lock_sysreg();
}