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

#ifndef SYSTEM_H
#define	SYSTEM_H

#include <xc.h>

#ifdef	__cplusplus
extern "C" {
#endif

#define CPU_FREQ    16000000

/* Unlocks system registers - interrupts should be disabled before calling this function */
static inline __attribute((always_inline)) void unlock_sysreg(void) {
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;
};

/* Locks system registers */
static inline __attribute((always_inline)) void lock_sysreg(void) {
    SYSKEY = 0x00000000;
};

/* Initialize oscillator module */
void init_osc(void);

#ifdef	__cplusplus
}
#endif

#endif	/* SYSTEM_H */

