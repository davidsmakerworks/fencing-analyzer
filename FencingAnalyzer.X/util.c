/*
 * Fencing Action Analyzer
 * Copyright (c) 2020 David Rice
 * 
 * Generic utility functions
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
#include "util.h"

/* Delay for specified number of milliseconds based on CP0 core timer */
void delay_ms(uint32_t delay) {
    uint32_t start;
    uint32_t cycles;
    
    start = (uint32_t)_CP0_GET_COUNT();
    cycles = ((CPU_FREQ / 2) / 1000) * delay;
    
    while(((uint32_t)_CP0_GET_COUNT() - start) < cycles);
}

