/*
 * Fencing Action Analyzer
 * Copyright (c) 2020 David Rice
 * 
 * Generic routines related to non-volatile memory access
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

#include <sys/kmem.h>

#include <stdint.h>

#include "nvm.h"

void erase_nvm_page(uint32_t addr) {
    /* Set NVM program mode to page erase */
    NVMCONbits.NVMOP = 0b0100;
    
    /* Convert virtual page address to physical address */
    NVMADDR = KVA_TO_PA(addr);
    
    /* Enable NVM writes */
    NVMCONbits.WREN = 1;
    
    /* Unlock NVM */
    unlock_nvm();
    
    /* Trigger NVM write */
    NVMCONSET = _NVMCON_WR_MASK;
    
    /* Wait for write to complete */
    while (NVMCONbits.WR);
    
    /* Disable NVM writes */
    NVMCONbits.WREN = 0;
    
    /* Set NVM program to no-op */
    NVMCONbits.NVMOP = 0b0000;
}

void write_nvm_dword(uint32_t addr, uint32_t word1, uint32_t word2) {
    /* Set NVM program mode to double-word write */
    NVMCONbits.NVMOP = 0b0010;
    
    /* Convert virtual page address to physical address */
    NVMADDR = KVA_TO_PA(addr);
    
    NVMDATA0 = word1;
    NVMDATA1 = word2;
    
    /* Enable NVM writes */
    NVMCONbits.WREN = 1;
    
    /* Unlock NVM */
    unlock_nvm();
    
    /* Trigger NVM write */
    NVMCONSET = _NVMCON_WR_MASK;
    
    /* Wait for write to complete */
    while (NVMCONbits.WR);
    
    /* Disable NVM writes */
    NVMCONbits.WREN = 0;
    
    /* Set NVM program to no-op */
    NVMCONbits.NVMOP = 0b0000;
}