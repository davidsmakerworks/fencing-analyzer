/*
 * Fencing Action Analyzer
 * Copyright (c) 2019 David Rice
 * 
 * Main source file for fencing action analyzer
 * Currently implements only transmission fo raw IMU data
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

// PIC32MM0064GPL028 Configuration Bit Settings

// 'C' source line config statements

// FDEVOPT
#pragma config SOSCHP = OFF             // Secondary Oscillator High Power Enable bit (SOSC oprerates in normal power mode.)
#pragma config USERID = 0xFFFF          // User ID bits (User ID bits)

// FICD
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)
#pragma config ICS = PGx1               // ICE/ICD Communication Channel Selection bits (Communicate on PGEC1/PGED1)

// FPOR
#pragma config BOREN = BOR3             // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware; SBOREN bit disabled)
#pragma config RETVR = OFF              // Retention Voltage Regulator Enable bit (Retention regulator is disabled)
#pragma config LPBOREN = ON             // Low Power Brown-out Enable bit (Low power BOR is enabled, when main BOR is disabled)

// FWDT
#pragma config SWDTPS = PS1048576       // Sleep Mode Watchdog Timer Postscale Selection bits (1:1048576)
#pragma config FWDTWINSZ = PS25_0       // Watchdog Timer Window Size bits (Watchdog timer window size is 25%)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Watchdog timer is in non-window mode)
#pragma config RWDTPS = PS1048576       // Run Mode Watchdog Timer Postscale Selection bits (1:1048576)
#pragma config RCLKSEL = LPRC           // Run Mode Watchdog Timer Clock Source Selection bits (Clock source is LPRC (same as for sleep mode))
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (WDT is disabled)

// FOSCSEL
#pragma config FNOSC = FRCDIV           // Oscillator Selection bits (Fast RC oscillator (FRC) with divide-by-N)
#pragma config PLLSRC = FRC             // System PLL Input Clock Selection bit (FRC oscillator is selected as PLL reference input on device reset)
#pragma config SOSCEN = OFF             // Secondary Oscillator Enable bit (Secondary oscillator (SOSC) is disabled)
#pragma config IESO = ON                // Two Speed Startup Enable bit (Two speed startup is enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Selection bit (Primary oscillator is disabled)
#pragma config OSCIOFNC = OFF           // System Clock on CLKO Pin Enable bit (OSCO pin operates as a normal I/O)
#pragma config SOSCSEL = OFF            // Secondary Oscillator External Clock Enable bit (Crystal is used (RA4 and RB4 are controlled by SOSC))
#pragma config FCKSM = CSECME           // Clock Switching and Fail-Safe Clock Monitor Enable bits (Clock switching is enabled; Fail-safe clock monitor is enabled)

// FSEC
#pragma config CP = OFF                 // Code Protection Enable bit (Code protection is disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <sys/attribs.h>
#include <sys/kmem.h>

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "lsm6ds3x.h"
#include "lsm6ds3x-cfg.h"

#define CPU_FREQ                16000000
#define FIRMWARE_VERSION        "2.1"

#define MAX_AVG_WINDOW_SIZE     100

/* kseg0 virtual address for configuration page in flash */
#define CONFIG_PAGE_VIRT_ADDR   0x9D00F800UL

#define MODE_RUN        0
#define MODE_CONFIG     1

#define MENU_MAIN           0
#define MENU_XL_SCALE       1
#define MENU_G_SCALE        2
#define MENU_AVG_WINDOW     3
#define MENU_ACTIVE_AXES    4

#define XLX_ACTIVE_MASK (1 << 0) 
#define XLY_ACTIVE_MASK (1 << 1)
#define XLZ_ACTIVE_MASK (1 << 2)
#define GX_ACTIVE_MASK  (1 << 3)
#define GY_ACTIVE_MASK  (1 << 4)
#define GZ_ACTIVE_MASK  (1 << 5)

typedef struct {
    uint8_t fs_xl;
    uint8_t fs_g;
    uint8_t avg_window;
    uint8_t axis_mask;
    bool invert_xl_z;
} CONFIG_DATA;

/* Allocate last page of program memory for storage of config information */
const uint32_t __attribute((space(prog), section(".nvmconfig"), address(CONFIG_PAGE_VIRT_ADDR))) reserved_flash_space[512];

/* Handle processor exceptions */
void _general_exception_handler(void) {
    uint32_t exc_code;
    uint32_t exc_addr;
    
    exc_code = (_CP0_GET_CAUSE() & _CP0_CAUSE_EXCCODE_MASK) >> _CP0_CAUSE_EXCCODE_POSITION;
    exc_addr = _CP0_GET_EPC();
    
    printf("\r\n\r\n*** MIPS CORE EXCEPTION ***\r\n\r\n");
    printf("Exception code 0x%02X at 0x%08X\r\n", exc_code, exc_addr);
    printf("System halted\r\n");
    
    while(1);
}

/* Unlocks system registers - interrupts should be disabled before calling this function */
inline __attribute((always_inline)) void unlock_sysreg(void) {
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;
}

/* Locks system registers */
inline __attribute((always_inline)) void lock_sysreg(void) {
    SYSKEY = 0x00000000;
}

/* Unlocks NVM writes - interrupts should be disabled before calling this function */
inline __attribute((always_inline)) void unlock_nvm(void) {
    NVMKEY = 0xAA996655;
    NVMKEY = 0x556699AA;
}

/* Delay for specified number of milliseconds based on 
 * CP0 core timer */
void delay_ms(uint32_t delay) {
    uint32_t start;
    uint32_t cycles;
    
    start = (uint32_t)_CP0_GET_COUNT();
    cycles = ((CPU_FREQ / 2) / 1000) * delay;
    
    while(((uint32_t)_CP0_GET_COUNT() - start) < cycles);
}

/* Initialize oscillator module for SYSCLK = 16 MHz
 *  
 * This should check CLKSTAT at appropriate
 * points to ensure that components are stable
 * but the CLKSTAT register does not appear
 * to work on the PIC32MM */
void init_osc(void) {
    /* Unlock system registers */
    unlock_sysreg();
    
    /* Set PLL multiplication factor 4 and division factor 2 
     * With FRC of 8 MHz this results in SYSCLK = (8 * 4) / 2 = 16 MHz */
    SPLLCONbits.PLLODIV = 0b001;
    SPLLCONbits.PLLMULT = 0b0000010;
    
    /* Switch SYSCLK to system PLL */
    OSCCONbits.NOSC = 0b001;
    
    /* Initialite oscillator switch */
    OSCCONbits.OSWEN = 1;
    
    /* Wait for oscillator switch to complete */
    while (OSCCONbits.OSWEN);
    
    /* Lock system registers */
    lock_sysreg();
}

/* Inititlaize I/O ports */
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

/* Initiailize inertial module */
void init_im(CONFIG_DATA *config) {
    /* Enable accelerometer data-ready interrupt on INT1 and gyro data-ready interrupt on INT2 */
    lsm6d_set_register_bits(LSM6D_INT1_CTRL, 1 << _INT1_CTRL_INT1_DRDY_XL_POSN);
    lsm6d_set_register_bits(LSM6D_INT2_CTRL, 1 << _INT2_CTRL_INT2_DRDY_G_POSN);
    
    /* Enable block data update just in case it is needed */
    lsm6d_set_register_bits(LSM6D_CTRL3_C, 1 << _CTRL3_C_BDU_POSN);
    
    /* Set accelerometer and gyro data date to 104 samples/sec*/
    lsm6d_set_accel_data_rate(_CTRL1_XL_ODR_XL_104HZ);
    lsm6d_set_gyro_data_rate(_CTRL2_G_ODR_G_104HZ);
    
    /* Set accelerometer full-scale value */
    lsm6d_set_accel_scale(config->fs_xl);
    
    /* Set gyro full-scale value */
    lsm6d_set_gyro_scale(config->fs_g);
}

/* Read packed config data from NVM */
void get_nvm_config_data(CONFIG_DATA *config) {
    uint8_t *config_data;
    
    config_data = (uint8_t *)(CONFIG_PAGE_VIRT_ADDR);
    
    config->fs_xl = config_data[0];
    config->fs_g = config_data[1];
    config->avg_window = config_data[2];
    config->invert_xl_z = config_data[3];
}

/* Initialize all applicable system peripherals */
void init_system(void) {
    init_osc();
    init_ports();
    init_pps();
    init_spi2();
    init_uart2();
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

uint8_t uart2_get_byte(void) {
    uint8_t data;
    
    data = U2RXREG;
    
    return data;
}

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

/* Write packed configuration data to NVM */
void write_nvm_config_data(CONFIG_DATA *config) {
    uint32_t config_word1;
    uint32_t config_word2;
    
    config_word1 = 0;
    config_word2 = 0;
    
    /* Erase config page before writing new data */
    erase_nvm_page(CONFIG_PAGE_VIRT_ADDR);
    
    /* Pack config data into 32-bit words */
    config_word1 = config->fs_xl;
    config_word1 |= config->fs_g << 8;
    config_word1 |= config->avg_window << 16;
    config_word1 |= config->invert_xl_z << 24;
       
    /* Write two config words to NVM */
    write_nvm_dword(CONFIG_PAGE_VIRT_ADDR, config_word1, config_word2);
}

uint8_t get_next_fs_xl(uint8_t current) {
    switch (current) {
        case _CTRL1_XL_FS_XL_2G:
            return _CTRL1_XL_FS_XL_4G;
        case _CTRL1_XL_FS_XL_4G:
            return _CTRL1_XL_FS_XL_8G;
        case _CTRL1_XL_FS_XL_8G:
            return _CTRL1_XL_FS_XL_16G;
        case _CTRL1_XL_FS_XL_16G:
            return _CTRL1_XL_FS_XL_2G;
        default:
            return _CTRL1_XL_FS_XL_2G;
    }
}

uint8_t get_next_fs_g(uint8_t current) {
    switch (current) {
        case _CTRL2_G_FS_G_245DPS:
            return _CTRL2_G_FS_G_500DPS;
        case _CTRL2_G_FS_G_500DPS:
            return _CTRL2_G_FS_G_1000DPS;
        case _CTRL2_G_FS_G_1000DPS:
            return _CTRL2_G_FS_G_2000DPS;
        case _CTRL2_G_FS_G_2000DPS:
            return _CTRL2_G_FS_G_245DPS;
        default:
            return _CTRL2_G_FS_G_245DPS;
    }
}

void run_config_menu(CONFIG_DATA *config) {
    bool done = false;
    uint8_t cmd;
    uint8_t menu_state = MENU_MAIN;
    
    printf("\r\n\r\n\r\n\r\n\r\n\r\n");
    printf("+----------------------------------+\r\n");
    printf("| WIRELESS FENCING ACTION ANALYZER |\r\n");
    printf("|   Designed and implemented by    |\r\n");
    printf("|            David Rice            |\r\n");
    printf("|   Phoenix Falcons Fencing Club   |\r\n");
    printf("+----------------------------------+\r\n");
    printf("\r\n");
    printf("CPU: PIC32MM0064GPL028\r\n");
    printf("IMU: LSM6DS33\r\n");
    printf("FW:  v%s\r\n", FIRMWARE_VERSION);
    printf("\r\n");
    printf("Entering configuration mode...\r\n");
    printf("\r\n");
    
    while (!done) {
        switch (menu_state) {
            case MENU_MAIN:
                printf("Main menu\r\n");
                printf("---------\r\n");
                printf("\r\n");
                printf("1) Accelerometer full scale: ");
                switch (config->fs_xl) {
                    case _CTRL1_XL_FS_XL_2G:
                        printf("2G");
                        break;
                    case _CTRL1_XL_FS_XL_4G:
                        printf("4G");
                        break;
                    case _CTRL1_XL_FS_XL_8G:
                        printf("8G");
                        break;
                    case _CTRL1_XL_FS_XL_16G:
                        printf("16G");
                        break;
                    default:
                        printf("INVALID");
                }

                printf("\r\n");

                printf("2) Gyroscope full scale: ");
                switch (config->fs_g) {
                    case _CTRL2_G_FS_G_245DPS:
                        printf("245 deg/sec");
                        break;
                    case _CTRL2_G_FS_G_500DPS:
                        printf("500 deg/sec");
                        break;
                    case _CTRL2_G_FS_G_1000DPS:
                        printf("1000 deg/sec");
                        break;
                    case _CTRL2_G_FS_G_2000DPS:
                        printf("2000 deg/sec");
                        break;
                    default:
                        printf("INVALID");
                }

                printf("\r\n");

                printf("3) Average window size: %d\r\n", config->avg_window);
                printf("4) Invert accelerometer Z-axis: %s\r\n", config->invert_xl_z ? "Yes" : "No");
                printf("S) Save to NVM and exit\r\n");
                printf("X) Exit without saving\r\n");
                printf("\r\n");
                printf("Choose an option: ");

                /* Wait for command to be entered */
                while (!U2STAbits.URXDA);

                cmd = uart2_get_byte();

                /* Convert letters to upper case */
                if (cmd >= 'a') {
                    cmd -= 32;
                }

                printf("%c\r\n\r\n", cmd);

                switch(cmd) {
                    case '1':
                        //config->fs_xl = get_next_fs_xl(config->fs_xl);
                        menu_state = MENU_XL_SCALE;
                        break;
                    case '2':
                        //config->fs_g = get_next_fs_g(config->fs_g);
                        menu_state = MENU_G_SCALE;
                        break;
                    case '3':
                        config->avg_window += 5;

                        if (config->avg_window > MAX_AVG_WINDOW_SIZE) {
                            config->avg_window = 0;
                        }
                        // menu_state = MENU_AVG_WINDOW;
                        break;
                    case '4':
                        config->invert_xl_z = !config->invert_xl_z;
                        break;
                    case 'S':
                        printf("Writing configuration to NVM...\r\n");
                        write_nvm_config_data(config);
                        /* Fall through from 'S' to 'X' is expected */
                    case 'X':
                        printf("Exiting...\r\n\r\n");
                        delay_ms(2000);
                        done = true;
                        break;
                    default:
                        printf("Invalid selection!\r\n\r\n");
                }
                break;
            case MENU_XL_SCALE:
                printf("Accelerometer full scale\r\n");
                printf("------------------------\r\n");
                printf("\r\n");
                printf("1) 2G\r\n");
                printf("2) 4G\r\n");
                printf("3) 8G\r\n");
                printf("4) 16G\r\n");
                printf("\r\n");
                printf("Choose an option: ");
                
                /* Wait for command to be entered */
                while (!U2STAbits.URXDA);

                cmd = uart2_get_byte();
                
                printf("%c\r\n\r\n", cmd);
                
                switch(cmd) {
                    case '1':
                        config->fs_xl = _CTRL1_XL_FS_XL_2G;
                        menu_state = MENU_MAIN;
                        break;
                    case '2':
                        config->fs_xl = _CTRL1_XL_FS_XL_4G;
                        menu_state = MENU_MAIN;
                        break;
                    case '3':
                        config->fs_xl = _CTRL1_XL_FS_XL_8G;
                        menu_state = MENU_MAIN;
                        break;
                    case '4':
                        config->fs_xl = _CTRL1_XL_FS_XL_16G;
                        menu_state = MENU_MAIN;
                        break;
                    default:
                        printf("Invalid selection!\r\n\r\n");
                }
                break;
                
            case MENU_G_SCALE:
                printf("Gyroscope full scale\r\n");
                printf("------------------------\r\n");
                printf("\r\n");
                printf("1) 245 deg/sec\r\n");
                printf("2) 500 deg/sec\r\n");
                printf("3) 1000 deg/sec\r\n");
                printf("4) 2000 deg/sec\r\n");
                printf("\r\n");
                printf("Choose an option: ");
                
                /* Wait for command to be entered */
                while (!U2STAbits.URXDA);

                cmd = uart2_get_byte();
                
                printf("%c\r\n\r\n", cmd);
                
                switch(cmd) {
                    case '1':
                        config->fs_g = _CTRL2_G_FS_G_245DPS;
                        menu_state = MENU_MAIN;
                        break;
                    case '2':
                        config->fs_g = _CTRL2_G_FS_G_500DPS;
                        menu_state = MENU_MAIN;
                        break;
                    case '3':
                        config->fs_g = _CTRL2_G_FS_G_1000DPS;
                        menu_state = MENU_MAIN;
                        break;
                    case '4':
                        config->fs_g = _CTRL2_G_FS_G_2000DPS;
                        menu_state = MENU_MAIN;
                        break;
                    default:
                        printf("Invalid selection!\r\n\r\n");
                }
                break;
        }       
    }   
}

void clear_data(LSM6D_SENSOR_DATA *samples) {
    uint8_t index;
    
    for (index = 0; index < MAX_AVG_WINDOW_SIZE; index++) {
        samples[index].temp = 0;

        samples[index].xl.x = 0;
        samples[index].xl.y = 0;
        samples[index].xl.z = 0;

        samples[index].g.x = 0;
        samples[index].g.y = 0;
        samples[index].g.z = 0;
    }
}

void main(void) {
    LSM6D_SENSOR_DATA samples[MAX_AVG_WINDOW_SIZE];
    CONFIG_DATA config;
    
    uint8_t index;
    uint8_t serial_data;
    
    uint8_t state;
    
    int32_t xl_x_sum;
    int32_t xl_y_sum;
    int32_t xl_z_sum;
    
    int32_t g_x_sum;
    int32_t g_y_sum;
    int32_t g_z_sum;
    
    int32_t xl_x_avg;
    int32_t xl_y_avg;
    int32_t xl_z_avg;
    
    int32_t g_x_avg;
    int32_t g_y_avg;
    int32_t g_z_avg;
    
    /* Output from printf is sent to UART2 */
    __XC_UART = 2;
    
    /* Disable interrupts just in case - interrupts are not used at this time */
    __builtin_disable_interrupts();
    
    /* Initialize on-board peripherals */
    init_system();
    
    /* Read configuration values from NVM */
    get_nvm_config_data(&config);
    
    /* Initialize sample array */
    clear_data(samples);
    
    /* Brief delay to ensure that IMU is ready */
    delay_ms(50);
    
    /* Initialize IMU with config values */
    init_im(&config); 
    
    /* Initialize moving average index and sums */
    index = 0;
    
    xl_x_sum = 0;
    xl_y_sum = 0;
    xl_z_sum = 0;
    
    g_x_sum = 0;
    g_y_sum = 0;
    g_z_sum = 0;
    
    /* Start in run mode */
    state = MODE_RUN;
    
    while (1) {
        if (state == MODE_RUN) {
            /* Wait for valid data */
            while (!(LSM6D_INT1 && LSM6D_INT2));

            if (config.avg_window > 0) {
                xl_x_sum -= samples[index].xl.x;
                xl_y_sum -= samples[index].xl.y;
                xl_z_sum -= samples[index].xl.z;

                g_x_sum -= samples[index].g.x;
                g_y_sum -= samples[index].g.y;
                g_z_sum -= samples[index].g.z;

                lsm6d_get_all_motion_data(&samples[index]);

                xl_x_sum += samples[index].xl.x;
                xl_y_sum += samples[index].xl.y;
                xl_z_sum += samples[index].xl.z;

                g_x_sum += samples[index].g.x;
                g_y_sum += samples[index].g.y;
                g_z_sum += samples[index].g.z;

                index = (index < (config.avg_window - 1)) ? (index + 1) : 0;

                xl_x_avg = xl_x_sum / config.avg_window;
                xl_y_avg = xl_y_sum / config.avg_window;
                xl_z_avg = xl_z_sum / config.avg_window;

                g_x_avg = g_x_sum / config.avg_window;
                g_y_avg = g_y_sum / config.avg_window;
                g_z_avg = g_z_sum / config.avg_window;
            } else {
                lsm6d_get_all_motion_data(&samples[0]);
                
                xl_x_avg = samples[0].xl.x;
                xl_y_avg = samples[0].xl.y;
                xl_z_avg = samples[0].xl.z;

                g_x_avg = samples[0].g.x;
                g_y_avg = samples[0].g.y;
                g_z_avg = samples[0].g.z;
            }

            printf("%d,%d,%d,%d,%d,%d\r\n", xl_x_avg, xl_y_avg, 
                    config.invert_xl_z ? -xl_z_avg: xl_z_avg, g_x_avg, g_y_avg, g_z_avg);
            
            if (U2STAbits.URXDA) {
                serial_data = uart2_get_byte();
                
                if (serial_data == 27) {
                    state = MODE_CONFIG;
                }
            }
        } else {           
            run_config_menu(&config);
            
            lsm6d_set_accel_scale(config.fs_xl);
            lsm6d_set_gyro_scale(config.fs_g);
            
            clear_data(samples);
            
            index = 0;
            
            xl_x_sum = 0;
            xl_y_sum = 0;
            xl_z_sum = 0;

            g_x_sum = 0;
            g_y_sum = 0;
            g_z_sum = 0;
            
            state = MODE_RUN;
        }
    }
}

