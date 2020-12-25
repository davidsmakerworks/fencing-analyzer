/*
 * Fencing Action Analyzer
 * Copyright (c) 2020 David Rice
 * 
 * Main source file for fencing action analyzer
 * Currently implements only transmission of raw IMU data
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
#include <stdio.h>
#include <stdbool.h>

#include "lsm6ds3x.h"
#include "lsm6ds3x-cfg.h"

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

/*
 Zero out all data in an array of LSM6D_SENSOR_DATA structures.
 
 TODO: Find a better way to do with with memset()
 */
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
    init_osc();
    init_ports();
    init_pps();
    init_spi2();
    init_uart2();
    
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

