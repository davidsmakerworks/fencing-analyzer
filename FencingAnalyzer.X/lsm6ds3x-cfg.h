/* 
 * Application-specific configuration values for LSM6DS3x
 *
 * Requires definitions for:
 * LSM6D_SPI_ACTIVE() - Code to set CSN pin low
 * LSM6D_SPI_IDLE() - Code to set CSN pin high
 * LSM6D_INT1 - Bit value for I/O pin input for INT1
 * LSM6D_INT2 - Bit value for I/O pin input for INT2
 * LSM6D_SPI_TRANSFER(x) - Transfer one byte to/from SPI bus without changing CSN
 * 
 * Modify includes as necessary for processor architecture
 * 
 */

#include <xc.h>

#ifndef LSM6DS3X_CFG_H
#define	LSM6DS3X_CFG_H

#ifdef	__cplusplus
extern "C" {
#endif

#define LSM6D_SPI_ACTIVE()      LATBCLR = _LATB_LATB13_MASK
#define LSM6D_SPI_IDLE()        LATBSET = _LATB_LATB13_MASK

#define LSM6D_INT1              PORTBbits.RB15
#define LSM6D_INT2              PORTBbits.RB14

#define LSM6D_SPI_TRANSFER(x)   spi2_transfer(x)

uint8_t spi2_transfer(uint8_t data);

#ifdef	__cplusplus
}
#endif

#endif	/* LSM6DS3X_CFG_H */

