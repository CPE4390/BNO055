#ifndef BNO055_I2C_H
#define	BNO055_I2C_H

#include <xc.h>

#ifdef	__cplusplus
extern "C" {
#endif

    //i2c functions
    char pic18_i2c_enable(void);
    char pic18_i2c_disable(void);
    char pic18_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char const *data, unsigned char length);
    char pic18_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char *data, unsigned char length);

    //timing functions
    #define _XTAL_FREQ   32000000L
    char pic18_delay_ms(unsigned long num_ms);
    
    
#ifdef	__cplusplus
}
#endif

#endif	/* BNO055_I2C_H */

