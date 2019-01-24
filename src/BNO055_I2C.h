#ifndef BNO055_I2C_H
#define	BNO055_I2C_H

#include <xc.h>

#ifdef	__cplusplus
extern "C" {
#endif

    //i2c functions
    int pic18_i2c_enable(void);
    int pic18_i2c_disable(void);
    int pic18_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
    int pic18_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

    //timing functions
    #define _XTAL_FREQ   32000000L
    extern unsigned long tickCount;
    int pic18_delay_ms(unsigned long num_ms);
    int pic18_get_ms(unsigned long *count);
    
    
#ifdef	__cplusplus
}
#endif

#endif	/* BNO055_I2C_H */

