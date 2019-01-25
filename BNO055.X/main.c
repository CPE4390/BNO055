#include <xc.h>

// PIC18F87J11 Configuration Bit Settings
// CONFIG1L
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT disabled (control is placed on SWDTEN bit))
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Reset on stack overflow/underflow enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG1H
#pragma config CP0 = OFF        // Code Protection bit (Program memory is not code-protected)

// CONFIG2L
#pragma config FOSC = HSPLL     // Oscillator Selection bits (HS oscillator, PLL enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Two-Speed Start-up (Internal/External Oscillator Switchover) Control bit (Two-Speed Start-up disabled)

// CONFIG2H
#pragma config WDTPS = 32768    // Watchdog Timer Postscaler Select bits (1:32768)

// CONFIG3L
#pragma config EASHFT = ON      // External Address Bus Shift Enable bit (Address shifting enabled, address on external bus is offset to start at 000000h)
#pragma config MODE = MM        // External Memory Bus Configuration bits (Microcontroller mode - External bus disabled)
#pragma config BW = 16          // Data Bus Width Select bit (16-bit external bus mode)
#pragma config WAIT = OFF       // External Bus Wait Enable bit (Wait states on the external bus are disabled)

// CONFIG3H
#pragma config CCP2MX = DEFAULT // ECCP2 MUX bit (ECCP2/P2A is multiplexed with RC1)
#pragma config ECCPMX = DEFAULT // ECCPx MUX bit (ECCP1 outputs (P1B/P1C) are multiplexed with RE6 and RE5; ECCP3 outputs (P3B/P3C) are multiplexed with RE4 and RE3)
#pragma config PMPMX = DEFAULT  // PMP Pin Multiplex bit (PMP port pins connected to EMB (PORTD and PORTE))
#pragma config MSSPMSK = MSK7   // MSSP Address Masking Mode Select bit (7-Bit Address Masking mode enable)

//Project includes
#include "../src/BNO055_I2C.h"
#include "../src/driver/bno055.h"
#include <stdio.h>
#include <math.h>

/*
Connections:
        Master RD5 <-> SDA
        Master RD6 <-> SCL
 */

void main(void) {
    OSCTUNEbits.PLLEN = 1;
    __delay_ms(10); //Wait for PLL to stabilize
    TRISDbits.TRISD0 = 0;
    LATDbits.LATD0 = 1;
    //Configure the USART for 115200 baud asynchronous transmission
    SPBRG1 = 68; //115200 baud
    SPBRGH1 = 0;
    TXSTA1bits.BRGH = 1;
    BAUDCON1bits.BRG16 = 1;
    TXSTA1bits.SYNC = 0;
    RCSTA1bits.SPEN = 1;
    TXSTA1bits.TXEN = 1;
    printf("Starting\r\n");

    //setup INT1 for rising edge
    TRISB |= 0b00000010;
    INTCON2bits.INTEDG1 = 1;
    INTCON3bits.INT1IE = 1;
    INTCON3bits.INT1IF = 0;

    //Enable interrupts
    //INTCONbits.PEIE = 1;
    //INTCONbits.GIE = 1;

    unsigned char buff[6];
    pic18_i2c_enable();
    pic18_i2c_read(0x29, 0, buff, 6);
    printf("IDs: %02x %02x %02x %02x\r\n", buff[0], buff[1], buff[2], buff[3]);
    printf("SW Version %02x%02x\r\n", buff[5], buff[4]);
    pic18_i2c_read(0x29, 0x36, buff, 1);
    printf("POST result: %02x\r\n", buff[0] & 0x0f);

    struct bno055_t myBNO;
    myBNO.dev_addr = BNO055_I2C_ADDR2; //0x29
    myBNO.bus_read = pic18_i2c_read;
    myBNO.bus_write = pic18_i2c_write;
    myBNO.delay_msec = pic18_delay_ms;
    bno055_init(&myBNO);
    printf("IDs: %02x %02x %02x %02x\r\n", myBNO.chip_id, myBNO.accel_rev_id, myBNO.mag_rev_id, myBNO.gyro_rev_id);
    printf("SW Version %04x\r\n", myBNO.sw_rev_id);
    bno055_set_operation_mode(BNO055_OPERATION_MODE_IMUPLUS);
    while (1) {
        __delay_ms(500);
        struct bno055_euler_float_t eulerAngles;
        bno055_convert_float_euler_hpr_deg(&eulerAngles);
        printf("\froll= %.1f\r\n", eulerAngles.r);
        printf("pitch = %.1f\r\n", eulerAngles.p);
        printf("yaw = %.1f\r\n", eulerAngles.h);
        unsigned char accelCalib = 0;
        unsigned char gyroCalib = 0;
        unsigned char magCalib = 0;
        unsigned char sysCalib = 0;
        bno055_get_accel_calib_stat(&accelCalib);
        bno055_get_gyro_calib_stat(&gyroCalib);
        bno055_get_mag_calib_stat(&magCalib);
        bno055_get_sys_calib_stat(&sysCalib);
        printf("Calibration a:%d g:%d m:%d sys:%d\r\n", accelCalib, gyroCalib, magCalib, sysCalib);
        LATDbits.LATD0 ^= 1;
    }
}

void __interrupt(high_priority) HighIsr(void) {
    if (INTCON3bits.INT1IF == 1) {
        //Handle BNO055 interrupt signal
        INTCON3bits.INT1IF = 0;
    }
}

void putch(char c) {
    while (TX1IF == 0);
    TXREG1 = c;
}


