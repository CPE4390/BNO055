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

//Defines for BNO - need to be before .h is included
//#define MACHINE_16_BIT

//Project includes
#include "../src/BNO055_I2C.h"
//#include "../src/driver/bno055.h"
#include <stdio.h>
#include <math.h>

/*
Connections:
        Master RD5 <-> SDA
        Master RD6 <-> SCL
        Master RB1 <-> INT
 */

volatile char dataReady = 0;


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

    //setup INT1 for falling edge
    TRISB |= 0b00000010;
    INTCON2bits.INTEDG1 = 0;
    INTCON3bits.INT1IE = 1;
    INTCON3bits.INT1IF = 0;

    //setup Timer2 for 1ms ticks
    T2CONbits.T2CKPS = 0b10; //1:16 prescale
    T2CONbits.TOUTPS = 4; //1:5 postscale gives 100 kHz count rate
    PR2 = 100;
    TMR2 = 0;
    tickCount = 0;
    PIR1bits.TMR2IF = 0;
    PIE1bits.TMR2IE = 1;
    INTCONbits.PEIE = 1;
    //INTCONbits.GIE = 1;
    T2CONbits.TMR2ON = 1;
    
    unsigned char buff[6];
    pic18_i2c_enable();
    pic18_i2c_read(0x29, 0, 6, buff);
    printf("IDs: %02x %02x %02x %02x\r\n", buff[0], buff[1], buff[2], buff[3]);
    printf("SW Version %02x%02x\r\n", buff[5], buff[4]);
    pic18_i2c_read(0x29, 0x36, 1, buff);
    printf("POST result: %02x\r\n", buff[0] & 0x0f);
    
    //struct bno055_t myBNO;
    //myBNO.dev_addr = BNO055_I2C_ADDR1;
    
    while (1) {
        if (dataReady) {
            dataReady = 0;
            LATDbits.LATD0 ^= 1;
            
        }
    }
}

void __interrupt(high_priority) HighIsr(void) {
    if (PIR1bits.TMR2IF == 1) {
        ++tickCount;
        PIR1bits.TMR2IF = 0;
    }
    if (INTCON3bits.INT1IF == 1) {
        dataReady = 1;
        INTCON3bits.INT1IF = 0;
    }
}

void putch(char c) {
    while (TX1IF == 0);
    TXREG1 = c;
}


