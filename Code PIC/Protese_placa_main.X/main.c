/*
 * File:   main.c
 * Author: vinic
 *
 * Created on December 9, 2021, 1:27 PM
 * 
 * links:
 * https://www.microchip.com/doclisting/TechDoc.aspx?type=CodeExamples
 * https://turbofuture.com/computers/How-Use-ADCs-in-dspic30f-and-dspic33f
 * https://turbofuture.com/computers/Configure-PWM-on-dspic30f-and-dspic33f
 * https://flashgamer.com/blog/comments/basic-blink-example-for-dspic30f4013
 * https://www.youtube.com/watch?v=-Hrz6PZwdiw&list=PLp8IkbtcjNXwearFBxVFtzjGOzzEW1IR_
 * https://ww1.microchip.com/downloads/en/DeviceDoc/50002446C.pdf
 * https://microchipsupport.force.com/s/article/Delay-functions-in-XC16-and-XC8-and-their-functionaility
 * 
 * -----------------------------------------------------------------------------------------------
 * TAREFAS:
 * Testar pinos de controle de alimentação                          - OK
 * Tensão VM                                                        -
 * Tensão +3V3-1                                                    - OK
 * Tensão +3V3-2                                                    - OK
 * Tensão +5V/1                                                     - OK
 * Tensão +5V/1                                                     - OK
 */



// DSPIC33FJ128MC804 Configuration Bit Settings

// 'C' source line config statements


// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Mode (Internal Fast RC (FRC))
#pragma config IESO = OFF               // Internal External Switch Over Mode (Start-up device with user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor (Clock switching is enabled, Fail-Safe Clock Monitor is disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = ON              // Watchdog Timer Window (Watchdog Timer in Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR1             // POR Timer Value (Disabled)
#pragma config ALTI2C = ON              // Alternate I2C  pins (I2C mapped to ASDA1/ASCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

/*
// Internal FRC Oscillator
_FOSCSEL(FNOSC_FRC); // FRC Oscillator 
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);
// Clock Switching is enabled and Fail Safe Clock Monitor is disabled
// OSC2 Pin Function: OSC2 is Clock Output
// Primary Oscillator Mode: Disabled

_FWDT(FWDTEN_OFF); // Watchdog Timer Enabled/disabled by user software
// (LPRC can be disabled by clearing SWDTEN bit in RCON register
_FPOR(FPWRT_PWR1); // Turn off the power-up timers.
_FGS(GCP_OFF); // Disable Code Protection
 */
//#define SYS_FREQ 7370000UL
#define FRC 7370000UL
#define FCY FRC/2
//#define _XTAL_FREQ 7370000UL


#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <libpic30.h>
//#include <delay.h>;
#include <math.h>
//#include <pps.h>


int main(void) {

    // EXAMPLE!!!
    // Configure Oscillator to operate the device at 40Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 8M*40/(2*2)=80Mhz for 8M input clock
    PLLFBD = 5; // M=40
    CLKDIVbits.PLLPOST = 0; // N1=2
    CLKDIVbits.PLLPRE = 0; // N2=2
    OSCTUN = 0; // Tune FRC oscillator, if FRC is used


    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;

    // Make all the ANx pins as digital pins
    AD1PCFGL = 0xFFFF;
    //AD1PCFGH = 0xFFFF;

    // -------------------------------------------------------------------------
    /*
    //AD1PCFG = 0xFFFF; // set to digital I/O (not analog)
    TRISA = 0x1F; // REset
    PORTA = 0x00;
    LATA = 0x00;
    ODCA = 0x00;
    
    TRISC = 0x1F;
    PORTC = 0x00;
    LATC = 0x00;
    ODCC = 0x00;
    */
    //TRISA = 0x00;
    _TRISA10 = 0x00;
    _TRISA9 = 0x00;
    _TRISB5 = 0x00;
    TRISC = 0x00;
    //_TRISC4 = 0x00;

    while (1) {
        
        //PORTA = 0b10010000000;
        _RA10 = 0x01; //0b1; // FLAG_VM
        _RA9 = 0x01; //0b1; // SYNC_REG
        PORTC = 0x18; //0b11000; //FLAG_V1 (RC3) E FLAG_V2 (RC4)
        // SLEEP_DVR (RA7)
        _RB5 = 0x01;// FLAG_DVR (RB5)
        // SYNC_REG (RA9) (LOW =  ECONOMIA DE ENERGIA, HIGH = transfere a energia não utilizada de volta para a entrada)
        
       // __delay32(1500000);
        __delay_ms(500);
        
        //_RA10 = 0x00; //0b0;
        //PORTA = 0b00000000000;
        _RB5 = 0x00;// FLAG_DVR (RB5)
        PORTC = 0x00; //0b00000;
        __delay_ms(500);
 
    }

    return 0;
}
