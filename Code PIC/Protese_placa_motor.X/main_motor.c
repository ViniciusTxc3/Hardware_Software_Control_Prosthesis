/*
 * File:   main_motor.c
 * Author: vinic
 *
 * Created on December 13, 2021, 5:12 PM
 * 
 * links:   http://www.mech.tohoku-gakuin.ac.jp/rde/contents/sendai/mechatro/archive/Datasheet05/dsPIC_ADC.pdf
 *          http://www.t-es-t.hu/download/microchip/rm33f28_ADC2_a.pdf
 *          http://ww1.microchip.com/downloads/en/devicedoc/70183d.pdf
 *          https://turbofuture.com/computers/How-to-use-Interrupts-in-Pic-MicroControllers
 *          https://www.pantechsolutions.net/blog/how-to-interface-interrupts-with-dspic30f4011-dspic-development-board/
 */

// DSPIC33FJ32MC202 Configuration Bit Settings

// 'C' source line config statements

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

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
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR1             // POR Timer Value (Disabled)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//#define SYS_FREQ 7370000UL
#define FRC 7370000UL
#define FCY FRC/2
#define _XTAL_FREQ 7370000UL


#include <xc.h>
//#include <p33FJ32GP202.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <libpic30.h>
//#include <delay.h>;
#include <math.h>
//#include <pps.h>

#define FLAG_DVR    _RB2
#define I_MOTOR     _RA0
#define ECA         _RB6
#define ECB         _RB7
#define CTR_DVR1    _RB12
#define CTR_DVR2    _RB14
#define SLEEP_DVR   _RB13

// Variaveis para encoder
#define MAX_PULSE 500 // Máximo de pulso encoder (valor empírico)
//uint8_t pinALast = 0;
uint16_t pulse = 0;
_Bool limit = 0;

// Variaveis Corrente
#define MAX_CURRENT 4095*0.8 // (80% da corrente máxima medida (valor empírico))

// ----- Funções protótipos----------------------------------------------------/
void Interrupt_Init(void);
void init_ADC(void);
uint16_t readADC(int _ch);
//-----------------------------------------------------------------------------/
int main(void) {
    // OUTPUT
    _TRISB2 = 0x00;
    _TRISB12 = 0x00;
    _TRISB13 = 0x00;
    _TRISB14 = 0x00;

    // INPUT
    _TRISA0 = 0x01;
    _TRISB6 = 0x01;
    _TRISB7 = 0x01;

    // interrupção canais EMG
    // interrupção encoder - OK
    Interrupt_Init();
    // interrupção corrente motor - OK
    // interrupção botão externo
    // I2C
    
    //pinALast = ECB; // posição inicial
    
    // Acionando motor
    SLEEP_DVR = 0x00;
    FLAG_DVR = 0x00;
    CTR_DVR1 = 0x01;
    CTR_DVR2 = 0x00;

    while (1) {
        
        uint16_t _ADC_AN0 = readADC(I_MOTOR); // Leitura AN0
        if ((limit) || (_ADC_AN0 > MAX_CURRENT)) // Passar dos limites para o motor
        {
            SLEEP_DVR = 0x00;
            FLAG_DVR = 0x00;
            CTR_DVR1 = 0x00;
            CTR_DVR2 = 0x00;
        }
        /*
         // TESTE DE OSCILAR PINO PARA GRAVAÇÃO INCIAL
         _RB2 = 0x01;// FLAG_DVR (RB2)
         __delay32(1500000);
         _RB2 = 0x00;//
         __delay32(1500000);
         */
    }
    return 0;
}

void Interrupt_Init(void) {
    _INT0EP = 0; // negative/positive edge detect polarity              - POSITIVE
    _INT0IE = 1; // enable/disable external interrupt                   - ENABLE
    _INT0IP = 1; // 3-bit (0 to 7) interrupt priority config            - 001

}

void __attribute__((interrupt, auto_psv)) _INT0Interrupt(void) {
    if (ECB != ECA) 
    {
        // forward
        limit = 0;
        pulse++;
    } 
    else
    {
        // backward
        limit = 0;
        pulse--;
    }
    if (abs(pulse) == MAX_PULSE) // +/- value MAX_PULSE
    {
            //motor_pwm_config(0, "off");
            limit = 1;
    }
    _INT0IF = 0;
}

void init_ADC(void)
{
    _TRISA0 = 0x01;
    ADPCFG = 0x0000;  //Selecting all analogue pins to analogue mode
    _AD12B = 0x01;
    _CHPS = 0x00;
    AD1CHS0bits.CH0SA = 0;
    
    AD1CHS123bits.CH123SA=0; // Select AN0 for CH1 +ve input
    AD1CON1bits.ADON = 1; // turn on the ADC
    ADPCFGbits.PCFG0 = 0; // AN0 is configured as analog input
}
uint16_t readADC(int _ch)
{                      
    AD1CHS0  = _ch;               // select analog input channel
    ADC1BUF0 = 0x0000; // Reset Buffer
    AD1CON1bits.SAMP = 1;        // start sampling, automatic conversion will follow
    while (!AD1CON1bits.DONE); // wait to complete the conversion
    return ADCBUF0;         // read the conversion result
}