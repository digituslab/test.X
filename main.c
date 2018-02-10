/* 
 * File:   main.c
 * Author: ywatabe
 *
 * Created on February 8, 2018, 11:19 PM
 */

// PIC16F1705 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PPS1WAY = OFF    // Peripheral Pin Select one-way control (The PPSLOCK bit can be set and cleared repeatedly by software)
#pragma config ZCDDIS = ON      // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR)
#pragma config PLLEN = OFF      // Phase Lock Loop enable (4x PLL is enabled when software sets the SPLLEN bit)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), high trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

unsigned int adconv()
{
     unsigned int temp;

     GO_nDONE = 1 ;         // PICにアナログ値読取り開始を指示
     while(GO_nDONE) ;      // PICが読取り完了するまで待つ
     temp = ADRESH ;        // PICは読取った値をADRESHとADRESLのレジスターにセットする
     temp = ( temp << 8 ) | ADRESL ;  // 10ビットの分解能力です

     return temp ;
}

void interrupt isr_func(void);
void setup_tmr(void);
void flip_led(void);
void setup_irq(void);
void setup_sci(void);

/*
 * 
 */
int main(int argc, char** argv) {
    unsigned int num;
    
    // Basic configurations for PIC16F1705
     OSCCON     = 0b01110010 ;  // 内部クロックは8ＭＨｚとする
     OPTION_REG = 0b00000000 ;  // デジタルI/Oに内部プルアップ抵抗を使用する
     ANSELA     = 0b00000000 ;  // AN0-AN3は使用しない全てデジタルI/Oとする
     ANSELC     = 0b10000000 ;  // AN4-AN7は使用しない全てデジタルI/Oとする
     TRISA      = 0b00100000 ;  // RA5のみ入力、その他は全て出力に割当てる(RA3は入力専用)
     TRISC      = 0b00001000 ;  // ピン(RC)は全て出力に割当てる
     WPUA       = 0b00100000 ;  // RA5は内部プルアップ抵抗を指定する
     PORTA      = 0b00000000 ;  // RA出力ピンの初期化(全てLOWにする)
     PORTC      = 0b00000000 ;  // RC出力ピンの初期化(全てLOWにする)
     
     ADCON0 = 0b00011101;
     ADCON1 = 0b10010000;
     

     // ＣＬＣの設定
     RC2PPS    = 0b00001100 ;   // CLC1OUT出力をRA2から出す。
     CCP1CON = 0b00001100;      // CCP1 is PWM mode
     CCPR1L = 10;               // Initial PWM duty is 10.
     /*
     CLCIN0PPS = 0x05 ;       // CLCIN0 入力はRA5から入れる
     CLCIN1PPS = 0x00 ;
     CLCIN2PPS = 0x00 ;
     CLCIN3PPS = 0x00 ;
     CLC1GLS0  = 0x02 ;       // ゲート１は入力１の信号(CLCIN0)を真(非反転)で使用する
     CLC1GLS1  = 0x08 ;       // ゲート２は入力２の信号(PWM3)を真(非反転)で使用する
     CLC1GLS2  = 0x08 ;       // ゲート３は入力２の信号(PWM3)を真(非反転)で使用する
     CLC1GLS3  = 0x08 ;       // ゲート４は入力２の信号(PWM3)を真(非反転)で使用する
     CLC1SEL0  = 0x00 ;       // 入力１はCLCIN0PPSのレジスタ(RA5)から入力
     CLC1SEL1  = 0x0E ;       // 入力２はPWM3から入力
     CLC1SEL2  = 0x00 ;
     CLC1SEL3  = 0x00 ;
     CLC1POL   = 0x00 ;       // ゲート出力もＣＬＣ出力も反転出力はしない
     CLC1CON   = 0x82 ;       // CLCは有効で、ロジックは[4-input AND]
     */
     // ＰＷＭ３の設定
     
     //PWM3CON = 0b10000000 ;   // PWM3ピンからは出力しない
     PWM3DCH = 64 ;           // デューティ値は５０％で初期化
     PWM3DCL = 0 ;
     
     T2CON   = 0b00000010 ;   // TMR2プリスケーラ値を１６倍に設定
     TMR2    = 0 ;            // タイマー２カウンターを初期化
     PR2     = 153 ;          // PWMの周期を設定（1000Hzで設定）
     TMR2ON  = 1 ;            // TMR2(PWM)スタート
     
     num = 255;
     
     setup_tmr();
     setup_irq();
     setup_sci();
     
     while(1) {
          num = adconv() ;  // 7番ピン(AN7)から半固定抵抗の値を読み込む
          CCPR1L = num/4 ;  // アナログ値からのデータでデューティ値を設定
          //CCPR1L = 10;
          /*
          PORTC=0x00;
          PORTC=0xff;
          */
        TX1REG = 0x55;
        while(TX1STAbits.TRMT != 1);

     }    
    return (EXIT_SUCCESS);
}

void flip_led(void){
    PORTAbits.RA5 = !PORTAbits.RA5;
    /*
    unsigned char led = PORTA;
    if(led == 0b00100000){
        led = led & 0b11011111;
    } else {
        led = led | 0b00100000;
    }
    PORTA = led;
    */
}

void setup_sci(void){
    TRISCbits.TRISC4 = 0;   // RC4 is output port
    //ANSELCbits.ANSC4 = 0;   // 
    RC1STAbits.SPEN = 1;    // Enables serial port
    RC4PPS = 0b00010100;      // RC4 functionality is TX
    TX1STA = 0x00;          // Once clears the register to zero
    TX1STAbits.TXEN = 1;    // Transmission is enabled
    TX1STAbits.SYNC = 0;    // Asynchronous mode
    TX1STAbits.BRGH = 1;    // High speed baud rate mode
    TX1STAbits.CSRC = 1;
    SP1BRGL = 207;           // BRG value for 9,600 bps
    SP1BRGH = 0;
    BAUD1CON = 0x00;        // Once clears the reigister to zero
    BAUD1CONbits.SCKP = 0;  // Transmit data polarity, 0 = Non inverted
    BAUD1CONbits.BRG16= 1;  // Use 16bit baud rate generator
 
}

void setup_tmr(void){
    // Once disable all interrupt.
    INTCON = 0x00;
    
    // Enables global interrupt enable bit but peripheral and timer0.
    INTCON = 0b10000000;
    OPTION_REG = 0b00000111;
    TMR0 = 0;
    INTPPS = 0b00010000;
    
    // RA5 setup
    TRISA = 0x00;
    PORTA = 0b00000000;
    
    // Enables timer and interrupt.
    INTCON = 0b10100000;
}

void setup_irq(void){
    // Setup pin direction (set to IN)
    TRISAbits.TRISA4 = 1;
    TRISAbits.TRISA5 = 0;
    
    // Setup pin function to use RA4 as IRQ.
    INTPPS = 0b00000100;
    
    // Setup INT controller register to get interruption as RA4 changed.
    INTCONbits.INTE = 1;
}

void interrupt isr_func(void){
   
    static int tm = 0;

    INTCONbits.GIE = 0;
    
    if(TMR0IF){
        // Waits 10 times for to blink the LED much slower.
        if(tm > 5){
            flip_led();
            tm = 0;
        } else {
            tm++;
        }
        TMR0IF &= 0;
    }
    
    if(INTCONbits.INTF){
        while(TX1STAbits.TRMT != 1);
        TX1REG = 'A';
        INTCONbits.INTF &= 0;
//        PORTCbits.RC4 = 0;
//        PORTCbits.RC4 = 1;
//        PORTCbits.RC4 = 0;
        PORTAbits.RA5 = 1;
    }
    
    INTCONbits.GIE = 1;
    return;
}
