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
#include <math.h>

#define _XTAL_FREQ 8000000

#define DEG2RAD (3.141592/180.0)
#define LCD_ADRES   (0x3e)
#define RW_0    (0)

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
void setup_dac(void);

void setup_i2c(void);

void i2c_master_start(void);
void i2c_idleCheck(char mask);
int i2c_Start(int adrs,int rw);
void i2c_stop();

void LCD_Clear(void);
void LCD_SetCursor(int col, int row);
void LCD_Putc(char c);
void LCD_Puts(const char * s);
void LCD_CreateChar(int p,char *dt);
void LCD_Init(void);

static int AckCheck;

/*
 * 
 */
int main(int argc, char** argv) {
    unsigned int num;
    static float yd = 0;
    static float xd = 0;
    
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
     setup_i2c();
     
     LCD_Init();
     LCD_Clear();
     
//     i2c_write_cmd(0x38);
//     i2c_write_cmd(0x39);
//     i2c_write_cmd(0x14);
//     i2c_write_cmd(0x73);
//     i2c_write_cmd(0x52);
//     i2c_write_cmd(0x6c);
//     i2c_write_cmd(0x38);
//     i2c_write_cmd(0x01);
//     i2c_write_cmd(0x0f);
     
//     setup_sci();
//     setup_dac();
     
         
        LCD_SetCursor(0,0); 
        LCD_Puts("Hello world!");
        LCD_SetCursor(0,1); 
        LCD_Puts("Feb.19,2018");
//        LCD_SetCursor(1,0); 
//        LCD_Putc('B');        
    while(1) {

//          i2c_master_start();
//     i2c_master_write(0x7c);
//     i2c_master_stop();
     }    
    return (EXIT_SUCCESS);
}

void setup_i2c(void){
    __delay_ms(40);
    
    // Configure pin function
    RC1PPS = 0b00010001;    // RC1 is SDA for output
    RC0PPS = 0b00010000;    // RC0 is SCL for output

    // Set pin's special function for digital purposes
    ANSELCbits.ANSC0 = 0;   // RC0 is digital input pin
    ANSELCbits.ANSC1 = 0;   // RC1 is digital input pin
    
    // Initiates pin directions as input
    TRISCbits.TRISC1 = 1;   // RC1 is input for SDA IN
    TRISCbits.TRISC0 = 1;   // RC0 is input for SCL IN
    
    
    // I2C module configuration
    SSP1STAT = 0b10000000;          // Standard srew-rate (100kbps)
    SSP1CON1 = 0b00101000;          // Use RC0 and RC1 as SDA/SCL
    //SSP1CON2 = 0;
    SSP1ADD = 12;                   // Clock speed for I2C communication
    SSP1STAT = 0;

    PIE1bits.SSP1IE = 1;
    PIE2bits.BCL1IE = 1;
    INTCONbits.GIE = 1;             // Enables whole interruption 
    INTCONbits.PEIE = 1;
    PIR1bits.SSP1IF = 0;
    PIR2bits.BCL1IF = 0;
    
 
    // Waits until I2C module become stable condition
    //__delay_ms(100);
    
    return;
}

void i2c_idleCheck(char mask){
    while (( SSP1CON2 & 0x1F ) | (SSP1STAT & mask)) ;
}

int i2c_Start(int adrs,int rw)
{
     // スタート(START CONDITION)
     i2c_idleCheck(0x5) ;
     SSP1CON2bits.SEN = 1 ;
     // [スレーブのアドレス]を送信する
     i2c_idleCheck(0x5) ;
     AckCheck = 1 ;
     SSPBUF = (char)((adrs<<1)+rw) ;    // アドレス + R/Wを送信
     while (AckCheck) ;                 // 相手からのACK返答を待つ
     return SSPCON2bits.ACKSTAT ;
}

void i2c_stop()
{
     // ストップ(STOP CONDITION)
     i2c_idleCheck(0x5) ;
     SSP1CON2bits.PEN = 1 ;
}

int i2c_send(char dt)
{
     i2c_idleCheck(0x5) ;
     AckCheck = 1 ;
     SSP1BUF = dt ;                      // データを送信
     while (AckCheck) ;                 // 相手からのACK返答を待つ
     return SSP1CON2bits.ACKSTAT ;
}

// ＬＣＤにコマンドを発行する処理
void command(unsigned char c)
{
     int  ans ;
     ans = i2c_Start(LCD_ADRES,RW_0);     // スタートコンディションを発行する
     if (ans == 0) {
          // command word の送信
          i2c_send(0b00000000) ;             // control byte の送信(コマンドを指定)
          i2c_send(c) ;                      // data byte の送信
     }
     i2c_stop() ;                            // ストップコンディションを発行する
     __delay_us(26) ;
}
/*******************************************************************************
*  LCD_Clear( )                                                                *
*    ＬＣＤモジュールの画面を消す処理                                          *
*******************************************************************************/
void LCD_Clear(void)
{
     command(0x01) ;     // Clear Display : 画面全体に20Hのｽﾍﾟｰｽで表示、ｶｰｿﾙはcol=0,row=0に移動
     __delay_ms(5) ;  // LCDが処理(2.16ms)するのを待ちます
}
/*******************************************************************************
*  LCD_SetCursor(col,row)                                                      *
*    ＬＣＤモジュール画面内のカーソル位置を移動する処理                        *
*                                                                              *
*    col : 横(列)方向のカーソル位置(0-7)                                       *
*    row : 縦(行)方向のカーソル位置(0-1)                                       *
*******************************************************************************/
void LCD_SetCursor(int col, int row)
{
     int row_offsets[] = { 0x00, 0x40 } ;
     command(0x80 | (col + row_offsets[row])) ; // Set DDRAM Adddress : 00H-07H,40H-47H
}
/*******************************************************************************
*  LCD_Putc(c)                                                                 *
*    文字列は、NULL(0x00)まで繰返し出力します。                                *
*                                                                              *
*    c :  出力する文字データを指定                                             *
*******************************************************************************/
void LCD_Putc(char c)
{
     int  ans ;
     ans = i2c_Start(LCD_ADRES,RW_0);     // スタートコンディションを発行する
     if (ans == 0) {
          // command word の送信
          i2c_send(0b01000000) ;             // control byte の送信(データを指定)
          i2c_send(c) ;                      // data byte の送信
     }
     i2c_stop() ;                            // ストップコンディションを発行する
     __delay_us(26) ;
}
/*******************************************************************************
*  LCD_Puts(*s)                                                                *
*    ＬＣＤに文字列データを出力する処理                                        *
*    文字列は、NULL(0x00)まで繰返し出力します。                                *
*                                                                              *
*    *s :  出力する文字列のデータを格納した場所のアドレスを指定                *
*******************************************************************************/
void LCD_Puts(const char * s)
{
     int  ans ;
     while(*s) {
          LCD_Putc(*s++);
     }
}
/*******************************************************************************
*  LCD_CreateChar(p,*dt)                                                       *
*    オリジナルのキャラクタを登録します                                        *
*                                                                              *
*    p   : 登録する場所の指定(０〜５の６ヶ所のみ)                              *
*    p   : 登録する場所の指定(０〜５の６ヶ所のみ)                              *
*    *dt : 登録したいキャラクタのデータを格納したバッファを指定                *
********************************************************************************/
//　**** ACM602では未検証です ****
void LCD_CreateChar(int p,char *dt)
{
    command(0x40 | p << 3 );
    __delay_ms(5) ;
    for (int i=0; i < 8; i++) {
         LCD_Putc(*dt++);
    }
}
/*******************************************************************************
*  LCD_Init( )                                                                 *
*    ＬＣＤの初期化を行う処理                                                  *
*******************************************************************************/
void LCD_Init(void)
{
    command(0x38);      // Clear Diplay.
    __delay_ms(5) ;     // 5ms待ちます。
    command(0x39);      // Clear Diplay.
    __delay_ms(5) ;     // 5ms待ちます。
    command(0x14);      // Clear Diplay.
    __delay_ms(5) ;     // 5ms待ちます。
    command(0x73);      // Clear Diplay.
    __delay_ms(5) ;     // 5ms待ちます。
    command(0x53);      // Clear Diplay.
    __delay_ms(5) ;     // 5ms待ちます。
    command(0x6c);      // Clear Diplay.
    __delay_ms(5) ;     // 5ms待ちます。
    command(0x38);      // Clear Diplay.
    __delay_ms(5) ;     // 5ms待ちます。
    //---
    command(0x01);      // Clear Diplay.
    __delay_ms(5) ;     // 5ms待ちます。
    command(0x38);      // Function Set.
    __delay_ms(5) ;     // 5ms待ちます。
                        // 0b000x0000; DL; Interface Data length. 8bits/4bits
                        // 0b0000x000; N;  Numbers of display line. 2-line/1-line
                        // 0b00000x00; F;  Display font type. 5x10dots/5x8dots
    command(0x0c);      // Display ON/OFF Control.
    __delay_ms(5) ;     // 5ms待ちます。
                        // 0b00000x00; D;  Set display on/off.
                        // 0b000000x0; C;  Set curthor on/off.
                        // 0b0000000x; B;  Set blinking of curthor on/off.
    command(0x06);      // Entry Mode Set.
    __delay_ms(5) ;     // 5ms待ちます。
                        // 0b000000x0; I/D;Assign curthor moving direction.
                        // 0b0000000x; I/D;Enable the shift of entire display.
     LCD_Clear() ;       // Clear Display          : 画面を消去する
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

void setup_dac(void){
    // Setup ANSEL bit for DAC output as RA2
    ANSELAbits.ANSA2 = 1;   // RA2 is used for analog output.
    
    // Setup pin function
    
    // Configures DAC and enables it.
    DAC1CON0 = 0b10010000;
    DAC1CON1 = 0x00;
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
        if(tm > 10){
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
    
    if(PIR1bits.SSP1IF == 1){
        if(AckCheck == 1){ AckCheck = 0; }
        PIR1bits.SSP1IF = 0;
    }
    
    if(PIR2bits.BCL1IF == 1){
        PIR2bits.BCL1IF = 0;
    }
    
    INTCONbits.GIE = 1;
    return;
}
