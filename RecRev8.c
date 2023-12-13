//>>> Program Header >>>
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
File Name:    Group3 (Francis, Purnima & Josna)
Author:        Group3
Date:        08/06/2023
Modified:    None
Fanshawe College, 2022

Description:     Practice of configurations
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

// Preprocessor >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

// Libraries >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#include <p18f45k22.h>
#include <delays.h>
#include <stdio.h>
#include <timers.h>
#include <usart.h>

// PRAGMA SPACE=============================================================
#pragma config FOSC = INTIO67
#pragma config PLLCFG = OFF
#pragma config PRICLKEN = ON
#pragma config FCMEN = OFF
#pragma config IESO = OFF

#pragma config PWRTEN = OFF
#pragma config BOREN = ON
#pragma config BORV = 285

#pragma config WDTEN = ON
#pragma config WDTPS = 16384 // one minute timer for WatchDog

#pragma config PBADEN = OFF

#pragma config LVP = OFF
#pragma config MCLRE = EXTMCLR

// Constants >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#define TRUE        1
#define    FALSE        0

//LED STATES>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#define LEDOFF 0
#define LEDON 1

#define RC1BYTE RCREG1

//RECEIVER BUFFER
#define BUFFERSIZE 25

//Transmission USART Interrupt Flag>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#define TXIF0 PIR1bits.TX1IF

//Receive USART Interrupt Flag>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#define RCIF0 PIR1bits.RC1IF


// Preprocessor defines====================================================
#define TRUE 1
#define FALSE 0
#define SIZE 3
#define TRIGDIR TRISBbits.TRISB4
#define ECHODIR TRISBbits.TRISB5
#define TRIG LATBbits.LATB4 // Trig pin state def
#define ECHO PORTBbits.RB5    // Echo pin state def
#define EYER TRISBbits.TRISB2
#define EYER2 TRISBbits.TRISB3

// PROCESSOR DEFINES FOR TIMER1============================================

#define RBIF INTCONbits.RBIF
#define RBIE INTCONbits.RBIE
#define IOCB5 IOCBbits.IOCB5
#define TMR0IE INTCONbits.TMR0IE
#define TMR0IF INTCONbits.TMR0IF
#define TMR0ON T0CONbits.TMR0ON
//DEFINES FOR LCD DISPLAY=======================================================
#define RS PORTEbits.RE0
#define EN PORTEbits.RE1

//USART DEFINES
#define TXEN TXSTA1bits.TXEN
#define SYNC TXSTA1bits.SYNC
#define BRGH TXSTA1bits.BRGH
#define SPEN RCSTA1bits.SPEN
#define CREN RCSTA1bits.CREN
#define BRG16 BAUDCON1bits.BRG16
#define BAUD 4

//RX
#define D8 PORTBbits.RB0
#define D9 PORTBbits.RB1
#define D10 PORTBbits.RB2
#define D11 PORTBbits.RB3
// Global declaration for String===========================================
char startKey = 0, check1 = TRUE;
int index = 0;
float distance = 0;
int time = 0;





char errorMessage[] = {"REFILL"};


// User Selction Strings
char option1[] = {"1.FULL"};
char option2[] = {"2.MEDIUM"};
char errorMessage2[] = {"SYSTEM is NORMAL"};
char errorMessage1[] = {"insert Object"};
char cup[] = {"NO CUP!!"};
//char txBuf[BUFSIZE];
int baud[BAUD]={0x067,0x33,0x0C,0x0B};
// FUNCTIONS==========================================================

void set_osc(char clock)
{
switch (clock)
{
case 4:
    OSCCON = 0x53;
    OSCCON2 = 0x04;
    OSCTUNE = 0x80;
    break;
case 8:
    OSCCON = 0x63;
    OSCCON2 = 0x04;
    OSCTUNE = 0x80;
    break;
default:
    break;
}
while (OSCCONbits.HFIOFS != 1)
    ; // wait for osc to become stable
} // eo: set_osc::

void serialPortConfig(int baudRate)
{
SPBRGH1 = (baudRate&0xFF00);
SPBRG1 = (baudRate&0x00FF);
TXSTA1bits.TX9=0;//8bits

SYNC=0;//async
BRGH=0;//high speed

SPEN=1;//receive switch on
RCSTA1bits.RX9=0;
CREN=1;
BRG16=0;
TXEN=1;//switch

}// eo serialPortConfig::

void LCD_port_config4bits()
{
ANSELD = 0x00;
TRISD = 0x00;
LATD = 0x00;

ANSELE = 0x00;
TRISE = 0x00;
LATE = 0x00;
} // eo LCD_port_config::

/*>> > LCD_Clock: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author : Francis Nzekwe
Date : 01 / 07 / 2023
Modified : None
Desc : This function
Input :
Returns : None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void LCD_Clock(void)
{
// Period of ENABLE line = 2US total
EN = 1;
Delay10KTCYx(1);

EN = 0;
Delay10KTCYx(1);

} // eo LCD_Clock::

/*>> > LCD_print_4bitchar: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author : Francis Nzekwe
Date : 01 / 07 / 2023
Modified : None
Desc : This function
Input :
Returns : None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void LCD_print_4bitchar(unsigned char data)
{
char high4 = 0;
char low4 = 0;
high4 = (data & 0xF0);
low4 = (data & 0x0F);
RS = 1;
PORTD = high4;
LCD_Clock();


PORTD = (low4<<4);//Shift 4 places left to write to upper nibble
LCD_Clock();
Delay10KTCYx(3);
} // eo LCD_print_4bitchar::

/*>> > LCD_CMD4bits: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author : Francis Nzekwe
Date : 01 / 07 / 2023
Modified : None
Desc : This function
Input :
Returns : None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void LCD_CMD4bits(unsigned char command)
{
//uses the upper nibble
char high4 = 0;
char low4 = 0;
// split command into upper and lower nibbles
high4 = (command & 0xF0);
low4 = (command & 0x0F);
// Move The Command Data To LCD
RS = 0;
PORTD = (high4); // Send The EN Clock Signal
LCD_Clock();
Nop();

          // Move The Command Data To LCD
PORTD = (low4<<4); // Shift 4 places left to write to upper nibble
LCD_Clock();
Delay10KTCYx(3);
} // eo LCD_CMD4bits:

/*>> > LCD_Set_Cursor4bits::: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author : Francis Nzekwe
Date : 01 / 07 / 2023
Modified : None
Desc : This function
Input :
Returns : None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void LCD_Set_Cursor4bits(unsigned char column, unsigned char row)
{
switch (row)
{
case 0:
    LCD_CMD4bits(0x80 | column);
    break;

case 1:
    LCD_CMD4bits(0xC0 | column);
    break;

default:
    break;
}
} // eo LCD_Set_Cursor4bits::

/*>> > LCD_print_4BitString: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author : Francis Nzekwe
Date : 01 / 07 / 2023
Modified : None
Desc : This function
Input :
Returns : None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

void LCD_print_4BitString(char *strPtr)
{
while (*strPtr)
{
    LCD_print_4bitchar(*strPtr);
    strPtr++;
}
} // eo LCD_print_4BitString::

/*>>> txMessage: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        FRANCIS NZEKWE and JOSNA JOHNSON
Date:        24/10/2023
Modified:    None
Desc:        setting the serial ports to required configuration
        1200/2400/9600/10417 baud rate for 8MHZ CLock
        transmitting on, serial port on
Input:         None
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void txMessage(char*ptr)
{
while(*ptr)
{    while(!PIR1bits.TX1IF);
    TXREG1=*ptr;
    ptr++;
}
LCD_Set_Cursor4bits(0, 0);
//LCD_print_4BitString(var1);
}//eo txMessage::

/*>> > LCD_Init_4bits: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author : Francis Nzekwe
Date : 01 / 07 / 2023
Modified : None
Desc : This function
Input :
Returns : None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void LCD_Init_4bits()
{
// The Init. Procedure As Described In The Datasheet
Delay10KTCYx(3); // wait for 15ms
LCD_CMD4bits(0x03);//as instructed in datasheet
LCD_CMD4bits(0x03);//as instructed in datasheet
LCD_CMD4bits(0x03);//as instructed in datasheet

Delay10KTCYx(3);
LCD_CMD4bits(0x02);//as instructed in datasheet

LCD_CMD4bits(0x28);// set 4bit mode, 2-line display, font 5*8

// set home position
//LCD_CMD4bits(0x08);

LCD_CMD4bits(0x0F);// display oN //cursor oN //cursor blink


// clear display
LCD_CMD4bits(0x01);

// ENtry mode set
// increment
// no shift of display
LCD_CMD4bits(0x06);

} // eo LCD_Init_4bits::


// Functions >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// >>> Oscillator Configuration >>>
/*>>> set_osc_p18f45k22_4MHz(): >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Purnima Kaushal
Date:        08/06/2023
Modified:    None
Desc:        configures oscillator to 4 MHz
Input:         None
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void set_osc_p18f45k22_4MHz(void)
{
OSCCON =  0x53;                 // Sleep on slp cmd, HFINT 4MHz, INT OSC Blk
OSCCON2 = 0x04;                    // PLL No, CLK from OSC, MF off, Sec OSC off, Pri OSC
OSCTUNE = 0x80;                    // PLL disabled, Default factory freq tuning
while (OSCCONbits.HFIOFS != 1); // wait for osc to become stable
}
//eo: set_osc_p18f45k22_4MHz::

// >>> Port Configuration >>>
/*>>> configurePorts(): >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Purnima Kaushal
Date:        08/06/2023
Modified:    None
Desc:        configure ports A to D
Input:         None
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void configurePorts(void)
{
//PORTA
TRISA = 0xFF; // set PBs at PA0-PA2
ANSELA = 0x00; //all digital
PORTA = 0x0F; //active low PA0-PA3
LATA = 0x00; // setting initial voltage to 0V
//PORTB
TRISB = 0xFF; // all inputs
ANSELB = 0x00; //all digital
LATB = 0x00; // setting initial voltage to 0V
//PORTC
TRISC = 0xFF; //all inputs with Serial receiver on RC7 Serial transmitter on RC6
ANSELC = 0x00; //all digital
LATC = 0x00; // setting initial voltage to 0V
//PORTD
TRISD = 0xCF; // RD4Aand RD5 outputs
ANSELD = 0x00; //all digital
LATD = 0x10; // setting initial voltage to 0V LEDs at RD4-RD7
//PORTE
TRISE = 0xFF; //all inputs
ANSELE = 0x00; //all digital
LATE = 0x00; // setting initial voltage to 0V
}// eo configurePorts()::
// >>> USART Configuration >>>
/*>>> configureUSART(): >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Purnima Kaushal
Date:        21/07/2023
Modified:    None
Desc:        EUSART module, Serial port / TX / RX all on, 9600 baud, Asynchronous mode.
Input:         None
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void configureUSART(void)
{
SPBRG1 = 25; //set the Buad rate to 9600 for 4MHz
TXSTA1 = 0x24; //enable transmitter, asynchronous mode, high speed mode
RCSTA1 = 0x90; // serial port enabled, 8 bit reception mode, enable the receiver
BAUDCON1 = 0x00;
}//eo configureUSART::

// >>> configureSystem >>>
/*>>> configureSystem(): >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Purnima Kaushal
Date:        08/06/2023
Modified:    None
Desc:        Configure the system by configuring the ports, timer and osc
Input:         None
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void configureSystem(void)
{
set_osc(8);
//    set_osc_p18f45k22_4MHz();
//configurePorts();
//PORTC
TRISC = 0xFF; //all inputs with Serial receiver on RC7 Serial transmitter on RC6
ANSELC = 0x00; //all digital
LATC = 0x00; // setting initial voltage to 0V
//PORTB
TRISB = 0x0F; //RB0-RB3 as inputs
ANSELB = 0x00; //all digital
PORTB = 0x00; // setting initial voltage to 0V
}// eo configureSystem()::

void main( void )
{
unsigned char ch;
configureSystem();
LCD_port_config4bits();
LCD_Init_4bits();
while(1)
{
    if(( D11== 0) && (D10==0) && (D9==1) && (D8==1) )
    {
        LCD_Set_Cursor4bits(0, 1);
        LCD_print_4BitString(errorMessage);
        Delay10KTCYx(400);
        LCD_CMD4bits(0x01);
    }
    else if(( D11== 0) && (D10==0) && (D9==0) && (D8==0) )
    {
        LCD_Set_Cursor4bits(0, 1);
        LCD_print_4BitString(errorMessage1);
        Delay10KTCYx(400);
        LCD_CMD4bits(0x01);
    }
    else  if(( D11== 0) && (D10==0) && (D9==1) && (D8==0) )
    {
        LCD_Set_Cursor4bits(0, 1);
        LCD_print_4BitString(errorMessage2);
        LCD_CMD4bits(0x01);
        Delay10KTCYx(400);
    }
    LCD_CMD4bits(0x01);
}//eo while
}//eo main::
