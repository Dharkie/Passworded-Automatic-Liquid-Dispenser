
/*-----------------------------------------------------------------------------
    File Name:    AUTOMATICDISPENSER.c
    Author:       Francis Nzekwe
    Date:         3/12/2023
    Modified:     None
    Fanshawe College, 2023

Description: This Program has the following function:
            1. Password change feature.
            2. Password Validation feature
            3. Cup check feature
            4. Real-time water level check feature
            5. Communication feature
            6. Display feature
            7. Error message feature
            8. Interrupt feature on 2 seconds timer set by Timer0
            9. Timer1 to calculate time spent
*/
#include <p18f45k22.h>
#include <stdio.h>
#include <timers.h>
#include <delays.h>

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

// DEFINES FOR TRANSCIEVER=================================================

#define AD8 LATCbits.LATC3
#define AD9 LATCbits.LATC2
#define AD10 LATCbits.LATC1
#define AD11 LATCbits.LATC0
#define TE LATCbits.LATC4
#define MOTOR LATCbits.LATC5

// PROCESSOR DEFINES FOR TIMER1============================================
#define TMR1ON T1CONbits.TMR1ON
#define TMR1IF PIR1bits.TMR1IF

// TIMER0 DEFINES==========================================================
#define T08BIT T0CONbits.T08BIT
#define T0CS T0CONbits.T0CS
#define PSA T0CONbits.PSA
#define T0PS T0CONbits.T0PS
#define TMR0ON T0CONbits.TMR0ON
#define TMR0IF INTCONbits.TMR0IF 

// INTERRUPT DEFINES======================================================
#define TMR0IE INTCONbits.TMR0IE
#define IPEN RCONbits.IPEN
#define GIE INTCONbits.GIE
#define PEIE INTCONbits.PEIE

// DEFINES FOR PINCHANGE INTERUPT ON PORT B<7:4>===========================
#define RBIF INTCONbits.RBIF
#define IOCB7 IOCBbits.IOCB5
#define RBIE INTCONbits.RBIE
// DEFINES FOR LCD DISPLAY=======================================================
#define RS PORTEbits.RE0
#define EN PORTEbits.RE1

// DEFINES FOR KEYPAD=======================================================
#define ROWS 4
#define COLS 3
#define COL1 LATDbits.LATD0
#define COL2 LATDbits.LATD1
#define COL3 LATDbits.LATD2
#define ROW1 PORTAbits.RA0
#define ROW2 PORTAbits.RA1
#define ROW3 PORTAbits.RA2
#define ROW4 PORTAbits.RA3

// USART DEFINES
#define TXEN TXSTA1bits.TXEN
#define SYNC TXSTA1bits.SYNC
#define BRGH TXSTA1bits.BRGH
#define SPEN RCSTA1bits.SPEN
#define CREN RCSTA1bits.CREN
#define BRG16 BAUDCON1bits.BRG16

// REGULAR DEFINES===========================================
#define BAUD 4

// declaration================================================
int baudRates[BAUD] = {0x067, 0x33, 0xC, 0x0B};

// Global declaration for String===========================================
char startKey = 0, check1 = TRUE;
int index = 0;
float distance = 0;
int time = 0;
int motorTime = 0;

// Variable for Password Input and Quantity Selection Input
unsigned char key1 = '.', key2 = '.', key3 = '.', key4 = '.';

// Variable for password change(default password)
unsigned char passOne = '1', passTwo = '8', passThree = '0';
char newPassword[] = {"NEW PASSWORD"};
char newPassAnnounce[] = {"PASSWORD IS"};
char oldPassword[] = {"OLD PASSWORD"};

char arr[SIZE];
char message2[] = {"WRONG PASSWORD"};
char enterPassword[] = {"ENTER PASSWORD"};

// Start Page Strings
char var1[] = {"ENTER # TO START"};
char var2[] = {"*CHANGE PASSWORD"};
char error1[] = {"WRONG INPUT"};
char error2[] = {"REFILL!!"};

// User Selction Strings
char option1[] = {"1.VISCOUS"};
char option2[] = {" 2.SEMI"};
char option3[] = {"3.NON-VISC."};
char makeSelection[] = {"MAKE SELECTION"};
char cup[] = {"NO CUP!!"};

// Serving========================================
char serving[] = {"SERVING!!"};

// FUNCTIONS==========================================================

/*>>> set_osc: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Francis Nzekwe
Date:        01/07/2023
Modified:    None
Desc:        This function is used to setup the PIC oscillator to your desired frequency(4 & 8MHZ only)
Input:         char clock, this is used to input the frequency value to set the oscillation frequency
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

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

/*>>> tmr0Config: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Francis Nzekwe
Date:        01/07/2023
Modified:    None
Desc:        This function is used to setup Timer0 configuration
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

void tmr0Config(char enableTmr0, char t0Bit, char t0cs, char pSa, char t0Ps)
{
    T08BIT = t0Bit;
    T0CS = t0cs;
    PSA = pSa;
    /* PSA VALUES
    PSV 256 = 7
    PSV 128 = 6
    PSV 64 = 5
    PSV 32 = 4
    PSV 16 = 3
    PSV 8 = 2
    PSV 4 = 1
    PSV 2 = 0
    */
    T0PS = t0Ps;
    TMR0ON = enableTmr0;
} // eo timer0Config::

/*>>> resetTimer0: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:        Francis Nzekwe
Date:        28/06/2023
Modified:    None
Desc:        To reset Timer0 to preset count and clear the Timer0 flag for next count for 2secs using 8mHz FCY
Input:         None
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void resetTmr0(void)
{
    TMR0H = 0x0B;
    TMR0L = 0xDC;
    
    /*TMR0H = 0x67;
    TMR0L = 0x69;
*/
    TMR0IF = FALSE;

} // eo resetTimer0::

/*>>> portConfigForUltraSense: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:      Francis Nzekwe
Date:        28/06/2023
Modified:    None
Desc:        To configure the ports for the ultrasonic sensor
Input:       None
Returns:     None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void portConfigForUltraSense()
{
    ANSELB = 0x00;
    PORTB = 0x00; // initial state of pins is off
    TRIGDIR = 0;  // output pin
    ECHODIR = 1;  // input pin
    EYER2 = 1;
    EYER = 1;
} // eo portConfigForUltrsnicSensor::

/*>>> triggerUltraSense: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:      Francis Nzekwe
Date:        28/06/2023
Modified:    None
Desc:        This is used to turn on/off the trigger of the ultrasonic sensor, creating a pulse
Input:       None
Returns:     None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void triggerUltraSense()
{
    TRIG = TRUE;
    Delay10KTCYx(10); // delay (1Milliseconds)
    TRIG = FALSE;
} // eo triggerForUltrasonicSense::

// >>> Interrupts Configuration >>>
/*>>> disableInterrupt(): >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:      Francis Nzekwe
Date:        28/07/2023
Modified:    None
Desc:        Function to setup and disable the 2 seconds Timer0 interrupt
Input:       None
Returns:     None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void disableInterrupt(void)
{
    TMR0IE = FALSE; // timer0 enable
    IPEN = FALSE;    // disable priority
    GIE = FALSE;
    PEIE = TRUE; // peripheral enable
} // eo disableInterrupt::

/*>>> enableInterrupt(): >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:      Francis Nzekwe
Date:        28/07/2023
Modified:    None
Desc:        Function to setup and enable Timer0 interrupt for use
Input:       None
Returns:     None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void enableInterrupt(void)
{
    TMR0IE = TRUE; // timer0 enable
    IPEN = FALSE;  // disable priority
    GIE = TRUE;
    PEIE = TRUE; // peripheral enable
} // eo enableInterrupt::

/*>> > Keypad_port_config: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author :    Francis Nzekwe
Date :      01 / 07 / 2023
Modified :  None
Desc :      This function is used to configure poerts for the 3 by 4 keypad
Input :     None
Returns :   None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void Keypad_port_config()
{
    ANSELA = 0x00;
    TRISA = 0x0F;
    PORTA = 0x00;
} // eo Keypad_port_config::

/*>> > timer1Config: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author :   Francis Nzekwe
Date :     01 / 07 / 2023
Modified : None
Desc :    This function sets up the Timer1 registers for use
Input :   None
Returns : None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void timer1Config()
{
    T1CON = 0b00000110; // Disable timer1; No prescaler; 16bit resolution timer
    TMR1IF = 0;            // Turn-off Flag permanently
    TMR1H = 0;
    TMR1L = 0;    // Initialize initial count value
    TMR1ON = 0; // Turn-off TIMER1
} // eo timer0Config::

/*>> > LCD_Clock: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author :    Francis Nzekwe
Date :      01 / 07 / 2023
Modified :  None
Desc :      This function is used to setup the Clock line for the LCD
Input :     None
Returns :   None
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
Author :    Francis Nzekwe
Date :      01 / 07 / 2023
Modified :  None
Desc :      This function is used to setup LCD in 4-bit mode and print a character in 4-bit mode
Input :     char data
Returns :   None
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

    PORTD = (low4 << 4); // Shift 4 places left to write to upper nibble
    LCD_Clock();
    Delay10KTCYx(3);
} // eo LCD_print_4bitchar::

/*>> > LCD_print_4BitString: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author :    Francis Nzekwe
Date :      01 / 07 / 2023
Modified :  None
Desc :      This function is used to setup LCD in 4-bit mode and print a character in 4-bit mode
Input :     char *strPtr, to point to the position zero (0) of the input string
Returns :   None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

void LCD_print_4BitString(char *strPtr)
{
    while (*strPtr)
    {
        LCD_print_4bitchar(*strPtr);
        strPtr++;
    }
} // eo LCD_print_4BitString::

/*>> > LCD_CMD4bits: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author :    Francis Nzekwe
Date :      01 / 07 / 2023
Modified :  None
Desc :      This function is used to execute commads in 4-bit mode in LCD
Input :     unsigned char command, to take in commands as instructed in datasheet
Returns :   None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void LCD_CMD4bits(unsigned char command)
{
    // uses the upper nibble
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
    PORTD = (low4 << 4); // Shift 4 places left to write to upper nibble
    LCD_Clock();
    Delay10KTCYx(3);
} // eo LCD_CMD4bits:

/*>> > LCD_Init_4bits: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author :    Francis Nzekwe
Date :      01 / 07 / 2023
Modified :  None
Desc :      This function is used to setup 4-bit mode LCD for use, information was gotted from datasheet
Input :     None
Returns :   None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void LCD_Init_4bits()
{
    // The Init. Procedure As Described In The Datasheet
    Delay10KTCYx(3);    // wait for 15ms
    LCD_CMD4bits(0x03); // as instructed in datasheet
    LCD_CMD4bits(0x03); // as instructed in datasheet
    LCD_CMD4bits(0x03); // as instructed in datasheet

    Delay10KTCYx(3);
    LCD_CMD4bits(0x02); // as instructed in datasheet

    LCD_CMD4bits(0x28); // set 4bit mode, 2-line display, font 5*8

    // set home position
    // LCD_CMD4bits(0x08);

    LCD_CMD4bits(0x0F); // display oN //cursor oN //cursor blink

    // clear display
    LCD_CMD4bits(0x01);

    // ENtry mode set
    // increment
    // no shift of display
    LCD_CMD4bits(0x06);

} // eo LCD_Init_4bits::

/*>> > LCD_Set_Cursor4bits::: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author :    Francis Nzekwe
Date :      01 / 07 / 2023
Modified :  None
Desc :      This function is used to setup positioning of display on the LCD (16 by 2)
Input :     unsigned char column, to take column inputs. Values range from 0-16 positions
            unsigned char row, to take row value input. Range from 0-1
Returns :   None
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

/*>> > LCD_port_config4bits::: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author :    Francis Nzekwe
Date :      01 / 07 / 2023
Modified :  None
Desc :      This function is used to Configure the port for the LCD screen
Returns :   None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

void LCD_port_config4bits()
{
    ANSELD = 0x00;
    TRISD = 0x00;
    LATD = 0x00;

    ANSELE = 0x00;
    TRISE = 0x00;
    LATE = 0x00;
} // eo LCD_port_config4bits::

/*>>> serialPortConfig: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:      FRANCIS NZEKWE and JOSNA JOHNSON
Date:        24/10/2023
Modified:    None
Desc:        setting the serial ports to required configuration
             1200/2400/9600/10417 baud rate
             transmitting on, serial port on
Input:       int baudRate, this is defined as an array value of 4 int values and sizeof 20bytes.
             how to use: serialPortConfig(baudRate[0])
Returns:    None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

void serialPortConfig(int baudRate)
{
    SPBRGH1 = (baudRate & 0xFF00);
    SPBRG1 = (baudRate & 0x00FF);
    TXSTA1bits.TX9 = 0; // 8bits

    SYNC = 0; // async
    BRGH = 0; // high speed

    SPEN = 1; // receive switch on
    RCSTA1bits.RX9 = 0;
    CREN = 1;
    BRG16 = 0;
    TXEN = 1; // switch

} // eo serialPortConfig::

/*>> > KeyPress: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author :    Francis Nzekwe
Date :      01 / 07 / 2023
Modified :  None
Desc :      This function performs a keypress scan, and is used for a 3 by 4 keypad
Input :     None
Returns :   None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

// Scan all the keypad keys to detect any pressed key.
unsigned char keyPress()
{
    char flag = 1;
    while (flag)
    {
        COL1 = 1;
        COL2 = 0;
        COL3 = 0;

        // Delay10KTCYx(10);
        if (ROW1 == 1)
        {
            while (ROW1 == 1)
                ;
            flag = 0; // while (COL3==1);
            return '1';
        }
        if (ROW2 == 1)
        {
            while (ROW2 == 1);
            flag = 0; // while (COL1==1);
            return '4';
        }
        if (ROW3 == 1)
        {
            while (ROW3 == 1)
                ;
            flag = 0; // while (COL2==1);
            return '7';
        }
        if (ROW4 == 1)
        {
            while (ROW4 == 1)
                ;
            flag = 0; // while (COL2==1);
            return '*';
        }

        COL1 = 0;
        COL2 = 1;
        COL3 = 0;

        // Delay10KTCYx(10);
        if (ROW1 == 1)
        {
            while (ROW1 == 1)
                ;
            flag = 0; // while (COL3==1);
            return '2';
        }
        if (ROW2 == 1)
        {
            while (ROW2 == 1)
                ;
            flag = 0; // while (COL1==1);
            return '5';
        }
        if (ROW3 == 1)
        {
            while (ROW3 == 1)
                ;
            flag = 0; // while (COL2==1);
            return '8';
        }
        if (ROW4 == 1)
        {
            while (ROW4 == 1)
                ;
            flag = 0; // while (COL2==1);
            return '0';
        }
        COL1 = 0;
        COL2 = 0;
        COL3 = 1;

        // Delay10KTCYx(10);
        if (ROW1 == 1)
        {
            while (ROW1 == 1)
                ;
            flag = 0; // while (COL3==1);
            return '3';
        }
        if (ROW2 == 1)
        {
            while (ROW2 == 1)
                ;
            flag = 0; // while (COL1==1);
            return '6';
        }
        if (ROW3 == 1)
        {
            while (ROW3 == 1)
                ;
            flag = 0; // while (COL2==1);
            return '9';
        }
        if (ROW4 == 1)
        {
            while (ROW4 == 1)
                ;
            flag = 0;
            return '#';
        }
    }
    // return 'n';
}//eo KeyPress::

// interrupt prototype or declaraation
void ISR(void);

//function to go to the interrupt vector rom on address 0x08
#pragma code interrupt_vector = 0x08
void interrupt_vector()
{
    _asm GOTO ISR
        _endasm
}
#pragma code

/*>>> ISR: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:      Francis Nzekwe
Date:        08/06/2023
Modified:    None
Desc:        This is the function that will be serviced by the interrupt, when it is 2 seconds, it interrupts
             and calculates the distance value from the ultrasonic sensor.
Input:       None
Returns:     None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
#pragma interrupt ISR
void ISR(void)
{
    if (TMR0IF)
    {
        //Disable interrupt
        time = 0;
        TMR0IE = FALSE;
        GIE = FALSE;
        resetTmr0();// 5SECS
        triggerUltraSense(); /* Transmit 10us pulse to HC-SR04 */
        while (ECHO == 0); /* Wait for rising edge at Echo pin */
        TMR1H = 0;
        TMR1L = 0;/* Load Timer1 register with 0 */
        TMR1ON = 1;/* Turn ON Timer1*/
        while (ECHO == 1);//&& !TMR1IF);/* Wait for falling edge */
        time = ReadTimer1();/* Copy Time when echo is received */
        TMR1ON = 0;/* Turn OFF Timer1 */
        distance = (int)(time / 117.00); /* Distance =(velocity x Time)/2 */
        //enable interrupt
        TMR0IE = TRUE; // timer0 enable
        GIE = TRUE;
    }
} // eo ISR()::

/*>>> sendMessage(): >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:      Francis and Purnima Kaushal
Date:        08/06/2023
Modified:    None
Desc:        This is used to send a message from this device to another device usng an FS1000a transmitter
             Reciever pair. using an HT12E(Encoder) and HT12D(decoder)
Input:       char messageNumber, used to input the character into the switch statement. F,p or N
Returns:     None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void sendMessage(char messageNumber)
{

    switch (messageNumber)
    {
    case 'F':
        TE = 0;
        AD8 = 1;
        AD9 = 1;
        AD10 = 0;
        AD11 = 0;
        
        break;
    case 'N':
        TE = 0;
        AD8 = 0;
        AD9 = 0;
        AD10 = 0;
        AD11 = 0;
    
        break;
    case 'p':
        TE = 0;
        AD8 = 0;
        AD9 = 1;
        AD10 = 0;
        AD11 = 0;
    
        break;
    default:

        break;
    }
} // eo sendMessage()::

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
    TRISC = 0xC0;  // PORT RC0~RC4 AS OUTPUT
    ANSELC = 0x00; // all digital
    TE = TRUE;       // INITIAL STATE OF PIN IS HIGH
} // eo configureSystem()::

void main()
{

    // char buffer1[16];

    set_osc(8);
    portConfigForUltraSense();
    LCD_port_config4bits();
    Keypad_port_config();
    disableInterrupt();
    tmr0Config(1, 0, 0, 0, 5); // 7
    timer1Config();
    LCD_Init_4bits();
    resetTmr0();
    configureSystem();
    WDTCON = TRUE;
    MOTOR = 0;

    while (1)
    {
        sendMessage('p');
        distance=0;
        // Display message
        LCD_Set_Cursor4bits(0, 0);
        LCD_print_4BitString(var1);
        LCD_Set_Cursor4bits(0, 1);
        LCD_print_4BitString(var2);
        startKey = 0;
        startKey = keyPress();
        Delay10KTCYx(100);    // wait for keypress action
        LCD_CMD4bits(0x01); // clear screen

        LCD_Set_Cursor4bits(0, 0);
        LCD_print_4bitchar(startKey);

        switch (startKey)
        {
        case '#':
            startKey = 0;
            while (check1)
            {
                LCD_CMD4bits(0x01);
                LCD_Set_Cursor4bits(0, 0);
                LCD_print_4BitString(enterPassword);

                LCD_Set_Cursor4bits(0, 1);
                key1 = keyPress();
                Delay10KTCYx(100);
                LCD_print_4bitchar(key1);

                LCD_Set_Cursor4bits(1, 1);
                key2 = keyPress();
                Delay10KTCYx(100);
                LCD_print_4bitchar(key2);

                LCD_Set_Cursor4bits(2, 1);
                key3 = keyPress();
                Delay10KTCYx(100);
                LCD_print_4bitchar(key3);

                Delay10KTCYx(100);
                LCD_CMD4bits(0x01);

                // fix result into array[SIZE]
                // at the end this will result to: arr[3]={1,8,0}
                for (index = 0; index < SIZE; index++)
                {
                    switch (index)
                    {
                    case 1:
                        arr[index] = key2;
                        break;
                    case 2:
                        arr[index] = key3;
                        break;
                    case 0:
                        arr[index] = key1;
                        break;
                    }
                } // eo for loop for password check

                // Condition to make sure of our password
                if (arr[0] == passOne && arr[1] == passTwo && arr[2] == passThree)
                {
                    // to break away from loop
                    check1 = FALSE;
                }
                else
                {
                    // ERROR MESSAGE
                    LCD_Set_Cursor4bits(0, 0);
                    LCD_print_4BitString(error1);
                    Delay10KTCYx(100);
                    LCD_CMD4bits(0x01);
                }
            } // eo while check

            // To re-enter loop after program sweep
            check1 = TRUE;

            // Display machine options on screen for user to select
            // option1~3
            LCD_Set_Cursor4bits(0, 0);
            LCD_print_4BitString(makeSelection);
            Delay10KTCYx(200);

            LCD_Set_Cursor4bits(0, 0);
            LCD_print_4BitString(option1);
            LCD_Set_Cursor4bits(7, 0);
            LCD_print_4BitString(option2);

            LCD_Set_Cursor4bits(0, 1);
            LCD_print_4BitString(option3);
            //Delay10KTCYx(200);

            key4 = keyPress();
            Delay10KTCYx(100);
            LCD_CMD4bits(0x01);

            // display users selection
            LCD_Set_Cursor4bits(0, 0);
            LCD_print_4bitchar(key4);
            Delay10KTCYx(100);
            LCD_CMD4bits(0x01);

            // if true, keep asking the stupid user, who just decided to frustrate us for correct input
            while (key4 != '1' && key4 != '2' && key4 != '3')
            {
                // ERROR MESSAGE
                LCD_Set_Cursor4bits(0, 0);
                LCD_print_4BitString(error1);
                Delay10KTCYx(100);
                LCD_CMD4bits(0x01);

                LCD_Set_Cursor4bits(0, 0);
                LCD_print_4BitString(option1);
                LCD_Set_Cursor4bits(7, 0);
                LCD_print_4BitString(option2);

                LCD_Set_Cursor4bits(0, 1);
                LCD_print_4BitString(option3);

                key4 = keyPress();
                Delay10KTCYx(100);
                LCD_CMD4bits(0x01);

                LCD_Set_Cursor4bits(0, 0);
                LCD_print_4bitchar(key4);
                Delay10KTCYx(100);
                LCD_CMD4bits(0x01);
            }

            // CHEK FOR CUP POSITION================================
            while ((PORTBbits.RB3 == 1) || (PORTBbits.RB2 == 1)) // POLL till SENSOR SENSES CUP
            {
                LCD_Set_Cursor4bits(0, 0);
                LCD_print_4BitString(cup);
                Delay10KTCYx(300);
                LCD_CMD4bits(0x01);
                sendMessage('N'); // send message to next device
            }

            motorTime = 0;
            switch (key4)
            {
            case '1':
                enableInterrupt(); // check for liquid level and runmotor===============================
                Delay10KTCYx(400);//2secs
                while ((distance <= 10) && (motorTime <= 4))
                {
                    LCD_Set_Cursor4bits(0, 0); // Print "SERVING"
                    LCD_print_4BitString(serving);
                    MOTOR = 1; // OPEN LIQID PUMP
                    motorTime++;
                }
                MOTOR = 0;

                break;
            case '2':
                enableInterrupt(); // check for liquid level and runmotor===============================
                Delay10KTCYx(400);//2secs
                while ((distance <= 10) && (motorTime <= 7))
                {
                    LCD_Set_Cursor4bits(0, 0); // Print "SERVING"
                    LCD_print_4BitString(serving);
                    MOTOR = 1; // OPEN LIQID PUMP
                    motorTime++;
                }
                MOTOR = 0;

                break;
            case '3':
                enableInterrupt(); // check for liquid level and runmotor===============================
                Delay10KTCYx(400);//2secs
                while ((distance <= 10) && (motorTime <= 10))
                {
                    LCD_Set_Cursor4bits(0, 0); // Print "SERVING"
                    LCD_print_4BitString(serving);
                    MOTOR = 1; // OPEN LIQID PUMP
                    motorTime++;
                }
                MOTOR = 0;

                break;

            default:
                break;

            } // eo key switch
            disableInterrupt();
            while (distance >10)
            {
                LCD_Set_Cursor4bits(0, 0); // Print "SERVING"
                LCD_print_4BitString(error2);
                LCD_CMD4bits(0x01);
                Delay10KTCYx(300);

                sendMessage('F'); // send message to next device
            }
            sendMessage('p');
            startKey = 0;
            break;

        case '*':          // For Password Reset
            startKey = 0; // Step1: enter old password to get a new one
            while (check1)
            {
                LCD_CMD4bits(0x01);
                LCD_Set_Cursor4bits(0, 0);
                LCD_print_4BitString(oldPassword);

                LCD_Set_Cursor4bits(0, 1);
                key1 = keyPress();
                Delay10KTCYx(100);
                LCD_print_4bitchar(key1);

                LCD_Set_Cursor4bits(1, 1);
                key2 = keyPress();
                Delay10KTCYx(100);
                LCD_print_4bitchar(key2);

                LCD_Set_Cursor4bits(2, 1);
                key3 = keyPress();
                Delay10KTCYx(100);
                LCD_print_4bitchar(key3);

                
                Delay10KTCYx(100);
                LCD_CMD4bits(0x01);

                // fix result into array[SIZE]
                // at the end this will result to: arr[3]={1,8,0}
                for (index = 0; index < SIZE; index++)
                {
                    switch (index)
                    {
                    case 1:
                        arr[index] = key2;
                        break;
                    case 2:
                        arr[index] = key3;
                        break;
                    case 0:
                        arr[index] = key1;
                        break;
                    }
                }

                // Condition to make sure of our password
                if (arr[0] == passOne && arr[1] == passTwo && arr[2] == passThree)
                {
                    // to break away from loop
                    check1 = FALSE;
                }
                else
                {
                    // ERROR MESSAGE
                    LCD_Set_Cursor4bits(0, 0);
                    LCD_print_4BitString(error1);
                    Delay10KTCYx(100);
                    LCD_CMD4bits(0x01);
                }
            }
            check1 = TRUE;

            LCD_CMD4bits(0x01);
            LCD_Set_Cursor4bits(0, 0);
            LCD_print_4BitString(newPassword);

            LCD_Set_Cursor4bits(0, 1);
            passOne = keyPress();
            Delay10KTCYx(100);
            LCD_print_4bitchar(passOne);

            LCD_Set_Cursor4bits(1, 1);
            passTwo = keyPress();
            Delay10KTCYx(100);
            LCD_print_4bitchar(passTwo);

            LCD_Set_Cursor4bits(2, 1);
            passThree = keyPress();
            Delay10KTCYx(100);
            LCD_print_4bitchar(passThree);

            Delay10KTCYx(100);    // wait
            LCD_CMD4bits(0x01); // clear display for next input

            LCD_Set_Cursor4bits(0, 0);
            LCD_print_4BitString(newPassAnnounce); // new password is
            LCD_Set_Cursor4bits(0, 1);
            LCD_print_4bitchar(passOne);
            LCD_Set_Cursor4bits(1, 1);
            LCD_print_4bitchar(passTwo);
            LCD_Set_Cursor4bits(2, 1);
            LCD_print_4bitchar(passThree);
            Delay10KTCYx(200); // wait

            startKey = 'A';
            break;
        default:
            break;
        }
        //}
        ClrWdt(); // CLRWDT;
    }
}
