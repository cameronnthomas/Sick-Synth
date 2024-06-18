/* 
 * File:   main.c
 * Author: camit
 *
 * Created on March 18, 2022, 2:42 PM
 */
// CONFIG1L
#pragma config FEXTOSC = OFF    // External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_64MHZ// Power-up default value for COSC bits (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)

#include <stdio.h>
#include <stdlib.h> l
#define _XTAL_FREQ 64000000


#include <xc.h>
#include <pic18f47k40.h>

#include "ANY_LCD_H.h"

/*channel number that connects to VSS*/
#define DISCHARGE_SAMPLE_CAP                0x3C
/*channel number that connects to RA0*/
#define ANALOG_CHANNEL                      0x00


/* function prototypes */
/* check function definitions below */
void delayUS(uint32_t usec);
void writeLCD(uint8_t bitfield);
static void CLK_Initialize(void);
static void PPS_Initialize(void);
static void PORT_Initialize(void);
static void SPI1_Initialize(void);
static void SPI1_client1Select(void);
static void SPI1_client1Deselect(void);
void delayms(uint32_t msec);
static void ADCC_init(void);
static void ADCC_dischargeSampleCap(void);
static uint16_t ADCC_readValue(uint8_t channel);

uint8_t writeData = 1;          /* Data that will be transmitted */
uint8_t receiveData;            /* Data that will be received */
volatile uint16_t ADC_val;
volatile uint16_t i = 0;
volatile uint8_t period;
volatile uint8_t play = 0;
volatile any_lcd_t lcd;
static const uint16_t sin_lut[16] =
{
0x200,0x2c3,0x369,0x3d8,0x3ff,0x3d8,0x369,0x2c3,
0x200,0x13c,0x96,0x27,0x0,0x27,0x96,0x13c,
};

uint16_t *lut = sin_lut;


static void ADCC_init(void)
{
    ADCON0 = _ADCON0_ADON_MASK     /*enable ADCC module*/
           | _ADCON0_ADCS_MASK     /*Select FRC clock*/
           | _ADCON0_ADFM_MASK;    /*result right justified*/
}

static void ADCC_dischargeSampleCap(void)
{
    ADPCH = DISCHARGE_SAMPLE_CAP;
}

static uint16_t ADCC_readValue(uint8_t channel)
{   
    ADPCH = channel; 
    
    ADCON0 |= _ADCON0_ADGO_MASK; /*start conversion*/

    while (ADCON0 & _ADCON0_ADGO_MASK)
    {
        ;
    }   
    
    return ((uint16_t)((ADRESH << 8) + ADRESL));
}

static void CLK_Initialize(void)
{
    OSCCON1bits.NOSC = 6;        /* HFINTOSC Oscillator */
    
    OSCFRQbits.HFFRQ = 8;        /* HFFRQ 64 MHz */
}

static void PPS_Initialize(void)
{
    RC3PPS = 0x0F;               /* SCK channel on RC3 */
 
    SSP1DATPPS = 0x14;           /* SDI channel on RC4 */
    
    RC5PPS = 0x10;               /* SDO channel on RC5 */
}

static void PORT_Initialize(void)
{
    TRISDbits.TRISD0 = 0; 
    TRISDbits.TRISD1 = 0; 
    TRISDbits.TRISD2 = 0; 
    TRISDbits.TRISD3 = 0; 
    TRISDbits.TRISD4 = 0; 
    TRISDbits.TRISD6 = 0;	
    TRISDbits.TRISD7 = 0;	
    TRISDbits.TRISD5 = 0;
    
    /* SDI as input; SCK, SDO, SS1, SS2 as output */
    TRISC = 0x17;
    
    /* SCK, SDI, SDO, SS1, SS2 as digital pins */
    ANSELC = 0x07;
    
    TRISAbits.TRISA6 = 0; //Set RA6 (RED LED) pin as output
    
    TRISAbits.TRISA7 = 0; //Set RA7 (GREEN LED) pin as output
    
    TRISEbits.TRISE0 = 0; //Set RB0 (button) pin as input
    
    TRISAbits.TRISA0 = 1; //Set RA0 to input
    ANSELAbits.ANSELA0 = 1; //Set RA0 to analog
    
    TRISAbits.TRISA2 = 1; //Set RA2 (button) pin as input
    WPUAbits.WPUA2 = 1; // Enable weak pull-up for pin RA2 (switch)
    ANSELAbits.ANSELA2 = 0; //Enable digital input buffer for pin RA2 (switch)
    
    TRISAbits.TRISA3 = 1; //Set RA3 (button) pin as input
    WPUAbits.WPUA3 = 1; // Enable weak pull-up for pin RA3 
    ANSELAbits.ANSELA3 = 0; //Enable digital input buffer for pin RA3 
    
    TRISAbits.TRISA4 = 1; //Set RA4 (button) pin as input
    WPUAbits.WPUA4 = 1; // Enable weak pull-up for pin RA4
    ANSELAbits.ANSELA4 = 0; //Enable digital input buffer for pin RA4
    
    TRISAbits.TRISA5 = 1; //Set RA5 (button) pin as input
    WPUAbits.WPUA5 = 1; // Enable weak pull-up for pin RA5
    ANSELAbits.ANSELA5 = 0; //Enable digital input buffer for pin RA5
   
}

static void SPI1_Initialize(void)
{  
    /* SSP1ADD = 1 */
   // SSP1ADD = 0x01;
    
    /* Enable module, SPI Host Mode */
    SSP1CON1 = _SSP1CON1_SSPEN1_MASK | 0x01;
    SSP1CON1bits.CKP = 1;
    SSP1STATbits.CKE = 0;
}

static void SPI1_client1Select(void)
{
    LATCbits.LATC6 = 0;          /* Set SS1 pin value to LOW */
}

static void SPI1_client1Deselect(void)
{
    LATCbits.LATC6 = 1;          /* Set SS1 pin value to HIGH */
}

static void SPI1_exchangeByte(uint8_t data)// uint8_t period)
{
    
    
    SSP1BUF = data;
    
    while(!PIR3bits.SSP1IF) 	/* Wait until data is exchanged */
    {
        ;
    }   
    PIR3bits.SSP1IF = 0;       // clears flag

    
}

void delayUS(uint32_t usec){
  for(uint32_t i = 0; i < usec; i++)
  {
    __delay_us(1); /* some kind of provided software delay */  
  }
}

void write_to_DAC(uint16_t data){
    SPI1_client1Select();
    SPI1_exchangeByte(0x00F0 | (data >> 6));
    SPI1_exchangeByte((data << 2) & 0x00FF);
    SPI1_client1Deselect();
    
}
    
static void TMR0_Initialize(){
       
    if(ADC_val <= 300){
        period = period * 2;
        LCD_WriteString(" OCT: 1", lcd);
        LATAbits.LATA6 = 1;
    }  
    else if(ADC_val >= 600){
        period = period / 2;
        LCD_WriteString(" OCT: 3", lcd);
        LATAbits.LATA7 = 1;
    }
    else{
        LCD_WriteString(" OCT: 2", lcd);
        LATEbits.LATE0 = 1; 
    }
    
    if(!PORTAbits.RA2){
        LCD_WriteString("NOTE: A", lcd);
        period = 0x11;}
    else if(!PORTAbits.RA5){ 
        LCD_WriteString("NOTE: D", lcd);
        period = 0x0C;}
    else if(!PORTAbits.RA4){ 
        LCD_WriteString("NOTE: C", lcd);
        period = 0x0E;}
    else if(!PORTAbits.RA3){ 
        LCD_WriteString("NOTE: B", lcd);
        period = 0x0F;}

       
    PIR0bits.TMR0IF = 0; //clear interrupt flag
    T0CON1 = 0b01111001;
    TMR0H = period;
    TMR0L = 0x00;
    PIR0bits.TMR0IF = 0;
    PIE0bits.TMR0IE = 1;
    T0CON0 = 0b10000000;
}

void TMR0_ISR()
{
    PIR0bits.TMR0IF = 0;
    write_to_DAC(sin_lut[i]);
    i += 1;
    if(i == 16){
        i = 0;
    }
}

void __interrupt() INTERRUPT_InterruptManager(void)
{
    if(PIE0bits.TMR0IE == 1 && PIR0bits.TMR0IF == 1){
        TMR0_ISR();
    }
}

int main(void)
{
    
    CLK_Initialize();
    PPS_Initialize();
    PORT_Initialize();
    SPI1_Initialize();
    ADCC_init();
    ADCC_dischargeSampleCap();
    //TMR0_Initialize();
    INTCONbits.PEIE = 1; //peripheral interrupt enable 
    INTCONbits.GIE = 1; //global interrupt enable
    
    /* ***LCD INIT****/
    lcd.delay_us  = &delayUS; /* provide pointers to necessary functions */
    lcd.lcd_write = &writeLCD;
    LCD_Init(lcd); //will initialize lcd "logically" ie send the correct commands for 4 bit mode 
    
    int i = 0;
    
    while(1)
    {
        
        
        if((!PORTAbits.RA2 || !PORTAbits.RA3 || !PORTAbits.RA4 || !PORTAbits.RA5) && !play)
        {
            ADC_val = ADCC_readValue(ANALOG_CHANNEL);
            LCD_Reset(lcd);
            TMR0_Initialize();
            play = 1;
        }
        
        if(PORTAbits.RA2 && PORTAbits.RA3 && PORTAbits.RA4 && PORTAbits.RA5)
        {
            T0CON0 = 0x0;
            play = 0;
            LCD_Reset(lcd);
            LATAbits.LATA7 = 0;
            LATAbits.LATA6 = 0;
            LATEbits.LATE0 = 0;

        }
        
    }
    
    
}

void writeLCD(uint8_t bitfield){
  /*****************
   ASSUMING:
   PORTD 7-4 -> LCD D 7-4
   PORTD 2 -> LCD E 
   PORTD 1 -> LCD RW 
   PORTD 0 -> LCD RS 
  ******************/	
  PORTD = bitfield;
} 