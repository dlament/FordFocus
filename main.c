/* 
 * File:   main.c
 * Author: damian
 *
 * Created on 1 stycznia 2018, 22:41
 */
// GITHUB edit

#include <stdio.h>
#include <stdlib.h>
#include "p18f1320.h"

// CONFIG1H
#pragma config OSC = INTIO2     // Oscillator Selection bits (Internal RC oscillator, port function on RA6 and port function on RA7)
#pragma config FSCM = OFF        //  (Fail-Safe Clock Monitor enabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal External Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bit (Brown-out Reset disabled)
// BORV = No Setting

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 1024     // Watchdog Timer Postscale Select bits (1:1024)

// CONFIG3H
#pragma config MCLRE = OFF      //  (RA5 input pin enabled, MCLR disabled)

// CONFIG4L
#pragma config STVR = OFF        // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Low-Voltage ICSP Enable bit (Low-Voltage ICSP enabled)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (00200-000FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (001000-001FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (00200-000FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (001000-001FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (00200-000FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (001000-001FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from table reads executed in other blocks)

/* defines */
#define ON 1
#define OFF 0
#define NUMBER_OF_READS 4
/* Globals */
unsigned char gState = 0;
unsigned char gLowBeamStability[NUMBER_OF_READS] = {0, 1, 0, 1};// 0,1,0,... to change the state after 4 reads (1s)
unsigned char gLowBeamState = OFF;
unsigned char gFogLightState = OFF;
unsigned char gFogLightCounter = 100;

/* functions */
void FogLightON(void)
{
  /* PortB3 -> IRF3710 -> Relay 30A
     1 = FogLightsON
     0 = FogLightsOFF
  */
  PORTBbits.RB3 = 1;      
}

void FogLightOFF(void)
{
  /* PortB3 -> IRF3710 -> Relay 30A
     1 = FogLightsON
     0 = FogLightsOFF
  */
  PORTBbits.RB3 = 0;      
}

void ClearWDG(void)
{
  _asm clrwdt  _endasm
}

#pragma code
#pragma interrupt InterruptHandlerHigh

void InterruptHandlerHigh ()
{
    unsigned char lightState = 0;
    unsigned char i = 0;
    unsigned char localLowBeamState = 0;
    
	if (INTCONbits.TMR0IF)
	{
        INTCONbits.TMR0IF = 0;            //clear interrupt flag
        
        if (gState < NUMBER_OF_READS)
        {
          gLowBeamStability[gState]= PORTBbits.RB2; //inverted logic: 1 = OFF
          gState++;
        }
        else
        {
          gState = 0;
          gLowBeamStability[gState]= PORTBbits.RB2; //inverted logic: 1 = OFF          
        }

        //check the stability of the low Beam lights
        for ( i = 0; i < NUMBER_OF_READS; i++ )
        {
          lightState += gLowBeamStability[i];
        }
        
        if (NUMBER_OF_READS == lightState) //inverted logic: PORTB = 1 -> OFF 
            localLowBeamState = OFF; 
        
        if (0 == lightState)
            localLowBeamState = ON; //inverted logic: 0 = ON
        
        if ( localLowBeamState != gLowBeamState )
        {
            // Prepare the FogLights to be turned on if LowBeam lights changes the state from ON to OFF
            // and foglights are OFF            
            if ( localLowBeamState == OFF )
            {
                if ( gFogLightState == OFF )
                    gFogLightCounter = 0;
            }
            else // localLowBeamState == ON
            {   
                // Turn On the FogLights if LowBeam lights changed the state from OFF to ON 
                // after previous change within ~2s

                if ( gFogLightCounter < 3 * NUMBER_OF_READS ) // ~2s to turn on the low beam lights +1s for detection the state
                {
                    if ( OFF == gFogLightState)
                    {
                        FogLightON();                  
                        gFogLightState = ON;
                    }
                    else
                    {
                        FogLightOFF();                  
                        gFogLightState = OFF;  
                    }
                }
                else
                {
                    FogLightOFF();                  
                    gFogLightState = OFF;  
                }
            }
        }        
        
        gLowBeamState = localLowBeamState;
        //------------------------------------
        // ~2s to turn on the low beam lights; then 1s for detection the state
        switch (gFogLightCounter)
        {             
            case 2 * NUMBER_OF_READS:
                if ( 1 == PORTBbits.RB2 && OFF == gFogLightState ) //not started the procedure to turn on the fog lights
                    PORTAbits.RA0 = 1; // Buzzer ON ->1; end of time slot to turn on the foglights
                break;                
            default:
                PORTAbits.RA0 = 0; // Buzzer OFF ->0                
                break;
        }     
        
        if ( gFogLightCounter < 100 )
          gFogLightCounter++;
        //------------------------------------
        INTCONbits.GIEH = 1;            //Global Interrupt Enable bit
    }
}

// High priority interrupt vector
#pragma code InterruptVectorHigh = 0x08
void InterruptVectorHigh (void)
{
  _asm
    goto InterruptHandlerHigh //jump to interrupt routine
  _endasm
}


void main(void) 
{
  OSCCONbits.IRCF = 4;//1MHz
    
  INTCONbits.GIEH = 1;   //enable interrupts HIGH 
  INTCONbits.TMR0IE = 1; //Timer0 enable interrupt
  INTCON2bits.TMR0IP = 1; //TMR0 interrupt high priority
  
  /* Port IO config*/
  TRISBbits.TRISB3 = 0;  // PortB3 -> IRF3710 -> Relay 30A
  TRISBbits.TRISB2 = 1;  // low beam lights -> PC817 -> PortB2
  PORTAbits.RA0 = 0;     // Buzzer OFF
  TRISAbits.TRISA0 = 0;  // TODO: PortA0 -> Buzzer 
  
  /* Timer0: 1MHz internal_CLK/4 = 250.000 ticks per 1s */
  /* Timer0(16bit) -> 250.000/ 2^16 -> 3.8 interrupts per 1s*/
  TMR0H = 0; //clear timer
  TMR0L = 0; //clear timer
  T0CONbits.TMR0ON = 1;
  T0CONbits.T08BIT = 0; //16bit
  T0CONbits.T0CS = 0;
  T0CONbits.PSA = 1; //prescaler is NOT assigned
  //---------------------------------------------

  FogLightOFF();
  
  while(1)
  {
    ClearWDG();       
  }
}

