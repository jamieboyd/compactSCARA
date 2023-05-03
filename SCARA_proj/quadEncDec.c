/*
 * quadEncDec.c
 *
 *  Created on: Mar. 22, 2022
 *      Author: Matthew Wonneberg, Jamie Boyd
 */

#include <msp430.h>
#include "quadEncDec.h"

volatile signed int gPosCountL1 = 0;
volatile signed int gPosCountL2 = 0;

// initialize ports and quadrature signal values
void quadEncInit(){

    PORT2DIR;
    CLEARFLAGS;
    INTERUPTEN;
    prevState = CURRSTATE1; // read currentState
    preA = (P2IN & 0x20)>>5;
    preB = (P2IN & 0x10)>>4;

    prevState2 = CURRSTATE2; // read currentState
    P2IES = (prevState2 + prevState);
    preB2 = (P2IN & 0x40)>>6;
    preA2 = (P2IN & 0x80)>>7;

    P2IFG &= 0x00; // flags are cleared
}


#pragma vector = PORT2_VECTOR // PORT2_VECTOR is defined in msp430.h
 __interrupt void Port2_ISR1 (void) // Port 2 interrupt service routine
{

     if (currentState2 != CURRSTATE2){ // check if motor 2 interrupts went off
         currentState2 = (P2IN & 0xC0); // select the bits in reference to motor 2
         currB2 = (currentState2 & 0x40)>>6; // shift low bit into currB2 variable
         currA2 = (currentState2 & 0x80)>>7; // shift high bit into currA2 variable
         P2IES = ((currentState+currentState2) & 0xF0);// edge select for both motors
             if (currB2 ^ preA2){ // CCW
                 gPosCountL2--; // decrease the position by 1 pulse
                 dirStatus2 =1;
             }
             else{
                 gPosCountL2++;  // CW increase the position by 1 pulse
                 dirStatus2 =0;
             }
             preA2 = currA2; // only need to update previousA because we are not using preB
         P2IFG &= ( ~BIT6 & ~BIT7); // flags are cleared when exiting routine
     }
     else if (currentState != CURRSTATE1){ // check if motor 1 interrupts went off
     currentState = (P2IN &0x30); // select the bits in reference to motor 1
     currA = (currentState & 0x20)>>5; // /32
     currB = (currentState & 0x10)>>4; // /16
     P2IES = ((currentState+currentState2) & 0xF0);
         if (currB ^ preA){ // CCW
             gPosCountL1--; // decrease the position by 1 pulse
             dirStatus =1;
         }
         else{
             gPosCountL1++;  // CW increase the position by 1 pulse
             dirStatus =0;
         }
         preA = currA; // only need to update previousA because we are not using preB
     P2IFG &= ( ~BIT4 & ~BIT5 ); // flags are cleared when exiting routine
     }
     else
         P2IFG &= ~0xF0; // clear all flags if stuck


}
