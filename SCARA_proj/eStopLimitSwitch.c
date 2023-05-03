/*
 * eStopLimitSwitch.c
 *
 *  Created on: Apr. 13, 2022
 *      Author: Jamie Boyd/Matthew Wonneberg
 */
#include <msp430.h>
#include "eStopLimitSwitch.h"


volatile unsigned char gSTOP;


void eStopSetUp (void){

    P6DIR |= ESTOP_SOFT;   // set bit 6 high to send emergency stop signal from software
    P6OUT &= ~ESTOP_SOFT;   // emergency signal is low-to-high, so set low at start

    P6DIR |= ESTOP_RES;      // reset latch from software. high-low-high pulse resets the latch
    P6OUT &= ~ESTOP_RES ;
    __delay_cycles(100);
    P6OUT |= ESTOP_RES ;

    P6DIR |= ESTOP_LED;     // port 6 bit 1 set high to light LED for signalling ESTOP
    // P6OUT |= ESTOP_LED;
    P6OUT &= ~ESTOP_LED;

    P1DIR &= ~ESTOP_IN;      // enable ESTOPIN interrupt to go high-to-low when limit switch or estop pressed
    P1IE |= ESTOP_IN;
    P1IES |= ESTOP_IN;      // selects high-to-low interrupt
    P1IFG &= ~ESTOP_IN;

}


// P1 interrupt service routine - sets global variable when limit switch signal toggles
#pragma vector = PORT1_VECTOR
__interrupt void eStopInterrut(void) {
    switch(__even_in_range(P1IV,16)) {
    case  0: break;   // Vector  0: no interrupt
    case  2: break;   // Vector  2: P1.0
    case  4: break;   // Vector  4: P1.1
    case  6: break;   // Vector  6: P1.2
    case  8: break;   // Vector  8: P1.3
    case 10: break;   // Vector 10: P1.4
    case 12: break;   // Vector 12: P1.5
    case 14:  // Vector 14: P1.6
        gSTOP = 1;
        P6OUT |= ESTOP_LED;
        P1IFG &= ~ESTOP_IN;
        break;
    case 16: break;   // Vector 16: P1.7
    default: break;
    }
}

// set estop software input, which is then latched by hardware
void eStopSoftware (void){
    P6OUT |= ESTOP_SOFT;
}


void eStopReset (void){
    P6OUT &= ~ESTOP_SOFT;
    P6OUT &= ~ESTOP_RES;
    __delay_cycles(100);
    P6OUT |= ESTOP_RES;
    gSTOP = 0;
    P6OUT &= ~ESTOP_LED;
}
