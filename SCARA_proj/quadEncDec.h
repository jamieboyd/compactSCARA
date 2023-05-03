/*
 * quadEncDec.h
 *
 *  Created on: Mar. 22, 2022
 *      Author: Matthew Wonneberg, Jamie Boyd
 */

#ifndef QUADENCDEC_H_
#define QUADENCDEC_H_

#define CHAMASK 0xDF
#define CHBMASK 0xEF

#define CH2B BIT6
#define CH2A BIT7
#define CHA BIT5
#define CHB BIT4
#define CURRSTATE1 (P2IN & 0x30)
#define CURRSTATE2 (P2IN & 0xC0)

#define PORT2DIR P2DIR &= ~BIT4 &~BIT5 &~BIT6 &~BIT7;
#define CLEARFLAGS P2IFG = 0x00
#define INTERUPTEN P2IE |= (CHB | CHA | CH2B | CH2A) //


void quadEncInit();

volatile unsigned int prevState;
volatile unsigned int currentState;
volatile unsigned int dirStatus;
extern volatile signed int gPosCountL1;

volatile unsigned char preA;
volatile unsigned char preB;
volatile unsigned char currA;
volatile unsigned char currB;

volatile unsigned int prevState2;
volatile unsigned int currentState2;
volatile unsigned int dirStatus2;
extern volatile signed int gPosCountL2;

volatile unsigned char preA2;
volatile unsigned char preB2;
volatile unsigned char currA2;
volatile unsigned char currB2;

#endif /* QUADENCDEC_H_ */
