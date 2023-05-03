/*
 * UartPwmTimerA0.c
 *
 *  Created on: March. 14, 2022
 *      Author: Matthew Wonneberg, Jamie Boyd
 */
#include <msp430.h>
#include <PwmTimerA0.h>
#include <updateTimerB.h>

/**************************************
 * Function: void timerA0Init()
 *
 * purpose: Initialize timers to the correct settings and also configure the port settings
 * for direction selections
 *
 *returns nothing
 *Author: Matthew Wonneberg, Jamie Boyd
 **************************************/
void timerA0Init(unsigned int pwmFreq){
    volatile unsigned char pwmFreqSetRet =0;

    TA0CCR4 = 0;  // capture compare register 1 initialized to 0%
    TA0CCR3 = 0;  // capture compare register 2 initialized to 0%
    TA0CCTL4 |= (OUTMOD_7); // reset set mode (output is reset when the timer counts to the TAxCCRn value, it is set when the time counts to the TAxCCR4 value
    TA0CCTL3 |= (OUTMOD_7); // reset set mode (output is reset when the timer counts to the TAxCCRn value, it is set when the time counts to the TAxCCR1 value
    TA0EX0 = 0x004;     // divide by 5

    pwmFreqSetRet = timerA0PwmFreqSet(pwmFreq); // set up the frequency of the PWM signal

    TA0CTL = (TASSEL_2 | ID_2 | MC_1); // Timer_A0 control register, SMCLK, SMCLK/4 , Up mode(timer counts up to CCR0

    P1OUT &= ~BIT5;   // Reset pin
    P1SEL |= BIT5;    // pin 1.2
    P1DIR = BIT5;    // output pin 1.2    TA0.1 pin

    P1DIR |= BIT4;    // output pin 1.2    TA0.1 pin
    P1SEL |= BIT4;    // pin 1.2
    P1OUT &= ~BIT4;   // Reset pin

    // INA, INB, SEL outputs
    P3OUT |= 0x00;
    P3DIR |= (BIT0 + BIT1 +BIT2 +BIT3 + BIT4); // pins set as output direction
    P3OUT &= (~BIT0 & ~BIT1 & ~BIT2 & ~BIT3 & ~BIT4); // P3out set to 0 (led's off)

}
/* ***************************************
 * timerA0PwmFreqSet(unsigned int pwmFreq)
 *
 * Computes and sets TACCR0 value using pwmFreq.  Does not perform clock division on SMCLK
 *
 * pwmFreqmin and max needs to be computed by designer.  If the requested pwmFreq is outside the
 * range then TACCR0 is not change and -1 is returned
 *
 * otherwise TACCR0 is computed to implement a pwm freqency equal to pwmFreq and 0 is returned
 *
 ****************************************/
char timerA0PwmFreqSet(unsigned int pwmFreq)
{
    volatile unsigned char result=0;

    if ((pwmFreq <= PWMFREQMAX) && (pwmFreq >= PWMFREQMIN)){
        TA0CCR0 = (20000000/pwmFreq)-1; // assign CCR0 value if possible that the timer will count up to
    }
    else{
        result =-1; // not possible
    }

    return result;
}

/* ***************************************
 * char timerA0DutyCycleSet(unsigned char dutyCycle)
 *
 * Computes and sets TACCRx register to the appropriate vallue using dutyCycle who's
 * range is integers 0:10
 *
 * returns 0.  if dutyCycle is not within the range 0:10 then return -1.
 *
 ****************************************/
void timerA0DutyCycleSet(unsigned char dutyCycle)
{
    volatile double percentDuty =0; // duty cycle as a percent

    if (dutyCycle <= MAX_VELOCITY && dutyCycle >= 0){

        percentDuty = (double)dutyCycle/(double)DUTY_INC;
        TA0CCR4 = (percentDuty*(TA0CCR0+1));

    }

}

/* ***************************************
 * char timerA0DutyCycleSet(unsigned char dutyCycle)
 *
 * Computes and sets TACCRx register to the appropriate vallue using dutyCycle who's
 * range is integers 0:10
 *
 * returns 0.  if dutyCycle is not within the range 0:10 then return -1.
 *
 ****************************************/
void timerA0DutyCycleSet2(unsigned char dutyCycle)
{
    volatile double percentDuty =0; // duty cycle as a percent

    if (dutyCycle <= MAX_VELOCITY && dutyCycle >= 0){

        percentDuty = (double)dutyCycle/(double)DUTY_INC;
        TA0CCR3 = (percentDuty*(TA0CCR0+1));

    }

}

/* ********************************
 * funciton: char mddInputCtrl(unsigned char ctrl)
 *
 * Purpose:  Sends cntrl to output if in range 0x0 -> 0x7
 *
 * return 0 if in range and -1 if not
 * author: Matthew Wonneberg, Jamie Boyd
 * Date: March 14 2022
 *********************************/
void mddInputCtrl(unsigned char ctrl)
{

    if ((ctrl & ~CTRLMASK) == ZEROVAL){ // check that cntrl is within 0x0 - 0x7
       CTRLPORT = ((CTRLMASK2 & CTRLPORT)+ctrl) ;   //send ctrl to output
    }

}
/* ********************************
 * funciton: char mddInputCtrl2(unsigned char ctrl)
 *
 * Purpose:  Sends cntrl to output for second motor driver if in range 0x0 -> 0x7
 *
 * return 0 if in range and -1 if not
 * author: Matthew Wonneberg, Jamie Boyd
 * date: March 14 2022
 *********************************/
void mddInputCtrl2(unsigned char ctrl)
{

    if ((ctrl & ~CTRLMASK2) == ZEROVAL){ // check that cntrl is within 0x0 - 0x7
        CTRLPORT = ((CTRLMASK & CTRLPORT)+ctrl) ;   //send ctrl to output
    }

}







