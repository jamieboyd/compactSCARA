/*
 * UartPwmTimerA0.h
 *
 *  Created on: Feb. 20, 2021
 *      Author: Rinz
 */

#ifndef PWMTIMERA0_H_
#define PWMTIMERA0_H_

//------- PWM conditions-----------------------------

#define PWMFREQMAX 20000     //18.935?  20kHz?
#define PWMFREQMIN 100       // 18.9 Hz
#define PWMFREQ 10000 // current pwm frequency

#define DUTY_INC 100

void timerA0Init(unsigned int pwmFreq);
char timerA0PwmFreqSet(unsigned int pwmFreq);
void timerA0DutyCycleSet(unsigned char dutyCycle);
void timerA0DutyCycleSet2(unsigned char dutyCycle);


//--------------- motor driver -----------------------

#define CTRLPORT P3OUT

#define CTRLMASK (BIT0 + BIT1)
#define ZEROVAL 0x00
#define INA BIT0 // P3.0
#define INB BIT1 // P3.1
#define CTRLOUT (P3DIR |= 0x3);

#define CTRLMASK2 (BIT4 + BIT2)
#define INA2 BIT4 // P3.3
#define INB2 BIT2 // P3.2
#define CTRLOUT2 (P3DIR |= BIT4+BIT2); (P3SEL |= BIT4+BIT2)

#define CTRLCW (CTRLMASK & (INA & ~INB))
#define CTRLCCW (CTRLMASK & (INB & ~INA))
#define CTRLBRAKE (CTRLMASK & (~INA & ~INB))

#define CTRLCW2 (CTRLMASK2 & (INA2 & ~INB2))
#define CTRLCCW2 (CTRLMASK2 & (INB2 & ~INA2))
#define CTRLBRAKE2 (CTRLMASK2 & (~INA2 & ~INB2))


//---- CW and CCW / brake global variabless--------------

void mddInputCtrl(unsigned char ctrl);
void mddInputCtrl2(unsigned char ctrl);




#endif /* PWMTIMERA0_H_ */
