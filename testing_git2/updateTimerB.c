/*
 * updateTimerB.c
 *
 *  Created on: Nov. 8, 2021
 *      Author: Rinz
 */
#include <msp430.h>
#include <ucsiUart.h>
#include <updateTimerB.h>
#include <quadEncDec.h>
#include <ucsiUart.h>
#include <mdd_driver.h>

#include <string.h>
#include <stdio.h>
#include <math.h>

/**************************************
 * Function: void timerB0Init()
 *
 *purpose: Initialize timerA0 to the correct settings
 *purpose: also configure the port settings
 *
 *returns nothing
 **************************************/
void updateTimerBInit(){

    TB0CCR0 = 50000; //0xFFFE, 0.05sec*2(ID_1 divides by 2) = 0.1sec  = 10Hz update rate
    TB0CCR1 = 0;  // CCR1
    TB0CCTL1 |= (OUTMOD_7) // reset set mode
             | CM_0 // no capture
             | CCIE    // ie
             & ~CCIFG  // clr flags
             & ~COV;

    TB0EX0 &= 0x004;     // bits 000  divide by 5

    TB0CTL = (TBSSEL_2 | ID_3 | MC_1 | TBCLR); // Timer_B0 control register, SMCLK, ID_1 SMCLK/ , up mode

}
/**************************************
 * Function: void updateTimer()
 *
 *purpose: provide a 10Hz or 100000 clk cycle update function
 *         that will be used for updating pwm dutyCycles and updating the PID loop
 *
 *Created Nov 8 2021
 *Created by: Matt W
 *returns nothing
 **************************************/
void updateTimer(){

    _BIS_SR(GIE);

    volatile double error;
    volatile double angJ1Current;
    volatile double voutM1;
    volatile signed int sendPWM;
    volatile int dir1;


    volatile signed int error2;
    volatile signed int angJ2Current;
    volatile signed int voutM2;
    volatile signed int sendPWM2;



    if (enterLoop == 1){

        //------------------- M1 ------------------------
        doneM1 =0;

        // need to change posCount to long signed int
        angJ1Current = (posCount) * DEG_PER_PUL1;
        error = angJ1Desired - angJ1Current;


        if (error < 3  && error > -3) // uncertainty.
        {
            error = 0;
            enterLoop =0;
            doneM1 =1;
        }
        voutM1 = 0.025*error;
        if (voutM1 > 0 && voutM1 <0.5)
            voutM1 = 0.5;
        if (voutM1 < 0 && voutM1 >-0.5)
            voutM1 = -0.5;
        sendPWM = round(TRANS_FUNC_V_TO_PWM * voutM1);
        //!!!! change this, only should round up???


        if (error == 0) // stop here
            mddCW(0);

        dir1 = sendPWM/-1;
        if (dir1 >0)
        sendPWM = -1*sendPWM;

        if (sendPWM > MAX_PWM) // constrain max limits
           sendPWM = MAX_PWM;

        if (sendPWM > 0 && sendPWM <= MAX_PWM){ // min voltage condition cw
               if (sendPWM < MIN_VELOCITY && sendPWM > 0 && error != 0) // min speed cw
                   sendPWM  = MIN_VELOCITY;
               else if (sendPWM >= MAX_VELOCITY) // max speed cw
                   sendPWM = MAX_VELOCITY;
        }

        if (dir1 <0)
            mddCW(sendPWM);
        else
            mddCCW(sendPWM);
    }

        //------------------------ M2 -----------------------
    if (enterLoop2 == 1){

        doneM2 =0;

        // need to change posCount to long signed int
    //    angJ2Current = (posCount2) * DEG_PER_PUL;
        angJ2Current = (posCount2) * DEG_PER_PUL;
        error2 = (angJ2Desired) - angJ2Current;


        if (error2 < 3  && error2 > -3) // uncertainty.
        {
            error2 = 0;
            enterLoop2 =0;
            doneM2 =1;
        }

        sendPWM = error2 * SLOPE;
        sendPWM = sendPWM/100;

        if (error2 == 0) // stop here
            mddCW2(0);

        dir1 = sendPWM/-1;
        if (dir1 >0)
        sendPWM = -1*sendPWM;

        if (sendPWM > MAX_PWM) // constrain max limits
           sendPWM = MAX_PWM;

        if (sendPWM > 0 && sendPWM <= MAX_PWM){ // min voltage condition cw
               if (sendPWM < MIN_VELOCITY && sendPWM > 0 && error2 != 0) // min speed cw
                   sendPWM  = MIN_VELOCITY;
               else if (sendPWM >= MAX_VELOCITY) // max speed cw
                   sendPWM = MAX_VELOCITY;
        }

        if (dir1 <0)
            mddCW2(sendPWM);
        else
            mddCCW2(sendPWM);
    }
}

#pragma vector = TIMER0_B1_VECTOR
interrupt void timer0_B1_ISR(void){
// CCIFG is still set here and TB0IV = 0x02
    switch(__even_in_range(TB0IV,2)){ // reading TB0IV clears CCIFG, TB0R is counting up from zero now.
    //case 0: break; // nothing
    case 2:// TB0CCTL1 CCIFG
        updateTimer();
    break; // TA0CCR1

    default: break;
    }

//    TB0CCTL1 &= ~CCIFG; // CCR0 auto reset

}
