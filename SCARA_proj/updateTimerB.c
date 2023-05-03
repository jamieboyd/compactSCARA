/*
 * updateTimerB.c
 *
 *  Created on: March 15, 2022
 *      Author: Matthew Wonneberg, Jamie Boyd
 */
#include <msp430.h>
#include <updateTimerB.h>
#include <quadEncDec.h>
#include <movement.h>

#include <stdio.h>
#include <math.h>
#include <PwmTimerA0.h>

/**************************************
 * Function: void timerB0Init()
 *
 *purpose: Initialize timerB0 to the correct settings
 *purpose: also configure the port settings
 *
 *returns nothing
 *Author: Matthew Wonneberg, Jamie Boyd
 **************************************/
void updateTimerBInit(){

    TB0CCR0 = 50000; // 0.0025sec*2(ID_2 divides by 2, EX0 divides by 1) = 0.005mS 200Hz
    TB0CCR1 = 0;  // CCR1 initialized to zero
    TB0CCTL1 |= (OUTMOD_7) // reset set mode
             | CM_0 // no capture
             | CCIE    // ie
             & ~CCIFG  // clr flags
             & ~COV;

    TB0EX0 &= 0x000;     // bits 000  divide by 1

    TB0CTL = (TBSSEL_2 | ID_2 | MC_1 | TBCLR); // Timer_B0 control register, SMCLK, ID_2 SMCLK/ , up mode

}
/**************************************
 * Function: void updateTimer()
 *
 *purpose: provides a function that updates every 0.005 seconds in order to operate the motors smoothly
 *         that will be used for updating pwm dutyCycles and updating the PID loop
 *
 *Created March 15 2022
 *Created by: Matthew Wonneberg, Jamie Boyd
 *returns nothing
 **************************************/
void updateTimer(){

    volatile signed int sendPWM;
    volatile unsigned int dir1 = 1;
    volatile signed int posError1;
    volatile signed int posError1Change;
    volatile signed int posError1Integral;

    volatile signed int sendPWM2;
    volatile unsigned int dir2 =1;
    volatile signed int posError2;
    volatile signed int posError2Change;
    volatile signed int posError2Integral;

    volatile signed int exit;

    // if startMoveJ is set, the move is commanded to start
    if (startMoveJ == 1){

        if (updateIndex >= arrayLength){ // exit condition, resets start bit, and applies the braking signal to both motors
            startMoveJ =0; // reset start bit
            updateIndex = 0;
            TA0CCR4 = 0; // send 0% PWM signals
            TA0CCR3 = 0;
            mddInputCtrl2(CTRLBRAKE2); // apply braking signals to motors
            mddInputCtrl(CTRLBRAKE);
            noMove1 = 0;// reset the no move condition variable
            noMove2 = 0;
            P2IFG &= ~0xF0; // clear flags due to quadrature encoder interrupts
            __disable_interrupt();
        }
        else{
            //-------------- Motor 1 ------------------------
            if (noMove1 == 0){// proceed if there is a move of the joint for joint 1

                posError1 = posArray1[updateIndex] - gPosCountL1; // motor 1 position error = setpoint - current position
                posError1Change = posError1 - prevError1; // delta position over time (position derivative)
                posError1Integral = posError1Sum; // summation of all errors throughout the move

                // apply gains to the output signal
                sendPWM = ((kP*posError1) + (kI*posError1Integral) + (kD*posError1Change));

                prevError1 = posError1; // store the current error for the derivative of position
                posError1Sum = posError1Sum + posError1; // update the sum of the errors for the integral of position

                if (sendPWM < 0){ // convert sendPWM to a posotive signal with a direction (dir1)
                    sendPWM = sendPWM*-1;
                    dir1 = 0; // ccw
                }
                if (sendPWM > MAX_VELOCITY) // constrain max PWM duty cycle limits
                   sendPWM = MAX_VELOCITY;

                if (dir1 == 1){ // send motor the speed signal based on direction
                    mddInputCtrl(CTRLCW);
                    timerA0DutyCycleSet(sendPWM); // send the previously pressed dutyCycle
                }
                else{
                    mddInputCtrl(CTRLCCW);
                    timerA0DutyCycleSet(sendPWM); // send the previously pressed dutyCycle

                }

            }
            //------------- Motor 2 ------------------------
            if(noMove2 ==0){// proceed if there is a move of the joint

                posError2 = posArray2[updateIndex] - gPosCountL2;// motor 2 position error = setpoint - current position
                posError2Change = posError2 - prevError2;// delta position over time (position derivative)
                posError2Integral = posError2Sum;// summation of all errors throughout the move

                // apply gains to the output signal
                sendPWM2 = ((kP*posError2) + (kI*posError2Integral) + (kD*posError2Change));

                prevError2 = posError2;// store the current error for the derivative of position
                posError2Sum = posError2Sum + posError2;// update the sum of the errors for the integral of position


                if (sendPWM2 < 0){ // convert sendPWM to a posotive signal with a direction (dir1)
                    sendPWM2 = sendPWM2*-1;
                    dir2 = 0; // ccw
                }

                if (sendPWM2 > MAX_VELOCITY) // constrain max PWM duty cycle limits
                   sendPWM2 = MAX_VELOCITY;

                if (dir2 == 1){ // send motor the speed signal based on direction
                    mddInputCtrl2(CTRLCW2);
                    timerA0DutyCycleSet2(sendPWM2); // send the previously pressed dutyCycle
                }
                else{
                    mddInputCtrl2(CTRLCCW2);
                    timerA0DutyCycleSet2(sendPWM2); // send the previously pressed dutyCycle
                }



                }
            updateIndex++;
        }


    }

}

#pragma vector = TIMER0_B1_VECTOR
interrupt void timer0_B1_ISR(void){

    switch(__even_in_range(TB0IV,2)){ // reading TB0IV clears CCIFG, TB0R is counting up from zero now.
    //case 0: break; // nothing
    case 2:// TB0CCTL1 CCIFG
        updateTimer();
    break;

    default: break;
    }

}

