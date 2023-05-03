/*
 * Uartmddhpi.c
 *
 *  Created on: March 8, 2022
 *      Author: Matthew Wonneberg, Jamie Boyd
 */

#include <mdd_driver.h>
#include <cmdInterpreter7070.h>
#include <msp430.h>
#include <ucsiUart.h>
#include <quadEncDec.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <PwmTimerA0.h>
/* ********************************
 * funciton: void displayPos
 *
 * Purpose:  displays the current position of motor 1
 *
 * return 0 if in range and -1 if not
 * author: Matthew Wonneberg, Jamie Boyd
 * Date: March 14 2022
 *********************************/
void displayPos(){
    volatile char posPrint[20] = {0}; // Uart set up character array
    volatile int ret;

       sprintf(posPrint, "\nCount = %d \n\r", gPosCountL2); // insert the number of characters into the display string
       ret = ucsiA1UartTxString(&posPrint); // print the string

}

/* ********************************
 * funciton: void displayPos2
 *
 * Purpose:  displays the current position of motor 2
 *
 * return 0 if in range and -1 if not
 * author: Matthew Wonneberg, Jamie Boyd
 * Date: March 14 2022
 *********************************/
void displayPos2(){
    volatile char posPrint[20] = {0};
    volatile int ret;

       sprintf(posPrint, "\nCount = %d \n\r", posCount2); // insert the number of characters into the display string
       ret = ucsiA1UartTxString(&posPrint); // print the string

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
char mddInputCtrl(unsigned char ctrl)
{
    volatile signed char result =0;

    if ((ctrl & ~CTRLMASK) == ZEROVAL){ // check that cntrl is within 0x0 - 0x7
       CTRLPORT = ((CTRLMASK2 & CTRLPORT)+ctrl) ;   //send ctrl to output
    }
    else{
        result =-1; // if ctrl is out of range
    }

    return result;
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
char mddInputCtrl2(unsigned char ctrl)
{
   volatile signed char result =0;

    if ((ctrl & ~CTRLMASK2) == ZEROVAL){ // check that cntrl is within 0x0 - 0x7
        CTRLPORT = ((CTRLMASK & CTRLPORT)+ctrl) ;   //send ctrl to output
    }
    else{
        result =-1; // if ctrl is out of range
    }

    return result;
}


