/*************************************************************************************************
 * ucsiUart.c
 * - C implementation or source file for MSP430 UCSI UART A1
 *
 *  Author: Greg Scutt
 *  Created on: March 1, 2017
 *  Modified: February 26th, 2018
 **************************************************************************************************/

#include <msp430.h>
#include "ucsiUart.h"
#include "mdd_driver.h"
#include <cmdInterpreter7070.h>
#include <string.h>


/************************************************************************************
* Function: ucsiA1UartInit
* - configures UCA1 UART to use SMCLK, no parity, 8 bit data, LSB first, one stop bit
*  BAUD rate = 19.2Kbps with 16xoversampling.
*  assumes SMCLK = 2^20 Hz.
* argument:
* Arguments: none, but baud rate would be useful
*
* return: none
* Author: Greg Scutt
* Date: March 1st, 2017
* Modified: <March 15, 2022>
* Modified by: Matthew Wonneberg, Jamie Boyd
************************************************************************************/
void ucsiA1UartInit(){

	// READ THIS --> You must use UCSI Control BITS defined in msp430.h !!
	// In fact you must use pre-defined control bits whenever possible. no obscure HEX codes allowed anymore.



    P4SEL |= TXD_A1 | RXD_A1;       // Peripheral module function is selected
    UCA1CTL1 |= UCSWRST;            // USCI A1  Software Reset Enabled
    //********************

    UCA1CTL1    |=  UCSSEL_2;       // select SMCLK. User is responsible for setting this rate.

    UCA1CTL0     =  0;              // RESET UCA1CTL0 before new configuration
    UCA1CTL0    &=  ~UCPEN          // No Parity  = 0x0
                &   ~UCMSB          // LSB First       &~ 0x1(MSB first) = LSB first
                &   ~UC7BIT         // 8 bits of data  &~ 0x1(7 bit data)  = 8 bits data
                &   ~UCSPB          // 1 stop bit      &~ 0x1(two stop bits) = one stop bit
                &   ~UCSYNC;        // UART Mode       &~ 0x1(sync) = ASYNC


    // only modify when UCSWRST =1

   /* UCA1BR1 = HIGHBYTE;    // high byte of N   =  54.6/16  = 3.41   -> 3  -> 0x0003  = 0x00
    UCA1BR0 = LOWBYTE;   // low byte of N = 0x0003 = 0x03
    UCA1MCTL =  UCOS16 + UCBRS_1 + UCBRF_6;*/   // oversampling mode enabled first and second stage modulation
  //           +  UCBRF    // First modulation stage select.  number of BITCLK16 clocks after last falling BITCLK edge
  ///          +  UCBRS;   // Second mod stage sel.  where m is placed


/*19200 baud    UCA1BR1 = 0x00;    // high byte of N   =  0
    UCA1BR0 = 0x41;   // low byte of 65 -> 0x41
    UCA1MCTL =  UCOS16 + UCBRS_0 + UCBRF_2;   // oversampling mode enabled first and second stage modulation
*/

    // 115200 baud
   UCA1BR1 = 0x00;    // high byte of N   =  0
    UCA1BR0 = 0x0A;   // low byte of 65 -> 0x41
    UCA1MCTL =  UCOS16 + UCBRS_0 + UCBRF_14;   // oversampling mode enabled first and second stage modulation


    UCA1CTL1    &= ~UCSWRST;        //  configured. take state machine out of reset.
	}


/************************************************************************************
* Function: ucsiA1UartTxChar
* - writes a single character to UCA1TXBUF if it is ready
* argument:
* Arguments: txChar - byte to be transmitted
*
* return: none
* Author: Greg Scutt
* Date: March 1st, 2017
* Modified: <March 15, 2022>
* Modified by: Matthew Wonneberg, Jamie Boyd
************************************************************************************/
void ucsiA1UartTxChar(unsigned char txChar) {

    unsigned char test = txChar;

    while (!(UCA1IFG & UCTXIFG)); // is this efficient ?  // ~UCTX 0 = T buff empty,  ~T buff empty, R buff empty
    UCA1TXBUF = txChar;  // if TXBUFF ready then transmit a byte by writing to it // clars TXIFG

}


/************************************************************************************
* Function: ucsiA1UartTxString
* - writes a C string of characters, one char at a time to UCA1TXBUF by calling
*   ucsiA1UartTxChar. Stops when it encounters  the NULL character in the string
*   does NOT transmit the NULL character
* argument:
* Arguments: txChar - pointer to char (string) to be transmitted
*
* return: number of characters transmitted
* Author: Greg Scutt
* Date: March 1st, 2017
* Modified: <March 15, 2022>
* Modified by: Matthew Wonneberg, Jamie Boyd
************************************************************************************/
int ucsiA1UartTxString(unsigned char *txChar){

    volatile unsigned int charReturn =0;
    volatile unsigned char sendChar;
    volatile unsigned int x=0;

    while (txChar[x] != NULLNUM){ // while txChar is not the last character

       sendChar = txChar[x];
       ucsiA1UartTxChar(sendChar); // send the character to the console

       if (sendChar != ' '){
           charReturn++;  // count the characters as long as they are not a space
       }
        x++;
    }

    return charReturn;
}
/***********************************
 * Function usciA1UartGets
 *
 * recieve a string entered from the console and store it into rxBuffer.
 * make a global array of 100 bytes called rxBuffer
 *
 * when the user calls this function, the program will wait, until the user enters a string and presses return.
 * then rxBuffer are copied into rxString
 *
 * use RXIFG as an interrupt to copy RXBUF -> rxBuffer[i]
 *
 * argurments: char *rxString
 *
 * returns: pointer to rxString or Null if unsucessful
 *Modified: <March 15, 2022>
* Modified by: Matthew Wonneberg, Jamie Boyd
 **********************************/
char usciA1UartGets(char *rxString){
    volatile unsigned int i=0;
    volatile unsigned int result =1;
    volatile unsigned char back =0;
    volatile unsigned int y =0;
    volatile unsigned char numChars;

    x=0;

   for (y; y <= BUFFLEN; y ++){ // set the buffer values to zero
       rxBuffer[y] =0;
   }

   while (!(UCA1IFG & UCTXIFG)); // wait for TxBuf to empty, then, ~UCTX 0 = T buff empty,  ~T buff empty, R buff empty
          UCA1TXBUF = '\n';      // send a new line
   while (!(UCA1IFG & UCTXIFG)); // wait for TxBuf to empty, then, ~UCTX 0 = T buff empty,  ~T buff empty, R buff empty
          UCA1TXBUF = '\r';      // send a return line
   UCA1IE &= ~UCRXIE;


      do{
          while(!(UCRXIFG & UCA1IFG));  // wait for console input flag
          rxBuffer[i] = UCA1RXBUF;      // update buffer

          while (!(UCA1IFG & UCTXIFG)); // wait for TxBuf to empty, then, ~UCTX 0 = T buff empty,  ~T buff empty, R buff empty
          UCA1TXBUF = rxBuffer[i];      // echo back to txBuf
          i++;

      }while (UCA1RXBUF != '\r' && x < 100);

      rxBuffer[i-=1] = NULL; // replace the \r with a NULL char
      i=0;

      strcpy(rxString, rxBuffer); // copy the finished buffer into rxString

    return result;// if unsuccessful
}


