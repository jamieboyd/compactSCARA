/*************************************************************************************************
 * libUART1A
 * - library for setting up and controlling a serial port on an MSP430 using USCI UART A1
 * Has functions for initializing at different Bauds,
 * and for sending/receiving from strings (null terminated character arrays)
 * or buffers (character arrays with specified lengths)
 *
 *  Author: Greg Scutt
 *  Created on: March 1, 2017
 *  Modified: February 26th, 2018
 *  Modified: 2022/01/13 by Jamie Boyd  - completed or added many functions
 *  Modified: 2022/01/23 by Jamie Boyd made into a library
 **************************************************************************************************/

#ifndef INCLUDE_LIBUART1A_H_
#define INCLUDE_LIBUART1A_H_

/************************************** Defines ***************************************************/
#define     TXD_A1          BIT4                        // A1 UART Transmits Data on P4.4
#define     RXD_A1          BIT5                        // A1 UART Receives Data on P4.5
#define     _UART_A1PSEL    P4SEL |= TXD_A1 | RXD_A1    // example of using macros for short expressions.

#define     RX_BUF_SZ       100                         // size for buffer to receive characters from terminal

#ifndef NULL                                            // NULL is defined in stdint.h, but it the only thing we use from there
#define NULL 0
#endif
#define LONG_INT_DEC_PLACES 10    // biggest signed long int is a 10 digit number used to TX a big decimal number


typedef unsigned char (*rxIntFunc)(char);
typedef char (*txIntFunc)(unsigned char*);

/******************************* Function Headers **********************************/
/* Function: usciA1UartInit
* - configures UCA1 UART to use SMCLK, no parity, 8 bit data, LSB first, one stop bit
* - assumes SMCLK = 2^20 Hz
* Arguments: 1
* argument 1: Baud, an msp430 supported baud, 16x over-sampling is used if supported for the Baud
* return: 1 if a supported Baud was requested, else 0
* Author: Greg Scutt
* Date: March 1st, 2017
* Modified: 2022/01/10 by Jamie Boyd */
void usciA1UartInit(void);

void usciA1UartUbyte (unsigned char theByte);
void usciA1UartSbyte (signed char theByte);

/* Function: usciA1UartTxChar
* - writes a single character to UCA1TXBUF, first waiting until the write register UCA1TXBUF is empty
* Arguments:1
* argument1: txChar - byte to be transmitted
* return: none
* Author: Greg Scutt
* Date: March 1st, 2017
* Modified: 2022/01/10 by Jamie Boyd */
void usciA1UartTxChar(char txChar);

/*Function: usciA1UartTxString
* - writes a C string of characters, one char at a time to UCA1TXBUF by calling
*   usciA1UartTxChar. Stops when it encounters the NULL character in the string
*   does NOT transmit the NULL character
* Arguments:1
* argument1: txChar - pointer to char (string) to be transmitted
* return: number of characters transmitted
* Author: Greg Scutt
* Date: March 1st, 2017
* Modified: 2022/01/10 by Jamie Boyd */
int usciA1UartTxString(char* txChar);


/*Function: usciA1UartTxLongInt
* - writes a string representation of a long integer by doing the string
* conversion into a buffer and then calling usciA1UartTxBuffer.
* Arguments:1
* argument1: cntVal - a signed long integer to be transmitted
* return: nothing
* Author: Jamie Boyd
* Date:2022/02/10  */
void usciA1UartTxLongInt (signed long cntVal);

/* Function: usciA1UartTxBuffer
* - transmits bufLen characters from a text buffer
* Arguments:2
* argument1: buffer - unsigned char pointer to text buffer to be transmitted
* argument2: bufLen - integer number of characters transmitted
* return: number of bytes transmitted
* Author: Jamie Boyd
* Date: 2022/02/10  */
int usciA1UartTxBuffer (char * buffer, unsigned int bufLen);

/* Function: usciA1UartGets
* - receive a string entered from the console and store it into an array pointed to by rxString.
* Arguments:1
* argument1: rxString - unsigned char pointer to text buffer to put received characters into
* return:  pointer to rxString or NULL if unsuccessful (too many characters entered)
* Author: Jamie Boyd
* Date: 2022/02/10  */
char * usciA1UartGets (char * rxString);

/* Function: usciA1UartInstallRxInt
 * - saves a global pointer to a function to be run when a character has been received.
 * Arguments:1
 * argument1: interuptFuncPtr - pointer to a function that has a single char argument
 * returns:nothing
 * Author: Jamie Boyd
 * Date: 2022/02/13 */
void usciA1UartInstallRxInt (unsigned char(*interuptFuncPtr)(char RXBUF));

/* Function: usciA1UartEnableRxInt
* - enables or disables interupts for character in Rx buffer
* Arguments:1
* argument1:isOnNotOFF - non-zero enables, 0 disables
* Author: Jamie Boyd
* Date: 2022/02/13 */
void usciA1UartEnableRxInt (char isOnNotOFF);

/* Function: usciA1UartInstallTxInt
* - saves a global pointer to a function to be run when a character can be transmitter.
* Arguments:1
* argument1: interuptFuncPtr - pointer to a function that returns a single char
* returns: unsigned char that will be put in the Tx buffer
* Author: Jamie Boyd
* Date: 2022/02/13  */
void usciA1UartInstallTxInt (char(*interuptFuncPtr)(unsigned char*));

/* Function: usciA1UartEnableTxInt
* - enables or disables interrupts for Tx buffer ready for a character
* Arguments:1
* argument:isOnNotOFF - non-zero enables, 0 disables
* returns: nothing
* Author: Jamie Boyd 
* Date: 2022/02/13   */
void usciA1UartEnableTxInt (char isOnNotOFF);

/* Function: echoInterrupt
* a simple Rx interrupt function that can be installed by usciA1UartInstallRxInt
* It echoes received characters back to host
* Arguments:1
* argument 1: the character in the Rx buffer
* returns: nothing
* Author: Jamie Boyd
* Date: 2022/02/13 */
unsigned char echoInterrupt (char  RXBUF);


#endif /* INCLUDE_LIBUART1A_H_ */
