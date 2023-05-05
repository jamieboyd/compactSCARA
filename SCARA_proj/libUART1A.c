/*************************************************************************************************
 * libUART1A.c
 * - C implementation or source file for MSP430 usci UART A1
 * Has functions for sending/receiving from strings (null terminated character arrays)
 * and buffers (character arrays with specified lengths)
 *
 *  Author: Greg Scutt
 *  Created on: March 1, 2017
 *  Modified: February 26th, 2018
 *  Modified: 2022/01/13 by Jamie Boyd
 **************************************************************************************************/

#include <msp430.h>
#include "libUART1A.h"

char rxBuffer [RX_BUF_SZ]; // buffer that receive data from usciA1UARTgets. referenced in header so can be accessed easily
char intBuffer [LONG_INT_DEC_PLACES + 1];

rxIntFunc rxIntFuncPtr = NULL;
txIntFunc txIntFuncPtr = NULL;

//unsigned char (*rxIntFuncPtr)(char) = NULL; // pointer to function to run to get a byte from RXBUFF
//char (*txIntFuncPtr)(unsigned char*) = NULL; // pointer to function to run to transfer a byte into TXBUFF


/************************************************************************************
* Function: usciA1UartInit
* - configures UCA1 UART for 115200 baud, no parity, 8 bit data, LSB first, one stop bit
* Arguments: none
* return: nothing
* Author: Greg Scutt
* Date: March 1st, 2017
* Modified: 2022/01/10 by Jamie Boyd/Mathew Wonneburg for 20MHz clock, 115200 baud
************************************************************************************/
void usciA1UartInit(void){
    _UART_A1PSEL;                   // // macro selects special functions (TXD and RXD) for P4 pins 4 and 5 which connect to TXD, RXD jumpers
    UCA1CTL1 |= UCSWRST;            // Sets USCI A1  Software Reset Enabled bit in USC A1 CTL1 register.

    UCA1CTL1    |=  UCSSEL_2;       // sets bit 7 - selects SMCLK for BRCLK. User is responsible for setting this rate. 1.0485 MHz
    UCA1CTL1    &=  ~UCRXEIE      // clears bit 5 no erroneous char interrupt
                & ~UCBRKIE        // no break character interrupts
                &  ~UCDORM         // not dormant
                &  ~UCTXADDR       // just data, no addresses
                &  ~UCTXBRK;       // not a break

    UCA1CTL0     =  0;              // RESET UCA1CTL0 before new configuration
    UCA1CTL0    &=  ~UCPEN          // bit 7 clear means No Parity
                &   ~UCMSB          // bit 5 clear means LSB First
                &   ~UC7BIT         // bit 4 clear means 8 bits of data, not 7
                &   ~UCSPB          // bit 3 clear means 1 stop bit, not 2
                &   ~(UCMODE0 | UCMODE1) // bits 1 and 2 clear mean UART Mode
                &   ~UCSYNC;        // bit 0 clear means asynchronous mode
    // 115200 baud
    UCA1BR1 = 0x00;    // high byte of N   =  0
    UCA1BR0 = 0x0A;   // low byte of 65 -> 0x41
    UCA1MCTL =  UCOS16 + UCBRS_0 + UCBRF_14;   // oversampling mode enabled, first and second stage modulation
    UCA1CTL1 &= ~UCSWRST;        //  configured. take state machine out of reset.
}

/************************************************************************************
* Function: usciA1UartTxChar
* - writes a single character to UCA1TXBUF, first waiting until the write register UCA1TXBUF is empty
* Arguments:1
* argument1: txChar - byte to be transmitted
* return: none
* Author: Greg Scutt
* Date: March 1st, 2017
* Modified: 2022/01/10 by Jamie Boyd
************************************************************************************/
void usciA1UartTxChar(char txChar) {

    while (!(UCA1IFG & UCTXIFG)); // is this efficient ? No, it is polling, could use interrupt
        UCA1TXBUF = txChar;  // if TXBUFF ready then transmit a byte by writing to it
}

/************************************************************************************
* Function: usciA1UartTxString
* - writes a C string of characters, one char at a time to UCA1TXBUF by calling
*   usciA1UartTxChar. Stops when it encounters  the NULL character in the string
*   does NOT transmit the NULL character
* argument:
* Arguments:1
* argument1: txChar - pointer to char (string) to be transmitted
* return: number of characters transmitted
* Author: Greg Scutt
* Date: March 1st, 2017
* Modified: 2022/01/10 by Jamie Boyd
************************************************************************************/
int usciA1UartTxString(char* txChar){
    char* txCharLocal = txChar;       // make a local copy of txChar pointer
    while (*txCharLocal != '\0'){                 // pre-test for the terminating NULL, it is not transmitted
        usciA1UartTxChar (*txCharLocal);       // transmit one character at a time
        txCharLocal +=1;                       // increment local pointer
    }
    return (txCharLocal - txChar);             // returns number of characters sent
}

/************************************************************************************
* Function: usciA1UartTxBuffer
* - transmits bufLen characters from a text buffer
* Arguments:2
* argument1: buffer - unsigned char pointer to text buffer to be transmitted
* argument2: bufLen - integer number of characters transmitted
* return: number of bytes transmitted
* Author: Jamie Boyd
* Date: 2022/02/10
************************************************************************************/
int usciA1UartTxBuffer (char * buffer, unsigned int bufLen){
    unsigned int ii =0;
    for (ii =0; ii < bufLen; ii +=1){
        usciA1UartTxChar (buffer[ii]);
    }
    return ii;
}

/******************** usciA1UartUbyte **********************************************
* - writes a string representation of the decimal value of unsigned byte by doing the
* string conversion into a buffer and then calling usciA1UartTxBuffer.
* Arguments:1
*   theByte - an unsigned byte to be transmitted
* return: nothing
* Author: Jamie Boyd
* Date:2022/02/10  */
void usciA1UartUbyte (unsigned char theByte){
    signed char strPos;
    for (strPos = 2; strPos >= 0; strPos--){
        intBuffer [strPos] = '0' + (theByte % 10);
        theByte /= 10;
    }
    usciA1UartTxBuffer (intBuffer, 3);
}

/******************** usciA1UartSbyte **********************************************
* - writes a string representation of the decimal value of a long integer by doing the string
* conversion into a buffer and then calling usciA1UartTxBuffer.
* Arguments:1
* argument1: cntVal - a signed long integer to be transmitted
* return: nothing
* Author: Jamie Boyd
* Date:2022/02/10  */
void usciA1UartSbyte (signed char theByte){
    unsigned char strPos;
    if (theByte < 0){
        intBuffer [0] = '-';
        theByte *= -1;
    } else{
        intBuffer [0] = '+';
    }
    for (strPos = 3; strPos > 0; strPos--){
        intBuffer [strPos] = '0' + (theByte % 10);
        theByte /= 10;
    }
    usciA1UartTxBuffer (intBuffer, LONG_INT_DEC_PLACES + 1);
}



/******************** usciA1UartTxLongInt **********************************************
* - writes a string representation of the decimal value of a long integer by doing the string
* conversion into a buffer and then calling usciA1UartTxBuffer.
* argument1: cntVal - a signed long integer to be transmitted
* return: nothing
* Author: Jamie Boyd
* Date:2022/02/10  */
void usciA1UartTxLongInt (signed long cntVal){
    unsigned char strPos;
    if (cntVal < 0){
        intBuffer [0] = '-';
        cntVal *= -1;
    } else{
        intBuffer [0] = '+';
    }
    for (strPos = LONG_INT_DEC_PLACES; strPos > 0; strPos--){
        intBuffer [strPos] = '0' + (cntVal % 10);
        cntVal /= 10;
    }
    usciA1UartTxBuffer (intBuffer, LONG_INT_DEC_PLACES + 1);
}

/************************************************************************************
* Function: usciA1UartGets
* - receive a string entered from the console and store it into an array pointed to by rxString.
* argument1: rxString - unsigned char pointer to text buffer to put received characters into
* return:  pointer to rxString or NULL if unsuccessful (too many characters entered)
* Author: Jamie Boyd
* Date: 2022/02/10
************************************************************************************/
char * usciA1UartGets (char * rxString){
    unsigned char count;
    unsigned char rxVal;
    for (count =0; count < RX_BUF_SZ; count+=1){
        while (!(UCA1IFG & UCRXIFG));   // poll, waiting for a RX character to be ready
        rxVal = UCA1RXBUF;
        if (rxVal == '\r'){           // return was received
            rxBuffer [count] = '\0'; // add NULL termination
            break;
        }else{
            rxBuffer [count] = rxVal;
            usciA1UartTxChar (rxVal);   // echo entered character back to sender
        }
    }
    char * OutputStr;
    if (count < RX_BUF_SZ){     // we did not overflow buffer
        OutputStr = rxString;
        for (count +=1; count > 0; count -= 1){ // offset needed to avoid useless comparison with 0. Thanks compiler for flagging that
            OutputStr [count-1] = rxBuffer[count-1];
        }
    }else{
        OutputStr = NULL;
    }
    return OutputStr;
}

/************************************************************************************
* Function: usciA1UartInstallRxInt
* - saves a global pointer to a function to be run when a character has been received.
* Arguments:1
* argument1: interuptFuncPtr - pointer to a function that has a single char argument
* returns:nothing
* Author: Jamie Boyd
* Date: 2022/02/13
************************************************************************************/
void usciA1UartInstallRxInt (rxIntFunc rxFunc){
    rxIntFuncPtr =rxFunc;
}

/************************************************************************************
* Function: usciA1UartEnableRxInt
* - enables or disables interupts for character in Rx buffer
* Arguments:1
* argument1:isOnNotOFF - non-zero enables, 0 disables
* Author: Jamie Boyd
* Date: 2022/02/13
************************************************************************************/
void usciA1UartEnableRxInt (char isOnNotOFF){
    if (isOnNotOFF){
        UCA1IE |= UCRXIE;           // set receive enable bit in UART1 interrupt enable register.
    }else{
        UCA1IE &= ~UCRXIE;          // clear receive enable bit in UART1 interrupt enable register.
    }
}

/************************************************************************************
* Function: usciA1UartInstallTxInt
* - saves a global pointer to a function to be run when a character can be transmitter.
* Arguments:1
* argument1: interuptFuncPtr - pointer to a function that returns a single char
* returns: unsigned char that will be put in the Tx buffer
* Author: Jamie Boyd
* Date: 2022/02/13
************************************************************************************/
void usciA1UartInstallTxInt (txIntFunc txFunc){
    txIntFuncPtr = txFunc;
}

/************************************************************************************
* Function: usciA1UartEnableTxInt
* - enables or disables interrupts for Tx buffer ready for a character
* Arguments:1
* argument:isOnNotOFF - non-zero enables, 0 disables
* returns: nothing
* Author: Jamie Boyd
* Date: 2022/02/13
************************************************************************************/
void usciA1UartEnableTxInt (char isOnNotOFF){
    if (isOnNotOFF){
        UCA1IE |= UCTXIE;           // set transmit enable bit in UART1 interrupt enable register.
    }else{
        UCA1IE &= ~UCTXIE;           // clear transmit enable bit in UART1 interrupt enable register.
    }
}

/************************************************************************************
* Function: USCI_A1_ISR
* - Interrupt function for USCIA1 vector. Calls functions installed by usciA1UartInstallTxInt
* or usciA1UartInstallRxInt. So you had better install the function before enabling
* the corresponding interrupt
* Arguments:none
* returns: nothing
* Author: Jamie Boyd
* Date: 2022/02/13
* Modified: 2022/03/22 by Jamie Boyd added parameter or return val to indicate wake from low power mode
************************************************************************************/
#pragma vector = USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void) {
    unsigned char lpm =0;
  switch(__even_in_range(UCA1IV,4))
  {
  case 0:break;
  case 2:   // UCRXIFG - run installed function to deal with received character
      UCA1IV &= ~UCRXIFG;
      lpm =(*rxIntFuncPtr)(UCA1RXBUF);

    break;
  case 4:   //UCTXIFG - transmit character returned from installed function
      UCA1IV &= ~UCTXIFG;
      UCA1TXBUF =(*txIntFuncPtr)(&lpm);
      break;
  default: break;
  }
  if (lpm){
      __low_power_mode_off_on_exit();
  }
}

/************************************************************************************
* Function: echoInterrupt
* a simple Rx interrupt function that can be installed by usciA1UartInstallRxInt
* It echoes received characters back to host
* Arguments:1
* argument 1: the character in the Rx buffer
* returns: nothing
* Author: Jamie Boyd
* Date: 2022/02/13
************************************************************************************/
unsigned char  echoInterrupt (char  RXBUF){
    while (!(UCA1IFG & UCTXIFG)){};   // poll, waiting for an opportunity to send
    UCA1TXBUF = RXBUF;
    return 0;
}
