 #include <msp430.h>
#include "libUART1A.h"
#include "BinaryCmdInterp.h"

// array of CMD structures
CMD gCMDs [CMD_LIST_SIZE];                                      // list of commands, i.e., function references plus argument size

// for getting user commands
unsigned char gCommands[BUFF_SIZE] [DATA_SIZE];                 // used to hold binary commands as sent by host, Python ,e.g.
volatile unsigned char gInCmd = 0;                              // for circular buffer of commands we are processing
volatile unsigned char gOutCmd = 0;                             // for circular buffer of commands we are processing
volatile unsigned char gCmdBufState = 0;                        // 0 means empty, 1 means inProgress, 2 means full

// for sending results from each command
unsigned char gResults [BUFF_SIZE] [DATA_SIZE];                 // results from commands, for sending back to host
volatile unsigned char gInRes =0;                               // for circular buffer of results we are processing
volatile unsigned char gOutRes =0;                              // for circular buffer of results we are processing
volatile unsigned char gResBufState = 0;                        // 0 means empty, 1 means inProgress, 2 means full


void binInterp_init (){
    usciA1UartInstallRxInt (&binInterp_RxInterupt);             // install UART interrupts
    usciA1UartInstallTxInt (&binInterp_TxInterrupt);
    usciA1UartEnableRxInt (1);                                  // enable Rx interrupt right away
    usciA1UartEnableTxInt (0);
}

unsigned char binInterp_addCmd (unsigned char nCharsIn, command commandFuncPtr){
    static unsigned char nCMDs=0;    // number of CMD structures in array, incremented when we add one
    gCMDs[nCMDs].theCommand = commandFuncPtr;
    gCMDs[nCMDs].nCharsIn = nCharsIn;
    nCMDs +=1;
    return nCMDs - 1;
}

/*********************************** binInterp_run *************************************************
* Function: binInterp_run
* - main loop. Goes into low power mode. When woken, processes commands till command buffer is empty
* Arguments: None
* returns: Nothing
* Author: Jamie Boyd
* Date: 2022/03/10
*************************************************************************************/
void binInterp_run (void){
    while (1){
        __low_power_mode_0();
        while (gCmdBufState > BUFF_EMPTY){
            binInterp_doNextCommand ();
        }
    }
}

/*************************************************************************************/
unsigned char binInterp_RxInterupt (char RXBUF){
   static unsigned int rCharCount = 255;       // count of characters in command buffer, 255 is flag value
   static unsigned char nChars;             // number of characters in this command - read this many
   unsigned char lpm = 0;                   // return value, will be set to 1 to wake from low power mode at end of a command
   unsigned char cmdCode;
   if (gCmdBufState < 2){                  // Buffer is not full, we can accept new commands
       if (rCharCount == 255){              // flag for starting a new command
           cmdCode = RXBUF;                 // first character is index into command array
           nChars = gCMDs[cmdCode].nCharsIn;    // number of characters this command is expecting
           rCharCount = 0;                      // reset flag to normal count
       }
       gCommands [gInCmd] [rCharCount++] = RXBUF;
       if (rCharCount == nChars){        // command fully entered
           lpm = 1;                            // set lpm to wake from low power mode
           rCharCount = 255;                    // set flag for next cmmand
           gInCmd += 1;                         // advance to next line in input buffer
           if (gInCmd == BUFF_SIZE){
                gInCmd = 0;
            }
            if (gCmdBufState == BUFF_EMPTY){
                gCmdBufState = BUFF_AVAIL;
            }
            if (gInCmd == gOutCmd){
                gCmdBufState = BUFF_FULL;
           }
       }
   }
  return lpm;
}


void binInterp_doNextCommand (void){
    // first byte received is index into CMD array
    unsigned char fCode = gCommands[gOutCmd] [0];
    CMD theCmd =  gCMDs [fCode];
    fCode = theCmd.theCommand (gCommands [gOutCmd], gResults[gInRes]);
    // increment out position in buffer of commands, cause we processed one
     gOutCmd += 1;
     if (gCmdBufState == BUFF_FULL){  //  command buffer state was full, now room for one more
         gCmdBufState = BUFF_AVAIL;
     }
     if (gOutCmd == BUFF_SIZE){
         gOutCmd = 0;
     }
     if (gOutCmd == gInCmd){
         gCmdBufState = BUFF_EMPTY;
     }
     if (fCode){ // then this function has something to say
         // increment in position in buffer of results to be sent, cause we added one
         gInRes +=1;
         if (gResBufState == BUFF_EMPTY){     // results buffer was empty, now has 1 message in it
             gResBufState = BUFF_AVAIL;
             usciA1UartEnableTxInt (1);
          }
         if (gInRes == BUFF_SIZE){
             gInRes = 0;
         }
         if (gInRes == gOutRes){
             gResBufState = BUFF_FULL;
         }
     }
}

/* - Called when it is enabled and TXBUF is empty. disables itself when result buffer is empty
* Arguments: 1
*   lpm  - pointer to an unsigned char which can be set to 1 to wake from low power mode
* returns: the next character from the global gErrStr
* Author: Jamie Boyd
* Date: 2022/03/16
************************************************************************************/
char binInterp_TxInterrupt (unsigned char* lpm){
    static unsigned char tCharCount=0;
    static unsigned char nChars =0;
    unsigned char rChar;
    if (tCharCount == 0){
        nChars =  gResults [gOutRes] [tCharCount++];
    }
    rChar = gResults [gOutRes] [tCharCount++];
    if (tCharCount >= nChars){
        tCharCount = 0;
        gOutRes += 1;
        if (gOutRes == BUFF_SIZE){  // wrap around to start of buffer
            gOutRes = 0;
        }
        if (gResBufState == BUFF_FULL){  // results buffer was full
            gResBufState = BUFF_AVAIL;    // but not any more
        }
        if (gOutRes == gInRes){  // buffer is empty
           gResBufState = BUFF_EMPTY;
           usciA1UartEnableTxInt (0);
        }
    }
    return rChar;
}
