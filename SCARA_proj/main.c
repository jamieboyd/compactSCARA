/**
 * main.c
 */

#include "SCARA.h"                  //



int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    _disable_interrupts();
    /*********Set main clock frequency to 20 MHz *********************************************/
    ucsSelSource(1,1,1,1);
    if (ucsDcoFreqSet (20, 2, 1)) SCARA_failure();
    //*************** motor control *********************************************/
    timerA0Init(PWMFREQ);        // initialize TimerA0 and port 3 pins for direction selection
    //*************** Quadrature Decoding *********************************************/
    quadEncInit();

    updateTimerBInit();

           prevState = CURRSTATE1; // read currentState
           preA = (P2IN & 0x20)>>5;
           preB = (P2IN & 0x10)>>4;

           prevState2 = CURRSTATE2; // read currentState
           P2IES = (prevState2 + prevState);
           preB2 = (P2IN & 0x40)>>6;
           preA2 = (P2IN & 0x80)>>7;

           P2IFG &= 0x00; // flags are cleared


   //--------------- Update Loop ----------------

           gPosCountL1 = 0;
           gPosCountL2 = 0;
           startMoveJ =0;
           noMove1 =0;
           noMove2 =0;
           armSolChange = 0;

           prevError1 = 0;
           prevError2 = 0;
           posError1Sum = 0;
           posError2Sum = 0;

           //  set control variables for angular moves vs linear moves
           kPAng = 2.2;
           kIAng = 0.03;
           kDAng = 0;

           kPLin = 2.2;
           kILin = 0.1;
           kDLin = 0.0;





           volatile signed int angleJ1;
           volatile signed int angleJ2;

           zAxisInit ();                       // initializes timer A 1 and GPIO pins for Z-axis control
           eStopSetUp ();                      // emergency stop


   //-----------UART CONTROL ------------------------
    usciA1UartInit();                   // Set up UART control and run binary UART commands in endless loop
    binInterp_init();
    binInterp_addCmd (1, binInterp_zeroCount);              //  0
    binInterp_addCmd (1, binInterp_getCount);               //  1
    binInterp_addCmd (6, binInterp_setMtrs);                //  2
    binInterp_addCmd (1, binInterp_eStop);                  //  3
    binInterp_addCmd (1, binInterp_eStopReset);             //  4
    binInterp_addCmd (1, binInterp_zAxisGetPos);            //  5
    binInterp_addCmd (1, binInterp_zAxisZero);              //  6
    binInterp_addCmd (4, binInterp_zAxisSetUpper);          //  7
    binInterp_addCmd (1, binInterp_zAxisSetUpperHere);      //  8
    binInterp_addCmd (4, binInterp_zAxisSetLower);          //  9
    binInterp_addCmd (1, binInterp_zAxisSetLowerHere);      //  10
    binInterp_addCmd (4, binInterp_zAxisSetSpeed);          //  11
    binInterp_addCmd (4, binInterp_zAxisGoToPos);           //  12
    binInterp_addCmd (4, binInterp_zAxisJog);               //  13
    binInterp_addCmd (1, binInterp_zAxisJogStop);           //  14
    binInterp_addCmd (6, binInterp_moveJ);                  //  15
    binInterp_addCmd (10, binInterp_moveJ_Coord);           //  16
    binInterp_addCmd (10, binInterp_moveL);                 //  17
    binInterp_addCmd (10, binInterp_moveC);                 //  18

       _enable_interrupts();

       binInterp_run ();                                    // endless loop of running commands
}

// better than nothing, blinking LED is some indication to user that things went south.
void SCARA_failure (void){
    P6DIR |= ESTOP_LED;
    while (1){
        P6OUT ^= ESTOP_LED;
        __delay_cycles(50000);  // 2 Hz, the angriest frequency
    }
}

