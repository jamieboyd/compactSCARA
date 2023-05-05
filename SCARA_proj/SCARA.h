/*
 * SCARA.h top-level includes and globals
 *
 *  Created on: May 4, 2022
 *      Author: Matthew Wonneberg, Jamie Boyd
 */

#include <msp430.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "UcsControl.h"             // Clock control
#include "quadEncDec.h"             // Quadrature decoder functions on port 2 interrupt
#include "PwmTimerA0.h"             // PWM motor signals on timer A0 P1.4 and 1.5 and motor direction on P3 0-4
#include "updateTimerB.h"           // timer for update function during a move
#include "movement.h"               // different types of movement - lines, joint interpolated moves, circles
#include "ZaxisCtrl.h"              // stepper motor for Z axis
#include "eStopLimitSwitch.h"       // runs emergency stop hardware
#include "libUART1A.h"              // runs serial port with interrupt-driven functions
#include "BinaryCmdInterp.h"        // sends and receives binary data over serial port



#ifndef SCARA_H_
#define SCARA_H_

void SCARA_failure(void);

unsigned char binInterp_zeroCount (unsigned char * inputData, unsigned char * outputResults);          // 0
unsigned char binInterp_getCount (unsigned char * inputData, unsigned char * outputResults);           // 1
unsigned char binInterp_setMtrs (unsigned char * inputData, unsigned char * outputResults);            // 2
unsigned char binInterp_eStop (unsigned char * inputData, unsigned char * outputResults);              // 3
unsigned char binInterp_eStopReset (unsigned char * inputData, unsigned char * outputResults);         // 4
unsigned char binInterp_zAxisGetPos (unsigned char * inputData, unsigned char * outputResults);         // 5
unsigned char binInterp_zAxisZero (unsigned char * inputData, unsigned char * outputResults);           // 6
unsigned char binInterp_zAxisSetUpper (unsigned char * inputData, unsigned char * outputResults);       // 7
unsigned char binInterp_zAxisSetUpperHere (unsigned char * inputData, unsigned char * outputResults);   // 8
unsigned char binInterp_zAxisSetLower (unsigned char * inputData, unsigned char * outputResults);       // 9
unsigned char binInterp_zAxisSetLowerHere (unsigned char * inputData, unsigned char * outputResults);   // 10
unsigned char binInterp_zAxisSetSpeed (unsigned char * inputData, unsigned char * outputResults);       // 11
unsigned char binInterp_zAxisGoToPos (unsigned char * inputData, unsigned char * outputResults);        // 12
unsigned char binInterp_zAxisJog (unsigned char * inputData, unsigned char * outputResults);            // 13
unsigned char binInterp_zAxisJogStop (unsigned char * inputData, unsigned char * outputResults);        // 14
unsigned char binInterp_moveJ (unsigned char * inputData, unsigned char * outputResults);               // 15
unsigned char binInterp_moveJ_Coord (unsigned char * inputData, unsigned char * outputResults);         // 16
unsigned char binInterp_moveL  (unsigned char * inputData, unsigned char * outputResults);              // 17
unsigned char binInterp_moveC (unsigned char * inputData, unsigned char * outputResults);               // 18

#endif /* SCARA_H_ */
