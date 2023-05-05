/*
 * eStopLimitSwitch.h
 *
 *  Created on: Apr. 13, 2022
 *      Author: Jamie Boyd/Matthew Wonneberg
 *
 *      Code for limit switches and emergency stop. The limit switches, physical emergency stop
 *      button, and an optoisolator used as a GPIO-driven switch are wired in series as normally closed.
 *      The GPIO-driven optoisolator acts as a an emergency stop triggerable from software.
 *      3.3 V is applied on one end of the chain, and the other output is used as the "set" input to a latching circuit
 *      made up of two NAND gates. When any switch in the series is interrupted, even momentarily, the outputs of the
 *      latch toggle, and  maintain their state until the reset input is pulsed from high to low to high. The output of the
 *      latch goes directly to the enable outputs of the arm drive motors and the Z stepper motor, so they are immediately stopped
 *      The output also triggers a software interrupt that sets a global gSTOP variable, so that all movement commands will
 *      stop executing and return. The stopped signal from the latch is reset by a GPIO output from the microcontroller,
 *      that is, it can only be reset from software.
 *
 */

#ifndef ESTOPLIMITSWITCH_H_
#define ESTOPLIMITSWITCH_H_

#define     ESTOP_IN        BIT6        // port 1 bit 6 input for hardware emergency stop. High is good. low is stop
#define     ESTOP_RES       BIT5        // port 6 bit 5 to reset emergency stop. keep it high, low-high to reset
#define     ESTOP_SOFT      BIT6        // port 6 set bit 6 high to send emergency stop signal from software
#define     ESTOP_LED       BIT1        // port 6 bit 1 set high to light LED for signaling ESTOP

extern volatile unsigned char gSTOP;

void eStopSetUp (void);
void eStopSoftware (void);
void eStopReset (void);
#endif /* ESTOPLIMITSWITCH_H_ */
