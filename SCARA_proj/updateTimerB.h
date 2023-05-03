/*
 * updateTimerB.h
 *
 *  Created on: Nov. 8, 2021
 *      Author: Rinz
 */

#ifndef UPDATETIMERB_H_
#define UPDATETIMERB_H_

//------- loop and position variables---------

#define MAX_VELOCITY 50
#define MIN_VELOCITY 12

volatile signed int updateIndex;
volatile unsigned int startMoveJ;

volatile float kP;
volatile float kI;
volatile float kD;

volatile float kPAng;
volatile float kIAng;
volatile float kDAng;

volatile float kPLin;
volatile float kILin;
volatile float kDLin;


volatile signed int prevError1;
volatile signed int prevError2;
volatile signed int posError1Sum;
volatile signed int posError2Sum;



void updateTimerBInit();
void updateTimer();

#endif /* UPDATETIMERB_H_ */
