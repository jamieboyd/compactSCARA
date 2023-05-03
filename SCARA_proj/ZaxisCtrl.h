/*
 * ZaxisCtrl.h
 *
 *  Created on: Apr. 17, 2022
 *      Author: Jamie Boyd / Matthew Wonneberg
 */

#ifndef ZAXISCTRL_H_
#define ZAXISCTRL_H_

extern volatile signed int gZPos;

// total travel is about 90 mm. max number of steps is (90 * 32) = 2880
// resolution is 32 steps/mm. 4 steps/mm with 8x micro-stepping. But this code deals in steps, and host does the conversion
#define     ZAXIS_MAX_SPEED             800         // in steps/sec. max rated speed is 25 mm/sec = 800 steps/sec
#define     ZAXIS_STEP                  BIT0        // timer A 1 CCR1 output on P2.0
#define     ZAXIS_DIR                   BIT3        // P3.3
#define     ZAXIS_ENABLE                BIT5        // P3.5

#define     ZAXIS_UNDER_LOW             2            // error codes
#define     ZAXIS_OVER_HIGH             3            // for requested movements outside set max and min

extern volatile signed int gZpos;

typedef struct zAxisController{
    signed int upperLimit;      // upper limit of travel, must be set by user
    signed int lowerLimit;      // lower limit of travel, must be set by user
    signed int jogSpeed;        // calculated speed in steps/second when "jogging" Positive goes down, Negative goes up
    unsigned int movSpeed;      // speed in pulses/second, when moving a defined distance. No direction
    signed char curDir;         // 1 for currently moving downwards, 0 for moving upwards
    signed int movTarget;       // holds a target position we are moving toward

} zAxisController, * zAxisPtr;


void zAxisInit (void);
void zAxisZero(void);
void zAxisSetUpper (signed int limit);
void zAxisSetUpperHere(void);
void zAxisSetLower (signed int limit);
void zAxisSetLowerHere(void);
unsigned int zAxisSetSpeed (unsigned int speed);  // sets speed for upcoming moves, not direction. returns set speed steps/second
unsigned char zAxisGoToPos (signed int movPos);
void zAxisJog (signed int speed);        // speed AND direction at which to go until stopped, or a limit is hit
void zAxisJogStop (void);                  // stops movement on Z-axis

#endif /* ZAXISCTRL_H_ */
