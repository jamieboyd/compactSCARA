/*
 * movement.h
 *
 *  Created on: Mar. 6, 2022
 *      Author: Rinz
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#define PI 3.1415
#define NUM_LINES 8           // number of lines for the main loop
#define LEFT_ARM_SOLUTION 1           // index that can be used to indicate left arm
#define RIGHT_ARM_SOLUTION 0           // index that can be used to indicate right arm
#define L1 15.24       // inner arm length
#define L2 15.24       // outer arm length

#define PUL_PER_DEG_N70 9.48866 // 3415.92pul/360deg
#define MAX_ABS_THETA1 110       // max angle of  arm 1
#define MAX_ABS_THETA1_PUL 1043
#define RELATIVE_THETA2 155 // degrees relative to arm one
#define RELATIVE_THETA2_PUL 1471 // 155 * PUL_PER_DEG_N70
#define MAX_ABS_THETA2 (180)       // max angle of outer arm relative to x axis
#define MAX_ABS_THETA2_PUL (MAX_ABS_THETA2 * PUL_PER_DEG_N70)
#define MAX_ABS_X 30.5      // max x value
#define MAX_ABS_Y 30.5       // max y value
#define MIN_ABS_X -30.5      // min x value
#define MIN_ABS_Y -30.5       // min y value
#define XY_RES_FACTOR 10
#define INNER_CIRCLE_BOUNDS 65 // 1000* 6.5cm // cm radius
#define TOOL_UP 0
#define TOOL_DOWN 1


#define DEG_PER_PUL_N70 0.10538896
#define T_UPDATE 0.005
#define W_MAX 354//360//708 // deg/s
#define W_MAX_PUL 3358
#define A_MAX 177//180//354 //deg/s^2
#define A_MAX_PUL 1679

#define A_MAX_LINEAR 23 // mm/s^2
#define V_MAX_LINEAR 47 // mm/s

#define MAX_ARRAY 801




// definition of a point
typedef struct POINT_2D {
    volatile float x;
    volatile float y;
}
POINT_2D;


// define the SCARA TCP and joint angles
typedef struct SCARA_POS{
    volatile float x, y;
    volatile float radius;
    volatile signed int theta1, theta2; // TCP coordinate and joint variables
    volatile int armSol;  // right(0) or left(1) arm solution
}SCARA_POS;

// define the SCARA tool (pen) position state
typedef struct SCARA_TOOL {
    volatile int penPos;         // pend state: up or down
}SCARA_TOOL;

// using above structures, define he SCARA ROBOT
typedef struct SCARA_ROBOT{
     SCARA_POS scaraPos;
     SCARA_TOOL scaraTool;
 }SCARA_ROBOT;

 // info needed to draw a line
 typedef struct LINE_DATA{
     POINT_2D pA;   // start point A
     POINT_2D pB;  // end point B
     volatile int numPts; // number of points on line (includes endpoints)
 }
 LINE_DATA;


 SCARA_ROBOT scaraStateEnd;
 SCARA_ROBOT scaraStateSet;

 LINE_DATA holdLine;
 LINE_DATA endLine;


 volatile signed int posArray1 [MAX_ARRAY];
 volatile signed int posArray2 [MAX_ARRAY];
 volatile unsigned int arrayLength;
 volatile unsigned int armSolChange;
 volatile unsigned int armSwitchSol;
 volatile unsigned int armSwitchSol2;
 volatile unsigned int moveJIndex;
 volatile unsigned int solutionMoveJ1;
 volatile unsigned int solutionMoveJ2;
 volatile float xHold;
 volatile float yHold;
 POINT_2D armChangeStart;
 POINT_2D armChangeEnd;
 volatile unsigned int noMove1;
 volatile unsigned int noMove2;

void sendMoveC(SCARA_ROBOT *scaraStateSolution);
void sendMoveL(SCARA_ROBOT *scaraStateSolution, LINE_DATA drawLine);
void sendMoveJ(SCARA_ROBOT scaraStateM2);
unsigned int moveJ(signed int endAng1, signed int endAng2);
int moveScaraL(SCARA_ROBOT* scaraState, LINE_DATA newLine);
int moveScaraC(SCARA_ROBOT* scaraState);
unsigned int scaraFkPulses(signed int pul1, signed int pul2, float* toolX, float* toolY);
unsigned int scaraIkPulses(signed int *ang1, signed int * ang2, double toolX, double toolY, SCARA_ROBOT *scaraState1);
SCARA_ROBOT scaraInitState(double x, double y, int armSol, char penState);
LINE_DATA initLine(double xA, double yA, double xB, double yB, int numPts);
double DegToRad(double);  // returns angle in radians from input angle in degrees
double RadToDeg(double);  // returns angle in degrees from input angle in radians
double PulToRad(double);
double RadToPul(double);



#endif /* MOVEMENT_H_ */
