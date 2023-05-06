/*
 * movement.c
 *
 *  Created on: Mar. 6, 2022
 *      Author: Matthew Wonneberg, Jamie Boyd
 */

#include <msp430.h>
#include "quadEncDec.h"
#include "PwmTimerA0.h"
#include "updateTimerB.h"
#include "movement.h"

#include <stdio.h>
#include <string.h>
#include <math.h>


/***********************************************************
* Name: void sendMoveJ
* function: used to perform the steps necessary to perform a joint interpolated move
* arguments
*            SCARA_ROBOT *scaraStateEnd:  pointer variable of type SCARA_ROBOT (this holds two angles that the robot will use for its joints)
*
* returns N/A
* created by: Matthew Wonneberg, Jamie Boyd
* Date: April 15  2022
************************************************************/
void sendMoveJ(SCARA_ROBOT scaraStateEnd){
    unsigned int exit;

    // calculate position arrays for the joint interpolated move
    exit = moveJ(scaraStateEnd.scaraPos.theta1,scaraStateEnd.scaraPos.theta2); // start end M1, start end M2;
    if (exit == 0){ // if the calculations were successful, then send them to the robot
        // restore the control variables to the values defined for angular movess
        kP = kPAng;
        kI = kIAng;
        kD = kDAng;
        __enable_interrupt();
        _BIS_SR(GIE);
        // set the variable to start the updating of the motors
        startMoveJ = 1;
        while (startMoveJ == 1){} // wait for the move to finish

    }

}

/***********************************************************
* Name: unsigned int moveJ
* function: used to perform the calculationss necessary to perform a joint interpolated move
* arguments
*            signed int endAng1:  the desired angle of joint two (L2)
*            signed int endAng2:  the desired angle of joint one (L1)
*
* returns exit
* created by: Matthew Wonneberg, Jamie Boyd
* Date: March 10  2022
************************************************************/
unsigned int moveJ(signed int endAng1, signed int endAng2){

    volatile unsigned int exit =0;
    volatile unsigned int x=0;
    volatile unsigned int displacement1;
    volatile unsigned int displacement2;
    volatile signed int direction1 = 0; // 0 is negative, 1 is positive direction
    volatile signed int direction2 = 0; // 0 is negative, 1 is positive direction
    volatile signed int deltaD;
    volatile signed int deltaD2;
    volatile float timeForMove;
    volatile float vMaxMove;
    volatile float aMaxMove;
    volatile float vMaxMove2;
    volatile float aMaxMove2;
    volatile unsigned int masterJoint;

    volatile unsigned int tInc = 0;
    volatile float w = 0;

    volatile signed int endAng1n = endAng1;
    volatile signed int endAng2n = endAng2;

    if (endAng1n <= MAX_ABS_THETA1_PUL && endAng1n >= -MAX_ABS_THETA1_PUL){ // verify range
        if (endAng2n <= MAX_ABS_THETA2_PUL && endAng2n >= -MAX_ABS_THETA2_PUL){ // verify range
            displacement1 = abs(endAng1n - gPosCountL1);
            displacement2 = abs(endAng2n - gPosCountL2);

            // determine if a no-move condition is requested
            if (gPosCountL1 < (endAng1n+9) && gPosCountL1 > (endAng1n-9))
                noMove1 = 1;
            else
                noMove1 =0;

            if (gPosCountL2 < (endAng2n+9) && gPosCountL2 > (endAng2n-9))
                noMove2 =1;
            else
                noMove2 =0;


            // determine which joint has to move the farthest and bases calculations on that joint

            if (displacement1 >= displacement2){ // L1 has to move farther
                masterJoint =1;
                deltaD = (endAng1n - gPosCountL1);
                deltaD2 = (endAng2n - gPosCountL2);
                if (endAng1n >= gPosCountL1)
                    direction1 = 1; // positive angle for L1
                else
                    direction1 = 0; // negative angle for L1
                if (endAng2n >= gPosCountL2)
                    direction2 = 1;
                else
                    direction2 = 0;
            }
            else{
                masterJoint =2; // L2 has to move farther
                deltaD = (endAng2n - gPosCountL2);
                deltaD2 = (endAng1n - gPosCountL1);
                if (endAng1n >= gPosCountL1)
                    direction2 = 1; // positive angle for L2
                else
                    direction2 = 0; // negative angle for L2
                if (endAng2n >= gPosCountL2)
                    direction1 = 1;
                else
                    direction1 = 0;
            }



            timeForMove = sqrt((abs(deltaD)*2*PI)/A_MAX_PUL); // calculate period (T) for sinusoidal profile
            w = (2*PI)/timeForMove;
            vMaxMove = (2*A_MAX_PUL)/w; // calc the Vmax for the move

            // check if you are within the velocity limit, otherwise you have to recalculate using the set max velocity
            if (vMaxMove > W_MAX_PUL){
                vMaxMove = W_MAX_PUL;
                timeForMove = (2*abs(deltaD))/(W_MAX_PUL);
                aMaxMove = (abs(deltaD)*w)/(timeForMove);
                aMaxMove2 = (abs(deltaD2)*w)/(timeForMove);
            }
            else{ // if the velocity is within the limit, then calculate acceleration values for each motor
                aMaxMove = abs(A_MAX_PUL);
                aMaxMove2 = ((abs(deltaD2)*w)/timeForMove);
            }

            //-------------- assign variables-------------------
            if (masterJoint == 1){ // if arm one moves further, then calculate the array length based on it
                w = (2*PI)/timeForMove;
                arrayLength = (timeForMove/T_UPDATE)+1;

                if (direction1 == 0) // relative to which joint moves the furthest
                    aMaxMove = -1*aMaxMove;
                if (direction2 == 0)
                    aMaxMove2 = -1*aMaxMove2;

                for(tInc; tInc<arrayLength; tInc++){
                    posArray1[tInc] = RadToPul((PulToRad(aMaxMove)*(tInc*T_UPDATE))/w)  -  RadToPul(PulToRad(aMaxMove)*(sin(w*(tInc*T_UPDATE)))/pow(w,2))+gPosCountL1;
                    posArray2[tInc] = RadToPul((PulToRad(aMaxMove2)*(tInc*T_UPDATE))/w)  -  RadToPul(PulToRad(aMaxMove2)*(sin(w*(tInc*T_UPDATE)))/pow(w,2))+gPosCountL2;
                    if ((posArray2[tInc] > (posArray1[tInc]+ RELATIVE_THETA2_PUL)) || (posArray2[tInc] < (-RELATIVE_THETA2_PUL + posArray1[tInc]))){ // on the fly max theta2 value verification
                        exit = 1; // error
                        tInc = arrayLength;// exit the calculation
                    }
                }
            }
            else if (masterJoint == 2){ // if arm two moves further
                w = (2*PI)/timeForMove;
                arrayLength = (timeForMove/T_UPDATE)+1;

                if (direction1 == 0)
                    aMaxMove = -1*aMaxMove;
                if (direction2 == 0)
                    aMaxMove2 = -1*aMaxMove2;

                for(tInc; tInc<arrayLength; tInc++){
                    posArray2[tInc] = RadToPul((PulToRad(aMaxMove)*(tInc*T_UPDATE))/w)  -  RadToPul(PulToRad(aMaxMove)*(sin(w*(tInc*T_UPDATE)))/pow(w,2))+gPosCountL2;
                    posArray1[tInc] = RadToPul((PulToRad(aMaxMove2)*(tInc*T_UPDATE))/w)  -  RadToPul(PulToRad(aMaxMove2)*(sin(w*(tInc*T_UPDATE)))/pow(w,2))+gPosCountL1;
                    if ((posArray2[tInc] > (posArray1[tInc]+ RELATIVE_THETA2_PUL)) || (posArray2[tInc] < (-RELATIVE_THETA2_PUL + posArray1[tInc]))){ // on the fly max theta2 value verification
                        exit = 1;// error
                        tInc = arrayLength; // exit the calculation
                    }
                }
           }
        }
        else
            exit = 1;
    }
    else
        exit = 1;

    return (exit);
}

/***********************************************************
* Name: void sendMoveL
* function: used to perform the calculationss necessary to perform a linear move with the TCP
* arguments
*            SCARA_ROBOT *scaraStateSolution:  holds the desired arm solution.
*            LINE_DATA drawLine:  holds desired pB.x and pB.y
*
* returns result
* created by: Matthew Wonneberg, Jamie Boyd
* Date: April 15  2022
************************************************************/
void sendMoveL(SCARA_ROBOT *scaraStateSolution, LINE_DATA drawLine){

    volatile unsigned int result =0;
    volatile signed int angleJ1;
    volatile signed int angleJ2;
    volatile unsigned int originalArmSolution;

    // store the original arm solution in case a solution change is required
    originalArmSolution = scaraStateSolution->scaraPos.armSol;

    __disable_interrupt();
    // calculate the line from the user defined variables
    result = moveScaraL(scaraStateSolution, drawLine);

    if(result == 0){ // the motor position arrays were calculated correctly, perform the move

        kP = kPLin; // set gain control variables to linear movement specifications
        kI = kILin;
        kD = kDLin;
        __enable_interrupt();
        _BIS_SR(GIE);
        __delay_cycles(10000);
        // set the variable to start the updating of the motors
        startMoveJ=1;

        while (startMoveJ == 1){}// wait until the move finishes

    }
    else if(result == 3){ // if the calculation was unsuccessful due to a arm solution change, then the desired line is split into two lines

        result = moveScaraL(scaraStateSolution, holdLine); // after first arm solution was not successfull, calculate part of the first line with the original arm solution
        if(result == 0){
            // move for the first line
            kP = kPLin; // set gain control variables to linear movement specifications
            kI = kILin;
            kD = kDLin;
            __enable_interrupt();
            _BIS_SR(GIE);
            __delay_cycles(10000);
            // set the variable to start the updating of the motors
            startMoveJ=1;
            // wait until the move finishes
            while (startMoveJ == 1){}

            /**********TOOLUP************/

            // after the first move is performed with the original arm solution, switch solutions
            if (originalArmSolution == LEFT_ARM_SOLUTION)
                scaraStateSolution->scaraPos.armSol = RIGHT_ARM_SOLUTION;
            else
                scaraStateSolution->scaraPos.armSol = LEFT_ARM_SOLUTION;

            // calculate the new joint angles for the new arm solution, at the same coordinate
            result = scaraIkPulses(&angleJ1, &angleJ2, endLine.pA.x, endLine.pA.y, scaraStateSolution);

            if (result == 0){
                // store new solution joint angles for the same spot at switch point of desired line,
                 scaraStateEnd.scaraPos.theta1 = angleJ1;
                 scaraStateEnd.scaraPos.theta2 = angleJ2;

                // send command to perform switching arm solutions
                sendMoveJ(scaraStateEnd);

                // perform the second part of the full line with the new arm solution
                if (result == 0){
                    // store the start point and endpoint of the second line for the full move
                    endLine = initLine(armChangeEnd.x, armChangeEnd.y, armChangeStart.x, armChangeStart.y, 0);//xb yb xa ya npts
                    // calculate the position array's for the move
                    result = moveScaraL(scaraStateSolution, endLine);

                    // perform the second smaller line move that makes up the user desired line
                    if(result == 0){
                        kP = kPLin;// set gain control variables to linear movement specifications
                        kI = kILin;
                        kD = kDLin;
                        __enable_interrupt();
                        _BIS_SR(GIE);
                        __delay_cycles(10000);
                        // set the variable to start the updating of the motors
                        startMoveJ=1;
                        // wait until the move finishes
                        while (startMoveJ == 1){}

                    }
                }
            }

        }
    }

}


/***********************************************************
* Name: int moveScaraL
* function: used to perform the steps necessary to calculate a linear move with the TCP
* arguments
*            SCARA_ROBOT *scaraState:  pointer variable of type SCARA_ROBOT that holds the desired arm solution
*            LINE_DATA newLine: newLine contains end (x, y) coordinate in the line
*
* returns value
* created by: Matthew Wonneberg, Jamie Boyd
* Date: March 17 2022
************************************************************/
int moveScaraL(SCARA_ROBOT* scaraState, LINE_DATA newLine){

    volatile unsigned int value=0;
    volatile unsigned int returned =0;

    volatile float deltaX;
    volatile float deltaY;
    volatile float deltaD;
    volatile unsigned int armSolution;

    volatile float timeForMove;
    volatile float w = 0;
    volatile float d = 0;
    volatile float xHoldPrev =0;
    volatile float yHoldPrev =0;
    volatile unsigned int attemptedArmSolution;
    volatile signed int holdPosition1 = 0;
    volatile signed int holdPosition2 = 0;
    volatile unsigned int aMaxMove;

    volatile unsigned int tInc = 0;

    volatile signed int desiredAngJ1;
    volatile signed int desiredAngJ2;


    SCARA_ROBOT scaraStateSet;
    SCARA_ROBOT scaraStateEnd;

    value = scaraFkPulses(gPosCountL1, gPosCountL2, &newLine.pA.x, &newLine.pA.y);
    attemptedArmSolution = scaraState->scaraPos.armSol; // store the attempted arm solution

    // arm solution change condition for first point
    // calc joint angles for arm solution beginning x,y
    value = scaraIkPulses(&desiredAngJ1, &desiredAngJ2, newLine.pA.x, newLine.pA.y, scaraState);// lowercase k
    // compare within one degree
    if (gPosCountL1 > (desiredAngJ1+9) || gPosCountL1 < (desiredAngJ2-9) || gPosCountL2 > (desiredAngJ2+9) || gPosCountL2 < (desiredAngJ2-9)){
        // if different, movJ to the same point with the correct arm solution
        //********TOOL_UP****************
        scaraStateEnd.scaraPos.theta1 = desiredAngJ1;
        scaraStateEnd.scaraPos.theta2 = desiredAngJ2;
        sendMoveJ(scaraStateEnd);
        //*********TOOL_RETURN***********
    }
    if (armSolChange == 1){ // change initial arm solution if starting with non possible solution
        if (attemptedArmSolution == LEFT_ARM_SOLUTION){
            attemptedArmSolution = RIGHT_ARM_SOLUTION;
            scaraState->scaraPos.armSol = RIGHT_ARM_SOLUTION;
            armSolChange = 0;
        }
        else{
            attemptedArmSolution = LEFT_ARM_SOLUTION;
            scaraState->scaraPos.armSol = LEFT_ARM_SOLUTION;
            armSolChange = 0;
        }
    }

    // calculate line displacements
    deltaX = newLine.pB.x - newLine.pA.x;
    deltaY = newLine.pB.y - newLine.pA.y;
    deltaD = (sqrt(pow(deltaX, 2) + pow(deltaY, 2))); // Pythagorean theorem for line distance in x,y

    timeForMove = (sqrt((abs(deltaD)*2*PI)/A_MAX_LINEAR));
    w = (2*(PI))/timeForMove;
    arrayLength = (timeForMove/T_UPDATE)+1;
    aMaxMove = A_MAX_LINEAR;

    // fill the position and velocity array targets
    for(tInc; tInc < arrayLength; tInc++){

        // calculate linear array in terms of d(t) and then fill X and Y positions
        d = ((aMaxMove * (tInc*T_UPDATE))/w -  (aMaxMove)*(sin(w*(tInc*T_UPDATE)))/pow(w,2));
        xHold = newLine.pA.x + (d*(deltaX)/deltaD);
        yHold = newLine.pA.y + (d*(deltaY)/deltaD);

        if ((abs(xHold*10) < INNER_CIRCLE_BOUNDS) && (abs(yHold*10) < INNER_CIRCLE_BOUNDS)){ // inner circle violation exit condition
            value = 1;
            tInc = arrayLength;
        }
        else{
            returned = scaraIkPulses(&posArray1[tInc], &posArray2[tInc], xHold, yHold, scaraState);
        if (returned == 0){
            if (armSolChange == 1){ // determine if a armSolution change was needed

                // hold values to be used for arm solution change
                armChangeStart.x = xHoldPrev;
                armChangeStart.y = yHoldPrev;
                armChangeEnd.x = newLine.pB.x;
                armChangeEnd.y = newLine.pB.y;
                holdLine = initLine(xHoldPrev, yHoldPrev, newLine.pA.x, newLine.pA.y, 0);//xb yb xa ya npts
                endLine = initLine(newLine.pB.x, newLine.pB.y, armChangeStart.x, armChangeStart.y, 0);

                scaraState->scaraPos.armSol = attemptedArmSolution; // return to original arm solution
                armSolChange =0;
                return(3);
            }
            xHoldPrev = xHold; // store the previous x,y values incase an arm solution change is needed
            yHoldPrev = yHold;

            if ((posArray2[tInc] > (posArray1[tInc]+RELATIVE_THETA2_PUL)) || (posArray2[tInc] < (-RELATIVE_THETA2_PUL + posArray1[tInc]))){ //  theta2 value verification
                value = 1;
                tInc = arrayLength; // exit loop due to error
            }
        }
        else{
            value = 1; // exit calculations if the move is not possible
            tInc = arrayLength;
        }
      }
    }


    return (value);
}
/***********************************************************
* Name: void sendMoveC
* function: used to perform the steps necessary send a circular move with the tool
* arguments
*            SCARA_ROBOT *scaraStateSolution:  pointer variable of type SCARA_ROBOT that holds the
*
*
* returns value
* created by: Matthew Wonneberg, Jamie Boyd
* Date: April 13 2022
************************************************************/
void sendMoveC(SCARA_ROBOT *scaraStateSolution){

    volatile unsigned int result =0;
    volatile signed int angleJ1;
    volatile signed int angleJ2;
    volatile unsigned int originalArmSolution;
    SCARA_ROBOT midState;

    // store the user defined arm solution
    originalArmSolution = scaraStateSolution->scaraPos.armSol;

    // calculate the position array's of each joint to create the arc
    __disable_interrupt();
    result = moveScaraC(scaraStateSolution);

    if(result == 0){ // perform the move if the calculation was successful and no arm solution change was necessary
        kP = kPLin; // set control gain variables to linear move settings
        kI = kILin;
        kD = kDLin;
        __enable_interrupt();
        __delay_cycles(10000);
        _BIS_SR(GIE);
        // set the start bit to send positions to the motors
        startMoveJ=1;

        while (startMoveJ == 1){} // wait until the move is finished

    }
    else if(result == 3){ // if an armSolution change is necessary

        result = moveScaraC(scaraStateSolution); // after first arm solution was not successfull, calculate part of the first arc with the original arm solution
        if(result == 0){ // if the calculation was successful, perform the first arc

            kP = kPLin;// set control gain variables to linear move settings
            kI = kILin;
            kD = kDLin;
            __enable_interrupt();
            __delay_cycles(10000);
            _BIS_SR(GIE);
            // set the start bit to send positions to the motors
            startMoveJ=1;

            while (startMoveJ == 1){} // wait until the move is finished

            /**********TOOLUP************/

            // assign the other arm solution
            if (originalArmSolution == LEFT_ARM_SOLUTION)
                scaraStateSolution->scaraPos.armSol = RIGHT_ARM_SOLUTION;
            else
                scaraStateSolution->scaraPos.armSol = LEFT_ARM_SOLUTION;

            // calculate joint angles for the same (x, y) coordinate, but with a new arm solution
            result = scaraIkPulses(&midState.scaraPos.theta1, &midState.scaraPos.theta2, armChangeStart.x, armChangeStart.y, scaraStateSolution);

            if (result == 0){
                // assign the joint angles for the new arm solution
                // perform the moveJ to the same coordiante, just with switched arm solutions
                sendMoveJ(midState); // start end M1, start end M2;

                /**********TOOLRETURN************/
                // perform final arc
                if (result == 0){
                    scaraStateSet.scaraPos.theta1 = scaraStateEnd.scaraPos.theta1; // set to midpoint of arc
                    scaraStateEnd.scaraPos.theta1 = scaraStateSet.scaraPos.theta2; // restore original end position

                    // calculate the position array's of each joint to create the arc
                    result = moveScaraC(scaraStateSolution);
                    if(result == 0){ // if the calculation is successful, perform the move
                        kP = kPLin;
                        kI = kILin;
                        kD = kDLin;
                        __enable_interrupt();
                        __delay_cycles(10000);
                        _BIS_SR(GIE);
                        startMoveJ=1;
                        while (startMoveJ == 1){}

                    }
                }
            }

        }
    }

}


/***********************************************************
* Name: int moveScaraC
* function: used to calculate a circular move with the tool
* arguments
*            SCARA_ROBOT *scaraState:  pointer variable of type SCARA_ROBOT that holds the desired arm solution
*
*
*
* returns value
* created by: Matthew Wonneberg, Jamie Boyd
* Date: April 13 2022
************************************************************/


int moveScaraC(SCARA_ROBOT* scaraState){

    volatile unsigned int value=0;
    volatile unsigned int returned =0;

    volatile float Xb, Yb, Xc, Yc;
    volatile float deltaXa, deltaYa, deltaXb, deltaYb;

    volatile float deltaD;
    volatile float resultant;
    volatile unsigned int armSolution;
    volatile signed int direction;

    volatile float timeForMove;
    volatile float arcAngle;
    volatile float holdArcAngle;
    volatile float w = 0;
    volatile float d = 0;
    volatile float xHoldPrev =0;
    volatile float yHoldPrev =0;
    volatile float j1HoldAng =0;
    volatile float j2HoldAng =0;

    volatile unsigned int attemptedArmSolution;
    volatile unsigned int aMaxMove;
    volatile unsigned int tInc = 0;
    volatile signed int desiredAngJ1;
    volatile signed int desiredAngJ2;
    volatile signed int thetaStart;
    volatile signed int thetaEnd;
    volatile unsigned int radius;

    thetaStart = scaraStateSet.scaraPos.theta1;
    thetaEnd = scaraStateEnd.scaraPos.theta1+9; // one degree is added, noticed that arc's were falling one degree behind
    radius = scaraStateSet.scaraPos.radius;

    // determine the direction to draw the circle
    if (thetaStart > thetaEnd){
        direction = -1; // negative direction / cw
    }
    else
        direction = 1; // positive direciton / ccw

    attemptedArmSolution = scaraState->scaraPos.armSol; // store the attempted arm solution
    value = scaraFkPulses(gPosCountL1, gPosCountL2, &scaraStateSet.scaraPos.x, &scaraStateSet.scaraPos.y);// lowercase k)

     // arm solution change condition for first point
     // calc joint angles for arm solution beginning x,y
     value = scaraIkPulses(&desiredAngJ1, &desiredAngJ2, scaraStateSet.scaraPos.x, scaraStateSet.scaraPos.y, scaraState);// lowercase k
     // compare within one degree
     if (gPosCountL1 > (desiredAngJ1+9) || gPosCountL1 < (desiredAngJ2-9) || gPosCountL2 > (desiredAngJ2+9) || gPosCountL2 < (desiredAngJ2-9)){
         // if different, movJ to the same point with the correct arm solution
         //********TOOL_UP****************
         scaraStateEnd.scaraPos.theta1 = desiredAngJ1;
         scaraStateEnd.scaraPos.theta2 = desiredAngJ2;

         sendMoveJ(scaraStateEnd);
         //*********TOOL_RETURN***********
     }
     if (armSolChange == 1){ // change initial arm solution if starting with non possible solution
         if (attemptedArmSolution == LEFT_ARM_SOLUTION){
             attemptedArmSolution = RIGHT_ARM_SOLUTION;
             scaraState->scaraPos.armSol = RIGHT_ARM_SOLUTION;
             armSolChange = 0;
         }
         else{
             attemptedArmSolution = LEFT_ARM_SOLUTION;
             scaraState->scaraPos.armSol = LEFT_ARM_SOLUTION;
             armSolChange = 0;
         }
     }

    // find delta xy between the current coordinate of the robot and the arc center position
    deltaXa = radius*cos(PulToRad(thetaStart));
    deltaYa = radius*sin(PulToRad(thetaStart));

    // find the center (x, y) coordiante of the arc
    Xc = scaraStateSet.scaraPos.x - deltaXa;
    Yc = scaraStateSet.scaraPos.y - deltaYa;

    // find the delta xy between the center coordiante of the arc and the end coordinate of the arc
    deltaXb = radius*cos(PulToRad(thetaEnd));
    deltaYb = radius*sin(PulToRad(thetaEnd));

    // find the end (x, y) coordiante of the arc
    Xb = Xc-deltaXb;
    Yb = Yc-deltaYb;

    deltaD = radius*PulToRad(thetaEnd - thetaStart); // find the linear distance of the arc that is requested

    // calculate the time for the move
    timeForMove = (sqrt((abs(deltaD)*2*PI)/A_MAX_LINEAR));
    w = (2*(PI))/timeForMove;

    // calculate the arrayLength and set the acceleration to the maximum TCP acceleration
    arrayLength = (timeForMove/T_UPDATE)+1;
    aMaxMove = A_MAX_LINEAR;

    // fill the position and velocity array targets
    for(tInc; tInc < arrayLength; tInc++){

        // calculate linear array in terms of d(t) and then fill X and Y positions
        d = ((aMaxMove * (tInc*T_UPDATE))/w -  (aMaxMove)*(sin(w*(tInc*T_UPDATE)))/pow(w,2));

        // calculate the arc angle with respect to time, while accounting for direction
        if (direction == 1)
            arcAngle = ((d/deltaD)*(thetaEnd - thetaStart) )+ thetaStart;
        else
            arcAngle = ((d/deltaD)*(thetaStart - thetaEnd) )+ thetaStart;

        // calculate the (x, y) coordinate for the corisponding circle angle wrt time
        xHold = radius*cos(PulToRad(arcAngle)) + Xc;
        yHold = radius*sin(PulToRad(arcAngle)) + Yc;

        if ((abs(xHold*10) < INNER_CIRCLE_BOUNDS) && (abs(yHold*10) < INNER_CIRCLE_BOUNDS)){ // inner circle violation exit condition
            value = 1;
            tInc = arrayLength;
        }
        else{
            returned = scaraIkPulses(&posArray1[tInc], &posArray2[tInc], xHold, yHold, scaraState);

            if (returned == 0){
                if (armSolChange == 1){ // determine if a armSolution change was needed

                    // hold values to be used for arm solution change
                    armChangeStart.x = xHoldPrev;
                    armChangeStart.y = yHoldPrev;

                    scaraStateSet.scaraPos.theta2 = thetaEnd;
                    scaraStateEnd.scaraPos.theta1 = holdArcAngle;

                    scaraState->scaraPos.armSol = attemptedArmSolution; // return to original arm solution
                    armSolChange =0;
                    return(3);
                }
                xHoldPrev = xHold; // store the previous x,y values incase an arm solution change is needed
                yHoldPrev = yHold;
                holdArcAngle = arcAngle;

                // varialbe range joint angle 2 verification
                if ((posArray2[tInc] > (posArray1[tInc]+RELATIVE_THETA2_PUL)) || (posArray2[tInc] < (-RELATIVE_THETA2_PUL + posArray1[tInc]))){
                    value = 1;
                    tInc = arrayLength;
                }
            }
            else{// exit calculations if the move is not possible
                value = 1;
                tInc = arrayLength;
            }
        }
    }


    return (value);
}


/***********************************************************
* Name: unsigned int scaraFkPulses
* function: provides the calculations to control the robot with forward kinematics
* using motor pulses instead of angles to reduce floating point calculations so that time can be saved
* arguments
*            ang1:  signed int - theta1
*            ang2:  signed int - theta2
*            toolX: pointer type double - points to x position of arm
*            toolY: pointer type double - points to y position of arm
*
* created by: Matthew Wonneberg, Jamie Boyd
* Date: March 24 2020
* Modified: April 22 2022
************************************************************/
unsigned int scaraFkPulses(signed int pul1, signed int pul2, float* toolX, float* toolY){

    volatile unsigned int exit = 0;
    volatile float toolX1, toolY1;
    volatile double thetaB, a;

    if (abs(pul1) > MAX_ABS_THETA1_PUL) // if theta1 is over the limit return 1
        exit = 1;
    else if (abs(pul2) > MAX_ABS_THETA2_PUL) // if theta2 is over the limit return 1
        exit =1;
    else
    {
        toolY1 = (L1 * sin(PulToRad(pul1))) + (L2 * sin(PulToRad(pul2))); // eqn for y coordinate
        toolX1 = (L1 * cos(PulToRad(pul1))) + (L2 * cos(PulToRad(pul2))); // eqn for x coordinate

        if (toolX1 <= MAX_ABS_X && toolX1 >= MIN_ABS_X){ // check within range
            *toolX = toolX1;   //assign X position
        }
        if (toolY1 <= MAX_ABS_Y && toolY1 >= MIN_ABS_Y){ // check within range
            *toolY = toolY1;   //assign Y position
        }

    }

    return (exit);
}

/***********************************************************
* Name: int scaraIkPulses
* function: provides the calculations to control the robot with inverse kinematics using pulses
* arguments
*            ang1:  (pointer type signed int - points to theta1 in pulses
*            ang2:  (pointer type signed int - points to theta2 in pulses
*            toolX: (double - x position of arm
*            toolY: (double - y position of arm
*
*
* created by: Matthew Wonneberg, Jamie Boyd
* Date: April 25 2022
************************************************************/
unsigned int scaraIkPulses(signed int *ang1, signed int *ang2, double toolX, double toolY, SCARA_ROBOT *scaraState1){

    volatile unsigned int exit = 0;
    volatile signed int angJ1;
    volatile signed int angJ2;
    volatile double B;     // length from origin to x,y
    volatile double beta;  // cosine law angle
    volatile double alpha; // angle of x,y

    volatile double xSqr = pow(toolX, 2);
    volatile double ySqr = pow(toolY, 2);

    volatile double holdSqr;

    holdSqr = xSqr + ySqr;

    B = sqrt(holdSqr);   // straight line distance from origin to (x,y) point
    alpha = RadToPul(atan2(toolY, toolX));                                        // angle of B from origin to (x,y) point
    beta = RadToPul(acos((pow(L2, 2) - pow(B, 2) - pow(L1, 2)) / (-2 * B * L1))); // cosine law to find beta

    if (scaraState1->scaraPos.armSol == LEFT_ARM_SOLUTION) {                      // left hand solution
        angJ1 = alpha + beta;
        if (angJ1 > MAX_ABS_THETA1_PUL || angJ1 < -MAX_ABS_THETA1_PUL){           // switch solutions if the selected solution was impossible
            angJ1 = alpha - beta;
            scaraState1->scaraPos.armSol = RIGHT_ARM_SOLUTION;                    // changed to Right hand solution
            armSolChange = 1;
            if (angJ1 > MAX_ABS_THETA1_PUL || angJ1 < -MAX_ABS_THETA1_PUL)
                exit =1;
        }
    }
    else if (scaraState1->scaraPos.armSol == RIGHT_ARM_SOLUTION) { // right hand solution
        angJ1 = alpha - beta;
        if (angJ1 < -MAX_ABS_THETA1_PUL || angJ1 > MAX_ABS_THETA1_PUL){  // switch solutions if the selected solution was not possible
            angJ1 = alpha + beta;
            scaraState1->scaraPos.armSol = LEFT_ARM_SOLUTION; // changed to left hand solution
            armSolChange = 1;
            if (angJ1 < -MAX_ABS_THETA1_PUL || angJ1 > MAX_ABS_THETA1_PUL)
                exit =1;
        }
    }

    angJ2 = RadToPul(atan2(toolY - (L1 * sin(PulToRad(angJ1))), toolX - (L1 * cos(PulToRad(angJ1)))));  // calculate joint2 angle
    if ((angJ2 < -MAX_ABS_THETA2_PUL || angJ2 > MAX_ABS_THETA2_PUL))
        exit = 1; // error if joint 2 angle is impossible to reach

    if (exit == 0) { // if the solution is possible then update structure values
        *ang1 = angJ1;
        *ang2 = angJ2;
    }

    return (exit);
}
/***********************************************************
* Name: LINE_DATA initLine
* function: used to set up the points of the line between the start and end points
* arguments
*            double xA:  x value for the robots start position
*            double yA:  y value for the robots start position
*            double xB:  x value for where the user wants the robot to end
*            double yB:  y value for where the user wants the robot to end
*            int numPts:  the number of points in the line that is used
*
* returns: LINE_DATA lineInit
* created by: Matthew Wonneberg, Jamie Boyd
* Date: March 17 2022
************************************************************/
LINE_DATA initLine(double xB, double yB, double xA, double yA, int numPts)
{
    LINE_DATA lineInit;

      lineInit.pA.x = xA; // pointsA are start points of the line
      lineInit.pA.y = yA;

      lineInit.pB.x = xB; // pointsB are end points of the line
      lineInit.pB.y = yB;
      lineInit.numPts = numPts; // num pts is the number of points that the robot moves by

    return (lineInit);
}


/***********************************************************
* Name: SCARA_ROBOT scaraInitState
* function: used to set up scaraState in with user entered values
* arguments
*            double x:  x value for where the user wants the robot to go
*            double y:  y value for where the user wants the robot to go
*            int armSol: either leftarm solution or rightarm solution 1 or 0
*            char mtrSpeed: either H, M, L
*
* returns SCARA_ROBOT Init
* created by: Matthew Wonneberg, Jamie Boyd
* Date: March 17
************************************************************/
SCARA_ROBOT scaraInitState(double x, double y, int armSol,char penState)
{
    SCARA_ROBOT Init;


    Init.scaraPos.x = x; // whatever the user enters for x and y position
    Init.scaraPos.y = y;
    Init.scaraPos.theta1 = 0;  // theta 1  and 2 are set to 0 because they are yet to be calculated
    Init.scaraPos.theta2 = 0;
    Init.scaraPos.armSol = armSol;     // armSol and penState and mtrSpeed are defined
    Init.scaraTool.penPos = penState;

    return Init;
}
//---------------------------------------------------------------------------------------
// Returns angle in radians from input angle in degrees
double DegToRad(double angDeg){
   return (PI/180.0)*angDeg;
}

//---------------------------------------------------------------------------------------
// Returns angle in radians from input angle in degrees
double RadToDeg(double angRad){
   return (180.0/PI)*angRad;
}
//---------------------------------------------------------------------------------------
// Returns angle in radians from input angle in motor pulses
double PulToRad(double pulses){
    return (PI/1708.0)*pulses; // 1707.96 deg/ rad
}
//---------------------------------------------------------------------------------------
// Returns angle in pulses from input angle in radians
double RadToPul(double radians){
    return (1708.0/PI)*radians;
}
