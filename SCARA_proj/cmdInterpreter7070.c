/*
 * cmdInterpreter7070.c
 *
 *  Created on: March, 8 2022
 *      Author: Matthew Wonneberg, Jamie Boyd
 */
#include <msp430.h>
#include "ucsiUart.h"
#include <string.h>
#include <UartPwmTimerA0.h>
#include <mdd_driver.h>
#include <cmdInterpreter7070.h>
#include <quadEncDec.h>
#include <movement.h>
#include <updateTimerB.h>

/************************************************
 * Function motorCmdInit
 *
 * sets up the names and number of arguments for the motor control commands
 *
 * argurments: CMD * vnhCmdList
 *
 * author Matthew Wonneberg, Jamie Boyd
 * date: March 18 2022
 * returns: nothing
 **************************************************/
void motorCmdInit(CMD *vnhCmdList){

    // initialize commands
    //M1
    vnhCmdList[0].name = CMD0;
    vnhCmdList[0].nArgs = CMD0_NARGS;

    vnhCmdList[1].name = CMD1;
    vnhCmdList[1].nArgs = CMD1_NARGS;

    vnhCmdList[2].name = CMD2;
    vnhCmdList[2].nArgs = CMD2_NARGS;

    vnhCmdList[3].name = CMD3;
    vnhCmdList[3].nArgs = CMD3_NARGS;

    vnhCmdList[4].name = CMD4;
    vnhCmdList[4].nArgs = CMD4_NARGS;

    vnhCmdList[5].name = CMD5;
    vnhCmdList[5].nArgs = CMD5_NARGS;

    //M2
    vnhCmdList[6].name = CMD6;
    vnhCmdList[6].nArgs = CMD6_NARGS;

    vnhCmdList[7].name = CMD7;
    vnhCmdList[7].nArgs = CMD7_NARGS;

    vnhCmdList[8].name = CMD8;
    vnhCmdList[8].nArgs = CMD8_NARGS;

    vnhCmdList[9].name = CMD9;
    vnhCmdList[9].nArgs = CMD9_NARGS;

    vnhCmdList[10].name = CMD10;
    vnhCmdList[10].nArgs = CMD10_NARGS;

    vnhCmdList[11].name = CMD11;
    vnhCmdList[11].nArgs = CMD11_NARGS;

    vnhCmdList[12].name = CMD12;
    vnhCmdList[12].nArgs = CMD12_NARGS;

    vnhCmdList[13].name = CMD13;
    vnhCmdList[13].nArgs = CMD13_NARGS;

    vnhCmdList[14].name = CMD14;
    vnhCmdList[14].nArgs = CMD14_NARGS;



}
/***********************************
 * Function parseCmd
 *
 * validate command name by calling validateCmd
 * gets the correct cmdIndex from validateCmd
 *
 * use the predetermined nArgs to parse the args
 *
 * argurments: CMD * cmdList, char * cmdLine
 *
 * author Matthew Wonneberg, Jamie Boyd
 * Date: March 8 2022
 * returns: pointer to rxString or Null if unsucessful
 **********************************/
int parseCmd(CMD * cmdList, char * cmdLine){

    volatile signed int value =0; // return data
    int index =0; // command number

    char *token;

    volatile unsigned int nArgs =0;
    volatile unsigned int size =0;
    volatile unsigned int n =0; // general counter
    volatile unsigned int m =0; // general counter

    volatile char invalidString[] = "INVALID COMMAND!\n\r";
    volatile char numChars;


    token = strtok(cmdLine, " ,.\t\n");   // token recieves the command until a delimiter is reached

    index = validateCmd(cmdList, token);  // the command is verified and the index of the command is assigned

    if (index >= 0 && index <= 5){
        while (token != NULL){            // while token is not the last character in the buffer
        token = strtok(NULL, " ,.\t\n");  // the argument is assigned to token following the command in line

        //-----------M1 cmds--------------------------
            if (index == 5){ // reset count, no args
                 token = NULL;
                 nArgs =0;
            }
            else if (index == 4){ // moveJ
                if (n == 0){// first arg
                    cmdList[index].args[n] = atoi(token); // convert angle ascii to integer
                    nArgs++;
                    token = NULL;
                }
            }
            else if (index == 3){ // display position
                token = NULL;
                nArgs =0;
            }
            else if (index == 2){ // Brake command no arguments
                token = NULL;                 // quits the loop with zero arguments
                cmdList[index].args[n] = 0;   // duty cycle for brake
                nArgs =0;                     // zero arguments are assigned
            }
            else if (index == 1){ // move (cw/ccw) (0:90)
                token = NULL;
                cmdList[index].args[n] = atoi(token);
                nArgs = 1; // inc argument count
            }

            else if (index == 0){ // change frequency command
                cmdList[index].args[n] = atoi(token); // frequency for the command is converted from ascii to integer
                nArgs++;          // inc argument count
                token = NULL;     // this is the last argument for the command

            }

        }
        if (nArgs > cmdList[index].nArgs || nArgs < cmdList[index].nArgs){ // too many arguments in command
             value = -1;  // parseCmd exits with invalid data
        }
        if (value == 0){
            value = index;
        }

    }
    //----------------M2 cmds ------------------------
    else if (index >= 6 && index <= 10){
        while (token != NULL){            // while token is not the last character in the buffer

            token = strtok(NULL, " ,.\t\n");  // the argument is assigned to token following the command in line

            if (index == 10){ // reset count, no args
                 token = NULL;
                 nArgs =0;
            }
            else if (index == 9){ // moveJ
                if (n == 0){// first arg
                    cmdList[index].args[n] = atoi(token); // convert angle ascii to integer
                    nArgs++;
                    token = NULL;
                }
            }
            else if (index == 8){ // display position
                token = NULL;
                nArgs =0;
            }
            else if (index == 7){ // Brake command no arguments
                token = NULL;                 // quits the loop with zero arguments
                cmdList[index].args[n] = 0;   // duty cycle for brake
                nArgs =0;                     // zero arguments are assigned
            }
            else if (index == 6){ // Move (cw/ccw) (1:9)

                if (n == 0){ // first argument
                    if (strcmp("cw", token) == 0 || strcmp("CW", token) == 0){
                        cmdList[index].args[n] = 1; // direction CW
                        nArgs++; // inc argument count
                    }
                    else if (strcmp("ccw", token) == 0 || strcmp("CCW", token) == 0){
                        cmdList[index].args[n] = 0; // direction CCW
                        nArgs++; // inc argument count
                    }
                    else{
                        value =-1; // command is invalid
                    }
                }
                else if (n == 1){  // second argument
                    cmdList[index].args[n] = atoi(token); // dutyCycle for the command is converted from ascii to integer
                    nArgs++;       // inc argument count
                    token = NULL;  // this is the last argument for the command
                }
                n++;
            }

        }
        if (nArgs > cmdList[index].nArgs || nArgs < cmdList[index].nArgs){ // too many arguments in command
             value = -1;  // parseCmd exits with invalid data
        }
        if (value == 0){
            value = index;
        }

    }//------------------- moveJ cmd11 ---------------------
    else if (index == 11){
        while (token != NULL){            // while token is not the last character in the buffer

            token = strtok(NULL, " ,.\t\n");  // the argument is assigned to token following the command in line

        if (n == 0){  // first argument
            cmdList[index].args[n] = atoi(token); // dutyCycle for the command is converted from ascii to integer
            nArgs++;       // inc argument count
        }
        else if (n == 1){  // second argument
            cmdList[index].args[n] = atoi(token); // dutyCycle for the command is converted from ascii to integer
            nArgs++;       // inc argument count
            token = NULL;
        }
        n++;
        }
        if (nArgs > cmdList[index].nArgs || nArgs < cmdList[index].nArgs){ // too many arguments in command
             value = -1;  // parseCmd exits with invalid data
        }
        if (value == 0){
            value = index;
        }
    }
    else if (index == 12){//----------------moveL---------------------
        while (token != NULL){            // while token is not the last character in the buffer

            token = strtok(NULL, " ,.\t\n");  // the argument is assigned to token following the command in line

            if (n == 0){  // first argument
                    cmdList[index].args[n] = atoi(token); // dutyCycle for the command is converted from ascii to integer
                    nArgs++;       // inc argument count
            }
            else if (n == 1){  // second argument
                cmdList[index].args[n] = atoi(token); // dutyCycle for the command is converted from ascii to integer
                nArgs++;       // inc argument count
            }
            else if (n == 2){  // third argument
                cmdList[index].args[n] = atoi(token); // dutyCycle for the command is converted from ascii to integer
                nArgs++;       // inc argument count
            }
            else if (n == 3){  // fourth argument
                cmdList[index].args[n] = atoi(token); // dutyCycle for the command is converted from ascii to integer
                nArgs++;       // inc argument count

            }
            n++;
        }
        if (nArgs > cmdList[index].nArgs || nArgs < cmdList[index].nArgs){ // too many arguments in command
             value = -1;  // parseCmd exits with invalid data
        }
        if (value == 0){
            value = index;
        }
    }
    else if (index == 13){ //-----------moveC-------------------
        while (token != NULL){            // while token is not the last character in the buffer

            token = strtok(NULL, " ,.\t\n");  // the argument is assigned to token following the command in line

            if (n == 0){  // first argument
                    cmdList[index].args[n] = atoi(token); // dutyCycle for the command is converted from ascii to integer
                    nArgs++;       // inc argument count
            }
            else if (n == 1){  // second argument
                cmdList[index].args[n] = atoi(token); // dutyCycle for the command is converted from ascii to integer
                nArgs++;       // inc argument count
            }
            else if (n == 2){  // third argument
                cmdList[index].args[n] = atoi(token); // dutyCycle for the command is converted from ascii to integer
                nArgs++;       // inc argument count
            }
            else if (n == 3){  // third argument
                cmdList[index].args[n] = atoi(token); // dutyCycle for the command is converted from ascii to integer
                nArgs++;       // inc argument count
            }
            else if (n == 4){  // third argument
                cmdList[index].args[n] = atoi(token); // dutyCycle for the command is converted from ascii to integer
                nArgs++;       // inc argument count
            }
            n++;
        }
        if (nArgs > cmdList[index].nArgs || nArgs < cmdList[index].nArgs){ // too many arguments in command
             value = -1;  // parseCmd exits with invalid data
        }
        if (value == 0){
            value = index;
        }
        else
            value = -1;
    }
    else if (index == 14){//---------------moveJcoord-------------------
        while (token != NULL){            // while token is not the last character in the buffer

            token = strtok(NULL, " ,.\t\n");  // the argument is assigned to token following the command in line

        if (n == 0){  // first argument
            cmdList[index].args[n] = atoi(token); // dutyCycle for the command is converted from ascii to integer
            nArgs++;       // inc argument count
        }
        else if (n == 1){  // first argument
            cmdList[index].args[n] = atoi(token); // dutyCycle for the command is converted from ascii to integer
            nArgs++;       // inc argument count
        }
        else if (n == 2){  // second argument
            cmdList[index].args[n] = atoi(token); // dutyCycle for the command is converted from ascii to integer
            nArgs++;       // inc argument count
            token = NULL;
        }
        n++;
        }
        if (nArgs > cmdList[index].nArgs || nArgs < cmdList[index].nArgs){ // too many arguments in command
             value = -1;  // parseCmd exits with invalid data
        }
        if (value == 0){
            value = index;
        }
    }

    if (index < 0 || index > 14)
        value = -1;

    return value;
}

/***********************************
 * Function validateCmd
 *
 * determines the correct index number for a entered command
 *
 * argurments: CMD * cmdList, char * cmdName
 * author G.Scutt
 * created April 2020
 * returns: idx
 **********************************/
int validateCmd(CMD *cmdList ,char * cmdName){

   int i =0;
   int idx =-1;
   int invalidCmd =1;

   while (invalidCmd && i < MAX_CMDS){
       invalidCmd = strcmp(cmdName, cmdList[i++].name);
   }
   if (!invalidCmd)
       idx = i - 1;

   return idx;
}

/***********************************
 * Function executeCMD
 *
 * executes the correct motor function based on the command entered into the console
 *
 * argurments: CMD *cmdList, int cmdIndex
 * author: Matthew Wonneberg, Jamie Boyd
 * date March 18 2022
 * returns: result
 **********************************/
int executeCmd(CMD *cmdList, int cmdIndex){

    volatile signed char result=0;
    volatile unsigned int pwmFreqReq;
    volatile unsigned char dutySend;
    volatile unsigned char dutyEnd;
    volatile unsigned char dutySend2;
    volatile unsigned char dutyEnd2;

    volatile float j1HoldAng =0;
    volatile float j2HoldAng =0;

    volatile unsigned char cwRet=0;
    volatile unsigned char ccwRet=0;


    switch(cmdIndex){
    case 0://---------------pwmFreqSet (freq)-----------

        pwmFreqReq = cmdList[0].args[0];      //pwmFreqReq is the integer frequency value

        if (pwmFreqReq >= PWMFREQMIN && pwmFreqReq <= PWMFREQMAX){
            result = timerA0DutyCycleSet(0); // set PWM to zero percent
            result = mddInputCtrl(CTRLBRAKE); // send brake signal
            result = timerA0DutyCycleSet2(0); // set PWM to zero percent
            result = mddInputCtrl2(CTRLBRAKE2); // send brake signal

           if (result != -1){ // if brake was successful
               result = timerA0PwmFreqSet(pwmFreqReq); // send the new pwmFrequency to the both signal timers
           }
        }
        else{
          result = -1;
        }

        break;
    case 1://---------------kP--------
        kP = cmdList[0].args[0]/100;
        break;
    case 2://----------------brakeM1()-------------
        result = timerA0DutyCycleSet(0); // set PWM to zero percent
        result = mddInputCtrl(CTRLBRAKE); // send brake signal
        break;
    case 3://--------------displayPos()---------------
        displayPos();
        break;
    case 4://--------------moveJ---------------------

        break;
    case 5://--------------resetCount--------------
      //  startM1 = 0;
        posCount =0;
        displayPos();
        break;
    case 6://-----------------------

        break;
    case 7://----------------BrakeM2-------------
        result = timerA0DutyCycleSet2(0); // set PWM to zero percent
        result = mddInputCtrl2(CTRLBRAKE2); // send brake signal
        break;
    case 8://--------------displayPos()---------------
        displayPos2();
        break;
    case 9://--------------moveJ---------------------
        break;
    case 10://--------------resetCount--------------
        posCount2 =0;
        displayPos2();
        break;
    case 11://-------------moveJ--------------------

       // convert degrees into pulses
       scaraStateEnd.scaraPos.theta1 = cmdList[11].args[0]*PUL_PER_DEG_N70;
       scaraStateEnd.scaraPos.theta2 = cmdList[11].args[1]*PUL_PER_DEG_N70;

       sendMoveJ(scaraStateEnd);
       break;
    case 12://-----------moveL-------------------

        holdLine = initLine(cmdList[12].args[0], cmdList[12].args[1], 0, 0, 0);//xb yb xa ya npts
        SCARA_ROBOT testRobot = scaraInitState(0, 0, cmdList[12].args[2], cmdList[12].args[3]); // x y armSol penPos

        sendMoveL(&testRobot, holdLine);
        break;
    case 13://----------moveC--------------------

        if (abs(cmdList[13].args[0]) > 360 || abs(cmdList[13].args[1]) > 360) // verify that both angles do not exceed 360 degrees
            result = -1;
      //  if (cmdList[13].args[2] < 1 || cmdList[13].args[2] > MAX_ABS_X) // verify that the radius is within the set limits
      //      result = -1;
        if (abs(cmdList[13].args[1] - cmdList[13].args[0]) > 361) // verify that the arc does not go over 361 degrees
            result = -1;
        if (result == 0){ // store the angles of the arc and the arm solution
            scaraStateSet.scaraPos.theta1 = cmdList[13].args[0]*PUL_PER_DEG_N70; // starting angle
            scaraStateEnd.scaraPos.theta1 = cmdList[13].args[1]*PUL_PER_DEG_N70; // ending angle
            scaraStateSet.scaraPos.radius = cmdList[13].args[2];
            SCARA_ROBOT robot = scaraInitState(0, 0, cmdList[13].args[3], cmdList[13].args[4]); // x y armSol penPos

            // send the move command
            sendMoveC(&robot);
        }
        break;
    case 14://---------------- moveJcoord-----------------

        // store the desired arm solution (1 for left arm, 0 for right arm
        scaraStateEnd.scaraPos.armSol = cmdList[14].args[2];

        // find the coorisponding arm angles based on the desired (x, y) coordinate
        result = scaraIkFloat(&j1HoldAng, &j2HoldAng, cmdList[14].args[0], cmdList[14].args[1], &scaraStateEnd);

        // store the joint angles in pulses
        scaraStateEnd.scaraPos.theta1 = j1HoldAng*PUL_PER_DEG_N70;
        scaraStateEnd.scaraPos.theta2 = j2HoldAng*PUL_PER_DEG_N70;

        // send move command
        sendMoveJ(scaraStateEnd);

    }//---------------------------------

 return result;
}
