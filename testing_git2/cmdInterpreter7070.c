/*
 * cmdInterpreter7070.c
 *
 *  Created on: Feb. 20, 2021
 *      Author: Rinz
 */
#include <msp430.h>
#include "ucsiUart.h"
#include <string.h>
#include <UartPwmTimerA0.h>
#include <mdd_driver.h>
#include <quadEncDec.h>

/************************************************
 * Function motorCmdInit
 *
 * sets up the names and number of arguments for the motor control commands
 *
 * argurments: CMD * vnhCmdList
 *
 * author mattw
 * created: Feb 18 2021
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
 * author mattw
 * created april 2020
 * modified Feb 18 2021
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
            value = executeCmd(cmdList, index); // send index and command to execute
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
            value = executeCmd(cmdList, index); // send index and command to execute
        }

    }
    else
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
 * author mattw
 * created Feb 18 2021
 * returns: result
 **********************************/
int executeCmd(CMD *cmdList, int cmdIndex){

    volatile signed char result=0;
    volatile unsigned int pwmFreqReq;
    volatile unsigned char dutySend;
    volatile unsigned char dutyEnd;
    volatile unsigned char dutySend2;
    volatile unsigned char dutyEnd2;

    volatile unsigned char cwRet=0;
    volatile unsigned char ccwRet=0;


    switch(cmdIndex){
    case 0://---------------pwmFreqSet (freq)-----------

        pwmFreqReq = cmdList[0].args[0];      //pwmFreqReq is the integer frequency value

        if (pwmFreqReq >= PWMFREQMIN && pwmFreqReq <= PWMFREQMAX){
            dutyEnd = dutyPrev;              // after braking, the dutyCycle can be returned
           result = mddBrake();          // brake motor
           if (result != -1){                // if brake was successful
               result = timerA0PwmFreqSet(pwmFreqReq); // send the new pwmFrequncy to the timer
               if (clkWise == 1){
                   result = mddCW(dutyEnd);// return to previous speed
               }
               else if (countClkWise == 1){
                   result = mddCCW(dutyEnd);// return to previous speed
               }
           }
        }
        else{
          result = -1;
        }

        break;
    case 1://---------------move (cw/ccw) (dutyCycle)--------
        enterLoop = 0;
        if (cmdList[1].args[0] == 0){             //  CCW command
            dutySend = cmdList[1].args[1];        //  dutyCycle request
            if (dutySend >= DUTYCYCLEMIN && dutySend <= DUTYCYCLEMAX){  // if dutyCycle is valid
               result = mddCCW(dutySend);     // update new dutyCycle
            }
            else{
                result =-1; // dutyCycle out of range
            }
        }
        else if (cmdList[1].args[0] == 1){       // CW command
            dutySend = cmdList[1].args[1];       // dutyCycle request
            if (dutySend >= DUTYCYCLEMIN && dutySend <= DUTYCYCLEMAX){ // validate dutyCycle
                result = mddCW(dutySend);    // update new dutycycle
            }
            else{
                result =-1; // dutyCycle out of range
            }
        }
        else{
            result = -1;
        }

        break;
    case 2://----------------brake()-------------
        enterLoop = 0;
        dutySend = 0;
        result = mddBrake(); // send brake signal
        break;
    case 3://--------------displayPos()---------------
        displayPos();
        break;
    case 4://--------------moveJ---------------------
        angJ1Desired = cmdList[4].args[0]; // update desired angle
        enterLoop = 1;
        break;
    case 5://--------------resetCount--------------
        enterLoop = 0;
        posCount =0;
        displayPos();
        break;
    case 6://---------------move (cw/ccw) (dutyCycle)--------
        enterLoop2 = 0;
        if (cmdList[6].args[0] == 0){             //  CCW command
            dutySend2 = cmdList[6].args[1];        //  dutyCycle request
            if (dutySend2 >= DUTYCYCLEMIN && dutySend2 <= DUTYCYCLEMAX){  // if dutyCycle is valid
               result = mddCCW2(dutySend2);     // update new dutyCycle
            }
            else{
                result =-1; // dutyCycle out of range
            }
        }
        else if (cmdList[6].args[0] == 1){       // CW command
            dutySend2 = cmdList[6].args[1];       // dutyCycle request
            if (dutySend2 >= DUTYCYCLEMIN && dutySend2 <= DUTYCYCLEMAX){ // validate dutyCycle
                result = mddCW2(dutySend2);    // update new dutycycle
            }
            else{
                result =-1; // dutyCycle out of range
            }
        }
        else{
            result = -1;
        }

        break;
    case 7://----------------vnhBrake()-------------
        enterLoop2 = 0;
        dutySend2 = 0;
        result = mddBrake2(); // send brake signal
        break;
    case 8://--------------displayPos()---------------
        displayPos2();
        break;
    case 9://--------------moveJ---------------------
        angJ2Desired = cmdList[9].args[0]; // update desired angle
        enterLoop2 = 1;
        break;
    case 10://--------------resetCount--------------
        enterLoop2 = 0;
        posCount2 =0;
        displayPos2();
        break;
    }//---------------------------------

 return result;
}