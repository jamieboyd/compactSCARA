/*
 * cmdInterpreter7070.h
 *
 *  Created on: Apr. 8, 2022
 *      Author: Matthew Wonneberg, Jamie Boyd
 */

#ifndef CMDINTERPRETER7070_H_
#define CMDINTERPRETER7070_H_

//-------- cmdInterpreter -----------

#define NULLNUM 0x00
#define MAX_ARGS 6

#define BUFFLEN 60

#define CMD0 "pwmFreqSet"
#define CMD0_NARGS 1
#define CMD1 "kP"
#define CMD1_NARGS 1
#define CMD2 "brakeM1"
#define CMD2_NARGS 0
#define CMD3 "displayPosM1"
#define CMD3_NARGS 0
#define CMD4 "movePosM1"
#define CMD4_NARGS 1
#define CMD5 "resetCountM1"
#define CMD5_NARGS 0

#define CMD6 "moveM2"
#define CMD6_NARGS 2
#define CMD7 "brakeM2"
#define CMD7_NARGS 0
#define CMD8 "displayPosM2"
#define CMD8_NARGS 0
#define CMD9 "movePosM2"
#define CMD9_NARGS 1
#define CMD10 "resetCountM2"
#define CMD10_NARGS 0

#define CMD11 "moveJ"
#define CMD11_NARGS 2
#define CMD12 "moveL"
#define CMD12_NARGS 4
#define CMD13 "moveC"
#define CMD13_NARGS 5
#define CMD14 "moveJcoord"
#define CMD14_NARGS 3

#define MAX_CMDS 15

volatile unsigned char data;
volatile unsigned char x;
volatile unsigned char done;
volatile unsigned char rxBuffer[99];

typedef struct CMD {
    const char *name;   // command name
    int nArgs;          // number of input arguments for a command
    int args[MAX_ARGS]; // arguments
}CMD;

void motorCmdInit(CMD *vnhCmdList);
int parseCmd(CMD * cmdList, char * cmdLine);
int validateCmd(CMD *cmdList ,char * cmdName);
int executeCmd(CMD *cmdList, int cmdIndex);



#endif /* CMDINTERPRETER7070_H_ */
