#ifndef MSG2PAYLOAD_H__
#define MSG2PAYLOAD_H__

#include <stdio.h>
#include <stdbool.h>
#include "crc16.h"
#include "in4073.h"

#define STARTBYTE       0xAA
#define STOPBYTE        0x55
#define ESCBYTE         0x7D
#define lowByte(MSG)    ((uint8_t)((MSG) & 0xFF))
#define highByte(MSG)   ((uint8_t)((MSG) >> 8))
#define combineByte(MSB,LSB) ((uint16_t) (((MSB) << 8) | (LSB)))

//ID + CMD
#define PWMODE 0x11    //PC2Drone Write Mode
#define PWMOV  0x12    //PC2Drone Write Movement
#define DWLOG  0x28    //Drone2PC Write Logging
#define DWMODE 0x29    //Drone2PC Write Mode
#define PRMODE 0x14    //PC2Drone Read Mode
#define PWKB   0x13    //PC2Drone Keyboard Input

//Length per CMD (in bytes)
#define PWMODELEN  6
#define PWMOVLEN   13
#define DWLOGLEN   39
#define DWMODELEN  6
#define PRMODELEN  5
#define PWKBLEN    6

#define ADDBYTES 5

#define MAXMSG          50

#define MAXPLDSIZE 10 // Maximum payload size (50 is actually too big, but it gies room for future protocol extensions)

#define MINBUFFCOUNT 6 //Minimum buffCount to start processing message

typedef struct message{
    uint8_t idCmd;
    uint8_t msg[MAXMSG];
} message_t;

enum packegeStates {
    wait, first_byte_received, receiveMsg, processMsg, CRC_Check, transmit, panic
} packState;

typedef struct payload *payload_p;

int readAttempts;
uint8_t msgId;
uint8_t buffCount;
uint8_t recChar[MAXMSG];
uint8_t readIndex;

uint8_t recBuff;

int msglen;

bool messageComplete;
bool receiveComplete;
message_t receivedMsg[MAXPLDSIZE];
void initProtocol();
uint8_t *makePayload(uint8_t idCmd, uint8_t *msg);
void receivePkt();
uint8_t cmd2len(uint8_t idCmd);
void slideMsg(uint8_t i);
void slideRecMsg(uint8_t i);
message_t getPayload(uint8_t msglen);
void processPkt();
bool checkCRC(uint8_t *msg, uint8_t length);

#endif // MSG2PAYLOAD_H__
