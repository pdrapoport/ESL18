#ifndef MSG2PAYLOAD_H__
#define MSG2PAYLOAD_H__

/*------------------------------------------------------------------
 *  msg2payload.h -- Wrapper for communication protocol
 *
 * 	Group 7:
 *  - Pavel Rapoport
 * 	- Antonio Rueda
 * 	- Haris Suwignyo
 * 	- Vincent Bejach
 *
 * 	TU Delft
 *
 *  June 2018
 *
 *****------------------------------------------------------------------
 */

#include <stdio.h>
#include <stdbool.h>
#include "crc16.h"

#define STARTBYTE       0xAA
#define lowByte(MSG)    ((uint8_t)((MSG) & 0xFF))
#define highByte(MSG)   ((uint8_t)((MSG) >> 8))
#define secondByte(MSG)  ((uint8_t)(((MSG) >> 8) & 0xFF))
#define thirdByte(MSG)  ((uint8_t)(((MSG) >> 16) & 0xFF))
#define fourthByte(MSG)  ((uint8_t)(((MSG) >> 24) & 0xFF))
#define combineByte(MSB,LSB) ((int16_t) (((MSB) << 8) | (LSB)))
#define combine32Byte(MSB1, MSB2, MSB3, LSB) ((int32_t) ((MSB1 << 24) | (MSB2 << 16) | (MSB3 << 8) | LSB))

//ID + CMD
#define PWMODE 0x11    //PC2Drone Write Mode
#define PWMOV  0x12    //PC2Drone Write Movement
#define DWTEL  0x27    //Drone2PC Telemetry Packet
#define PWKB   0x13    //PC2Drone Keyboard Input
#define DWERR  0x26    //Drone2PC Write Error Code

//Length per CMD (in bytes)
#define PWMODELEN  5
#define PWMOVLEN   12
#define DWTELLEN   47
#define PWKBLEN    5
#define DWERRLEN   5

#define ADDBYTES   4  //Additional bytes besides the original message
#define MAXMSG     50
#define MAXPLDSIZE 10 // Maximum payload size (50 is actually too big, but it gies room for future protocol extensions)
#define MINBUFFCOUNT 6 //Minimum buffCount to start processing message

typedef struct message{
    uint8_t idCmd;
    uint8_t msg[MAXMSG];
} message_t;

enum packegeStates {
    wait, first_byte_received, receiveMsg, CRC_Check, transmit, panic
} packState;

uint8_t msgId;
uint8_t buffCount;
uint8_t recChar[MAXMSG];
message_t receivedMsg[MAXPLDSIZE];
uint8_t readIndex;

uint8_t recBuff;

int msglen;

void initProtocol();
void receivePkt();
void slideMsg(uint8_t i);
void slideRecMsg(uint8_t i);
uint8_t *makePayload(uint8_t idCmd, uint8_t *msg);
uint8_t cmd2len(uint8_t idCmd);
message_t getPayload(uint8_t msglen);
bool checkCRC(uint8_t *msg, uint8_t length);

#endif // MSG2PAYLOAD_H__
