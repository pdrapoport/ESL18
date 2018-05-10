#include <stdio.h>
#include <stdbool.h>
#include "crc16.h"
#include "in4073.h"

#define DEBUG

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

//Length per CMD (in bytes)
#define PWMODELEN  5   
#define PWMOVLEN   12
#define DWLOGLEN   38
#define DWMODELEN  5
#define PRMODELEN  4

#define ADDBYTES 4

#define MAXMSG          256

#define MAXPLDSIZE 50 // Maximum payload size (50 is actually too big, but it gies room for future protocol extensions)

typedef struct payload{
    uint8_t stByte;       //start byte
    uint8_t idCmd;        //id and command
    uint8_t msgNum;        //message ID
    uint8_t msg[MAXMSG];  //actual message
    uint16_t crc16;
} payload_t;

typedef struct payload *payload_p;

uint8_t msgId = 0;
uint8_t buffCount = 0;
uint8_t recChar[MAXMSG];
uint8_t readIndex = 0;
payload_t convertMsg(uint8_t idCmd, uint8_t *msg);
uint8_t *parsePayload(payload_t packet);
void receivePkt();
uint8_t cmd2len(uint8_t idCmd);
bool checkCRC(uint8_t *msg, uint8_t length);

bool messageComplete = false;
uint8_t receivedMsg[MAXPLDSIZE];
