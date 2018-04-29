#include <stdio.h>
#include <stdbool.h>
#include "crc16.h"

#define DEBUG

#define STARTBYTE       0xAA
#define STOPBYTE        0x55
#define ESCBYTE         0x7D
#define lowByte(MSG)    ((uint8_t)((MSG) & 0xFF))
#define highByte(MSG)   ((uint8_t)((MSG) >> 8))
#define combineByte(MSB,LSB) ((uint16_t) (((MSB) << 8) | (LSB)))

#define MAXMSG          256

void convertMsg(uint8_t dest, uint8_t cmd, uint8_t *msg, uint8_t *lenmsg);
void parsePayload(uint8_t *msg, uint8_t *length);
bool checkCRC(uint8_t *msg, uint8_t length);