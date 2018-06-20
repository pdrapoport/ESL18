/*------------------------------------------------------------------
 *  msgprocess.c -- Wrapper for communication protocol
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

#define PC
#include "msgprocess.h"
#include <stdlib.h>
#include <string.h>

// Author: Vincent Bejach
// Function to initialize values for protocol related variables
void initProtocol() {
    msgId = 0;
    buffCount = 0;
    readIndex = 0;

    recBuff = 0;
    packState = wait;
}

// Author: Haris Suwignyo
// Function to make a payload based on the command ID and the message
uint8_t *makePayload(uint8_t idCmd, uint8_t *msg) {
    int msglen = cmd2len(idCmd);
    uint8_t *payload = malloc(msglen);
    uint8_t index = 0;
    int i = 0;

    // Start byte = 0xAA (1 byte)
    payload[index++] = STARTBYTE;

    /* ID + Command byte (1 byte)
     * Command: types of command
     *          e.g. change mode, joystick input, setpoint input
     */
    payload[index++] = idCmd;

    // ACTUAL MESSAGE
    // Copy input message

    for(i = 0; i < msglen-ADDBYTES; i++) {
        payload[i + index] = msg[i];
    }
    index = index + msglen-ADDBYTES;

    //compute crc
    uint16_t crc;
    //uint8_t *payload_p = payload;
    //crc16_compute function taken from components/libraries/crc16/ from the project directory
    crc = crc16_compute(payload + 1, index - 1, NULL); //compute crc without the start/stop byte
    payload[index++] = highByte(crc);
    payload[index++] = lowByte(crc);
    return payload;
}

// Author: Haris Suwignyo
// Function to convert command ID to message length
uint8_t cmd2len(uint8_t idCmd) {
    uint8_t msglen = 0;
    switch(idCmd) {
    case PWMODE:
        msglen = PWMODELEN;
        break;
    case PWMOV:
        msglen = PWMOVLEN;
        break;
    case PWKB:
        msglen = PWKBLEN;
        break;
    case DWTEL:
        msglen = DWTELLEN;
        break;
    case DWERR:
        msglen = DWERRLEN;
        break;
    default:
        msglen = 0;
        break;
    }
    return msglen;
}

// Author: Haris Suwignyo
// Function to check the CRC of a message, returns true if the calculation is true and vice versa.
bool checkCRC(uint8_t *msg, uint8_t length) {
    uint16_t calculatedCRC;
    uint16_t packetCRC = combineByte(msg[length - 2], msg[length - 1]);
    calculatedCRC = crc16_compute(msg + 1, length - 3, NULL); // msg + 1 (not counting start byte in the calculation), length - 3 (removing 2 bytes crc and 1 byte start byte)

    if(calculatedCRC == packetCRC) return true;
    else return false;
}

// Author: Vincent Bejach
/* Remove the first i-1 bytes from the recChar array, and move the remaining bytes at the start of the array (so recChar starts at index i)
 * Ex: recChar = [1 2 3 4 5] -> slide(2) -> recChar = [3 4 5]
 */
void slideMsg(uint8_t i) {
    uint16_t count = 0;
    uint8_t tmp[MAXMSG];

    for(count = i; count<MAXMSG; count++) { // Discard the i-1 first characters of recChar
        tmp[count-i] = recChar[count];
    }
    // Store the remaining characters back into recChar
    memcpy(recChar, tmp, MAXMSG);

    readIndex = 0;
    buffCount = buffCount - i;
}

// Author: Vincent Bejach
/* Remove the first i-1 bytes from the receivedMsg array, and move the remaining bytes at the start of the array (so receivedMsg starts at index i)
 * Ex: recChar = [1 2 3 4 5] -> slide(2) -> recChar = [3 4 5]
 */
void slideRecMsg(uint8_t i) {
    uint16_t count = 0;
    message_t tmp[MAXMSG];

    for(count = i; count<MAXMSG; count++) { // Discard the i-1 first characters of recChar
        tmp[count-i] = receivedMsg[count];
    }
    // Store the remaining characters back into recChar
    memcpy(recChar, tmp, MAXMSG);

    recBuff = recBuff - i;
}

// Author: Vincent Bejach
// Get the payload from the processed packet and store it in the global variable receivedMsg for further processing
message_t getPayload(uint8_t msglen) {
    uint8_t i, msgend = msglen - 2;
    message_t tmpMsg;

    tmpMsg.idCmd = recChar[1];

    for (i = 2; i<msgend; i++) { // Skip the first 2 bytes (start byte and packet header) and reads until the start of the CRC
        tmpMsg.msg[i-2] = recChar[i];
    }

    return tmpMsg;
}
