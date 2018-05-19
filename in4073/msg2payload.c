/* msg2payload.c
 * Haris Suwignyo
 *
 * initProtocol(): Initialize variable values
 *
 * cmd2len():
 * - Convert command input to the length of the message
 *
 * makePayload():
 * - Create a payload with command and message input
 *
 * receivePkt():
 * - Retrieve packet from the rx buffer
 */
#define DRONE
#include "msg2payload.h"
#include <stdlib.h>
#include <string.h>
void initProtocol(){
    msgId = 0;
    buffCount = 0;
    readIndex = 0;

    recBuff = 0;
    messageComplete = false;
    receiveComplete = false;
    packState = wait;
}

uint8_t *makePayload(uint8_t idCmd, uint8_t *msg){
    int msglen = cmd2len(idCmd);
    uint8_t *payload = malloc(msglen);
    uint8_t index = 0;
    int i = 0;

    // Start byte = 0xAA (1 byte)
    payload[index++] = STARTBYTE;

    /* ID + Command byte (1 byte)
     * ID: 4 MSB -> PC2DRONE or DRONE2PC
     * Command: 4 LSB -> types of command
     *                    e.g. change mode, joystick input, setpoint input
     */
    payload[index++] = idCmd;

    //msgNum TO BE IMPLEMENTED, just set it to 0x00 for now
    payload[index++] = 0x00;

    // ACTUAL MESSAGE
    // Copy input message

    for(i = 0; i < msglen-ADDBYTES; i++){
        payload[i + index] = msg[i];
    }
    index = index + msglen-ADDBYTES;

    //compute crc
    uint16_t crc;
    //uint8_t *payload_p = payload;
    crc = crc16_compute(payload + 1, index - 1, NULL); //compute crc without the start/stop byte
    payload[index++] = highByte(crc);
    payload[index++] = lowByte(crc);
    return payload;
}

#ifdef DRONE
void receivePkt(){
    //read data here
    while (rx_queue.count) {
        recChar[buffCount++] = (uint8_t)dequeue(&rx_queue);
        //printf("%02X ", recChar[buffCount - 1]);
    }
}
#endif


uint8_t cmd2len(uint8_t idCmd){
    uint8_t msglen = 0;
    switch(idCmd){
        case PWMODE:
            msglen = PWMODELEN;
            break;
        case PWMOV:
            msglen = PWMOVLEN;
            break;
        case DWLOG:
            msglen = DWLOGLEN;
            break;
        case DWMODE:
            msglen = DWMODELEN;
            break;
        case PRMODE:
            msglen = PRMODELEN;
            break;
        case PWKB:
            msglen = PWKBLEN;
            break;
        default:
            msglen = 0;
            break;
    }
    return msglen;
}

bool checkCRC(uint8_t *msg, uint8_t length){
    uint16_t packetCRC = combineByte(msg[length - 2], msg[length - 1]);
    uint16_t calculatedCRC = crc16_compute(msg + 1, length - 3, NULL); // msg + 1 (not counting start byte in the calculation), length - 3 (removing 2 bytes crc and 1 byte start byte)

    return (calculatedCRC == packetCRC);
}

/*
void drone2pc(uint8_t *msg){
    int i = 0;
    int msglen = cmd2len(msg[1]);
    //for(i = 0; i<msglen;printf("%04x ",msg[i]),i++);
}*/

// Author: Vincent Bejach
/* Remove the first i-1 bytes from the recChar array, and move the remaining bytes at the start of the array (so recChar starts at index i)
 * Ex: recChar = [1 2 3 4 5 '\0'] -> slide(2) -> recChar = [3 4 5 '\0' '\0'? '\0'?]
 * The characters marked ? are irrelevant because they are situated after the first '\0' and thus will never be processed. They will also be overwritten by the next characters being added to recChar
 */
void slideMsg(uint8_t i) {
    uint16_t count = 0;
    uint8_t tmp[MAXMSG];

    // Disable interrupts

    for(count = i; count<MAXMSG; count++) { // Discard the i-1 first characters of recChar
        tmp[count-i] = recChar[count];
    }
    // Store the remaining characters back into recChar
    memcpy(recChar, tmp, MAXMSG); //may cause buffer overflow according to dudes in stackoverflow

    readIndex = 0;
    buffCount = buffCount - i;

    // Enable interrupts back

    // Note: I think doing it like this avoids the need to allocate a temporary buffer for the packet to process: this should prevent a race condition between interrupt and the sliding function. It is the only place where we need to write to recChar, so there shouldn't be any problem for all the other interactions with this array (reads in the processPkt function)
}

void slideRecMsg(uint8_t i) {
    uint16_t count = 0;
    message_t tmp[MAXMSG];

    // Disable interrupts

    for(count = i; count<MAXMSG; count++) { // Discard the i-1 first characters of recChar
        tmp[count-i] = receivedMsg[count];
    }
    // Store the remaining characters back into recChar
    memcpy(recChar, tmp, MAXMSG); //may cause buffer overflow according to dudes in stackoverflow

    recBuff = recBuff - i;
    // Enable interrupts back

    // Note: I think doing it like this avoids the need to allocate a temporary buffer for the packet to process: this should prevent a race condition between interrupt and the sliding function. It is the only place where we need to write to recChar, so there shouldn't be any problem for all the other interactions with this array (reads in the processPkt function)
}



// Author: Vincent Bejach
// Get the payload from the processed packet and store it in the global variable receivedMsg for further processing
message_t getPayload(uint8_t msglen) {
    uint8_t i, msgend = msglen - 2;
    message_t tmpMsg;

    tmpMsg.idCmd = recChar[1];

    for (i = 3; i<msgend; i++) { // Skip the first 2 bytes (start byte and packet header) and reads until the start of the CRC
        tmpMsg.msg[i-3] = recChar[i];
    }

    return tmpMsg;
}


// Author: Vincent Bejach
/* Implement the FSM defined for the communication protocol.
 * Reads from the global variable recChar, and remove part of its content when a packet is done being processed or when some bytes are thrown away.
 * Outputs the message of the packet being processed in the global receivedMsg array. The fnished processing is indicated by the flag messageComplete being set to true.
 */
bool processPkt() {
    //uint16_t timeout; // TO BE IMPLEMENTED
    bool crc_result = false;
    //bool panic_on = false; // Used to exit the loop in case of an emergency transition to panic mode

        receivePkt();
        while (readIndex < buffCount) {
            switch (packState) {
                case wait:
                    if (recChar[readIndex] == STARTBYTE) {
                        ++readIndex;
                        packState = first_byte_received;
                    }
                    else {
                        slideMsg(1);
                    }
                    //printf("WAIT!\n");
                    break;
                case first_byte_received:
                    msglen = cmd2len(recChar[readIndex++]);
                    packState = receiveMsg;
                    if (msglen == 0){
                        slideMsg(1);
                        packState = wait;
                    }
                    //printf("FIRST!\n");
                    break;
                case receiveMsg:
                    if (readIndex < msglen - 1) {
                        ++readIndex;
                    }
                    else {
                        packState = CRC_Check;
                    }
                    //printf("RECV\n");
                    break;
                case CRC_Check:
                    crc_result = checkCRC(recChar, msglen);
                    if(crc_result)
                        packState = processMsg;
                    else {
                        printf("CRC FAIL!\n");
                        packState = wait;
                        slideMsg(1);
                    }
                    //printf("CRC!\n");
                    break;
                case processMsg:
                    receivedMsg[++recBuff] = getPayload(msglen);
                    //printf("RECEIVED MESSAGE: ");
                    for (int k = 0; k < msglen; ++k) {
                        //printf("%02X ", recChar[k]);
                    }
                    //printf("\n");
                    slideMsg(msglen);
                    packState = wait;
                    //printf("PROCESS!\n");
                    return true;
                case panic:
                    //TODO: Fall on the floor and cry "AAAAAAAAAAAAAAAAAAAAAAAAAAA!!!"
                    //panic_on = true;
                    break;
                default:
                    packState = panic;
            }
        }
    return false;
}
