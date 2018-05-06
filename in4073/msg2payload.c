/* msg2payload.c
 * Haris Suwignyo
 *
 * convertMsg():
 * - Append additional bytes to original messages
 * - Calculate CRC and append it to the payload
 * - Add byte stuffing to avoid mis-interpretation of packet
 * 
 * parsePayload():
 * - Parse incoming packet, outputting the original packet
 * 
 * checkCRC():
 * - Check the integrity of the packet
 * 
 * TO DO LIST:
 * - Make the sendAck function
 * - Finish the sliding window function in processPkt()
 */

#include "msg2payload.h"
#include <stdlib.h>

payload_t convertMsg(uint8_t idCmd, uint8_t *msg){
    payload_t payload;
    uint8_t msglen = 0;
    int i = 0;
    msglen = cmd2len(idCmd);

    // Start byte = 0xAA (1 byte)
    payload.stByte = 0xAA;

    /* ID + Command byte (1 byte)
     * ID: 4 MSB -> PC2DRONE or DRONE2PC
     * Command: 4 LSB -> types of command
     *                    e.g. change mode, joystick input, setpoint input
     */
    payload.idCmd = idCmd;
    
    // Message number (1 byte)
    payload.msgNum = msgId++;

    // ACTUAL MESSAGE
    // Copy input message
    for(i = 0; i < msglen - ADDBYTES; i++){
        payload.msg[i] = msg[i];
    }
    
    //compute crc
    uint16_t crc;
    crc = crc16_compute(&payload.idCmd, msglen - sizeof(payload.crc16), NULL); //compute crc without the start/stop byte
    payload.crc16 = crc;
    
    /*
    #ifdef DEBUG
    printf("\nOriginal Payload: \n");
    for(i = 0; i < index; i++){
        printf("%04x ", payload[i]);
    }
    #endif
    
    //byte stuffing
    int j = 0;
    i = 1;
    while(i < index - 1){
        if(payload[i] == STARTBYTE || payload[i] == STOPBYTE || payload[i] == ESCBYTE){
            for(j = index - 1; j >= i; j--) payload[j+1] = payload[j];
            payload[i] = ESCBYTE;
            i++;
            index++;
        }
        i++;
    }
    */
   printf("%04x %04x %04x ",payload.stByte,payload.idCmd,payload.msgNum);
   for(i = 0; i < msglen - ADDBYTES; printf("%04x ",payload.msg[i]),i++);
   printf("%04x\n",payload.crc16);
   return payload;
}


uint8_t *parsePayload(payload_t packet){
    int i = 0;
    uint8_t *parsedMsg = malloc(MAXMSG);
    uint8_t msglen = 0;

    if(!parsedMsg) return NULL;

    msglen = cmd2len(packet.idCmd);
        
    for(i = 0; i < msglen - ADDBYTES; i++){
        parsedMsg[i] = packet.msg[i];
    }

    #ifdef DEBUG
    printf("\nParsed Message: \n");
    for(i = 0; i < msglen - ADDBYTES; i++){
        printf("%04x ", parsedMsg[i]);
    }
    #endif

    return parsedMsg;
}

void receivePkt(){
    //read data here
    while(rx_queue.count){
        recChar[buffCount++] = (uint8_t)dequeue(&rx_queue);
    }
}

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
        default:
            msglen = 0;
            break;
    }
    return msglen;
}

//TO DO
void processPkt(){
    uint8_t msgBuff[MAXMSG];
    if(recChar[readIndex++] == STARTBYTE){
        int msglen = 0;
        int i = 0;
        msglen = cmd2len(recChar[readIndex++]);
        i = readIndex;
        while(i < readIndex + msglen){
            i++;
        }
    }
}


bool checkCRC(uint8_t *msg, uint8_t length){
    uint16_t calculatedCRC;
    uint16_t packetCRC = combineByte(msg[length - 2], msg[length - 1]);
    calculatedCRC = crc16_compute(msg, length - 2, NULL);
    //printf("%04x", crc);
    if(calculatedCRC == packetCRC) return true;
    else return false;
}

int main(){
    uint8_t msg[256];
    uint8_t len = 1;
    msg[0] = 0xAA;
    msg[1] = 0xBB;
    msg[2] = 0xCC;
    msg[3] = 0xDD;
    msg[4] = 0xEE;
    msg[5] = 0xFF;
    msg[6] = 0x11;
    msg[7] = 0x22;
    //msg[8] = 0x33;
    //msg[9] = 0x44;
    //msg[10] = 0x55;
    printf("Original Message:\n");
    for(int j = 0; j < len; printf("%04x ",msg[j]), j++);
    printf("\n");
    payload_t payload;
    payload = convertMsg(PWMOV, msg);

    uint8_t *recMsg = parsePayload(payload);
    free(recMsg);
    return 0;
}