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
 */

#include "msg2payload.h"

void convertMsg(uint8_t dest, uint8_t cmd, uint8_t *msg, uint8_t *lenmsg){
    uint8_t payload[MAXMSG];
    uint8_t index = 0;
    int i = 0;

    // Start byte = 0xAA (1 byte)
    payload[index++] = STARTBYTE;

    /* ID + Command byte (1 byte)
     * ID: 4 MSB -> PC2DRONE or DRONE2PC
     * Command: 4 LSB -> types of command
     *                    e.g. change mode, joystick input, setpoint input
     */
    payload[index++] = (dest << 4) | (cmd & 0x0F);
    
    // Message length (1 byte)
    payload[index++] = sizeof(payload[1]) + 1 + *lenmsg + 2;

    // ACTUAL MESSAGE
    // Copy input message
    for(i = 0; i < *lenmsg; i++){
        payload[i + index] = msg[i];
    }
    index = index + *lenmsg;
    
    //compute crc
    uint16_t crc;
    uint8_t *payload_p = payload;
    crc = crc16_compute(payload_p + 1, index - 1, NULL); //compute crc without the start/stop byte
    payload[index++] = highByte(crc);
    payload[index++] = lowByte(crc);

    payload[index++] = STOPBYTE;

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

    //update message length and message values
    *lenmsg = index;
    for(i = 0; i < index; msg[i] = payload[i], i++);
}

void parsePayload(uint8_t *msg, uint8_t *length){
    int i = 0;
    bool startMsg = false;
    uint8_t parsedMsg[MAXMSG];
    uint8_t parsedLen = 0;
    while(i < *length){
        //handling start byte
        if(startMsg == false && msg[i] == STARTBYTE){
            printf("\nStart of Message");
            startMsg = true;
        } 

        //handling packet with escape byte
        else if(startMsg == true && msg[i] == ESCBYTE){
            printf("\nFound Escape Byte");
            parsedMsg[parsedLen++] = msg[i + 1];
            i++;
        }

        //handling regular packet
        else if(startMsg == true && msg[i] != STOPBYTE && msg[i] != ESCBYTE){
            parsedMsg[parsedLen++] = msg[i];
        }
        
        //handling stop byte
        else if(startMsg == true && msg[i] == STOPBYTE && msg[i-1] != ESCBYTE){
            printf("\nEnd of Message");
            startMsg = false;
        }
        i++;
    }

    #ifdef DEBUG
    printf("\nParsed Message: \n");
    for(i = 0; i < parsedLen; i++){
        printf("%04x ", parsedMsg[i]);
    }
    #endif

    //update message and packet length
    *length = parsedLen;
    for(i = 0; i < parsedLen; msg[i] = parsedMsg[i], i++);
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
    uint8_t len = 4;
    msg[0] = 0xAA;
    msg[1] = 0x7D;
    msg[2] = 0x7D;
    msg[3] = 0xDD;
    printf("Original Message:\n");
    for(int j = 0; j < len; printf("%04x ",msg[j]), j++);
    //printf("%d", *lenp);
    convertMsg(0x01, 0x01, msg, &len);

    #ifdef DEBUG
    int i =0;
    printf("\nSent Payload: \n");
    for(i = 0; i < len; i++){
        printf("%04x ", msg[i]);
    }
    #endif
    
    parsePayload(msg, &len);

    #ifdef DEBUG
    i = 0;
    printf("\nReceived Payload: \n");
    for(i = 0; i < len; i++){
        printf("%04x ", msg[i]);
    }
    #endif

    bool crcCheck;
    crcCheck = checkCRC(msg, len);
    if (crcCheck == true) printf("\nPacket maintains integrity\n");
    else printf("\nPacket malformed\n");
    return 0;
}