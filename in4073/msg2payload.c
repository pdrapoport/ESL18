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
        recChar[buffCount] = '\0'; // Used to detect if the message reception is not complete (and if not, to wait for it)
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
        tmp[count-i] = recChar[count]
    }

    strncpy(recChar, tmp, MAXMSG); // Store the remaining characters back into recChar

    buffCount = buffCount - (i-1); // Store the new length of recChar so that the future additions to it are done correctly

    // Enable interrupts back

    // Note: I think doing it like this avoids the need to allocate a temporary buffer for the packet to process: this should prevent a race condition between interrupt and the sliding function. It is the only place where we need to write to recChar, so there shouldn't be any problem for all the other interactions with this array (reads in the processPkt function)
}


// Author: Vincent Bejach
// Get the payload from the processed packet and store it in the global variable receivedMsg for further processing
void getPayload(uint8_t msglen) {
    uint8_t i, msgend = msglen - 16;

    for (i = 2; i<msgend; i++) { // Skip the first 2 bytes (start byte and packet header) and reads until the start of the CRC
        receivedMsg[i-2] = recChar[i];
    }
}


// Author: Vincent Bejach
/* Implement the FSM defined for the communication protocol. 
 * Reads from the global variable recChar, and remove part of its content when a packet is done being processed or when some bytes are thrown away.
 * Outputs the message of the packet being processed in the global receivedMsg array. The fnished processing is indicated by the flag messageComplete being set to true.
 */
void processPkt() {

    enum states {
        wait, first_byte_received, receiveMsg, processMsg, CRC_Check, transmit, panic
    } state;
    
    uint8_t i, msglen = 0;
    //uint16_t timeout; // TO BE IMPLEMENTED
    bool crc_result = false;
    bool panic_on = false; // Used to exit the loop in case of an emergency transition to panic mode

    state = wait;

    if (buffCount >= 12){ // Ensures we have already received at least 1 full message to avoid delay
// Note: message length can also be 38, but only when flushing drone flash memory. In this case, the drone doesn't do anything else, so it doesn't matter if there is delay in the execution of this function

        while(!messageComplete && !panic_on){

            switch(state) {
                case wait:
                    if (recChar[readIndex] == STARTBYTE){

                        readIndex++;
                        state = first_byte_received;

                    } else {

                        readIndex++; // The current byte is not STARTBYTE, so we want to check the next one
                    }

                    break;

                case first_byte_received:
                    msglen = cmd2len(recChar[readIndex++]);
                    state = receiveMsg;
                    
                    break;

                case receiveMsg:
                    i = 0;
                    while(i < (msglen-1)){ // Should be a timeout somewhere in case link is broken, which makes the FSM go into panic state - TO BE IMPLEMENTED
                        if (recChar[readIndex++] == '\0'){
                            // Do nothing and wait for the next packet to be received (ie loop here until the rest of the packet is received)

                            // The infinite loop risk is (will be) taken care of by the timeout
                        } else {
                            i++;
                        }
                    }

                    state = CRC_Check;
                    break;

                case CRC_Check:
                    crc_result = checkCRC(recChar, msglen);
                    if(crc_result == true){

                        state = processMsg;

                    } else {

                        // Sliding window (search for the next STARTBYTE character, and restart the beginning of the packet prccessing)
                        i = 0;
                        while(recChar[i] != STARTBYTE) {
                            i++;
                        }
                        i++; // Index of the next STARTBYTE

                        slideMsg(i); // Modify the recChar buffer to start at the newly chosen STARTBYTE
                        readIndex = 0; // As we discart everything previosu to the new STARTBYTE, making the array starting at 0 again, the readIndex should also be reset.
                        
                        state = first_byte_received; // Make sure the function restarts at the beginning of the packet processing
                        break;
                    }

                    break;

                case processMsg:
                    getPayload(msglen);

                    messageComplete = true; // Indicate to other functions that it can run and process the command
                    
                    slideMsg(msglen+1); // Remove the processed packet from the queue, and make recChar start at the following byte. The interrupts are handled inside the function, preventing race conditions with newly received characters

                    state = wait;

                case panic:
                    // Switch to panic mode - TO BE IMPLEMENTED
                    panic_on = true; // Allows to exit the while and to go on with the panic mode
                    break;

                default:
                    state = panic;
            }
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
