#include "in4073.h"


void write_packet_flash(){
    uint8_t packet[42];
    uint32_t timestamp = get_time_us();
    int16_t tmp;
    static int j = 0;

    uint32_t add = 0x000000+j;

    if (add >0x01FFFF){
        flash_full = true;
        printf("FLASH MEMORY IS FULL!\n");
    }
    else
        j=j+42;

    if (!flash_full && start_logging){
        packet[0] = fourthByte(timestamp);
        packet[1] = thirdByte(timestamp);
        packet[2] = secondByte(timestamp);
        packet[3] = lowByte(timestamp);
        packet[4] = state;
        packet[5] = start_logging;
        for(int i = 0; i < 4; ++i) {
            packet[6 + 2 * i] = highByte(ae[i]);
            packet[6 + 2 * i + 1] = lowByte(ae[i]);
        }
        tmp = phi - phi_avg;
        packet[14] = highByte(tmp);
        packet[15] = lowByte(tmp);
        tmp = theta - theta_avg;
        packet[16] = highByte(tmp);
        packet[17] = lowByte(tmp);
        tmp = f_d.phi_kalman;
        packet[18] = highByte(tmp);
        packet[19] = lowByte(tmp);
        tmp = f_d.sax_filtered;
        packet[20] = highByte(tmp);
        packet[21] = lowByte(tmp);
        tmp = f_d.say_filtered;
        packet[22] = highByte(tmp);
        packet[23] = lowByte(tmp);
        tmp = sr - sr_avg;
        packet[24] = highByte(tmp);
        packet[25] = lowByte(tmp);
        tmp = sax - sax_avg;
        packet[26] = highByte(tmp);
        packet[27] = lowByte(tmp);
        tmp = say - say_avg;
        packet[28] = highByte(tmp);
        packet[29] = lowByte(tmp);
        tmp = f_d.sr_filtered;
        packet[30] = highByte(tmp);
        packet[31] = lowByte(tmp);
        packet[32] = highByte(bat_volt);
        packet[33] = lowByte(bat_volt);
        packet[34] = fourthByte(temperature); //SAVE ADDRESS JUST FOR TESTING
        packet[35] = thirdByte(temperature);
        packet[36] = secondByte(temperature);
        packet[37] = lowByte(temperature);
        packet[38] = fourthByte(pressure);
        packet[39] = thirdByte(pressure);
        packet[40] = secondByte(pressure);
        packet[41] = lowByte(pressure);

        //printf("\nWriting into memory = ");
        flash_write_bytes(add, &packet[0],42);
        //printf("%d\n",write_flash);
    }
}

void read_packet_flash(uint8_t * packet){
    static int j = 0;

    uint32_t add = 0x000000+j;
    if (add >0x01FFFF){
        read_completed = true;
        printf("FINISHING READING!\n");
    }
    else
        j=j+42;

	flash_read_bytes(add,&packet[0],42);

}

void print_to_terminal(uint8_t * packet){  //THIS HAS TO BE REPLACED BY THE PROTOCOL, FROM DRONE2PC

    if (combine32Byte(packet[34],packet[35],packet[36],packet[37]) == 0xffffffff){
        printf("READING COMPLETED!\n");
        read_completed = true;
    }
    else {
        printf("%10ld ",combine32Byte(packet[0],packet[1],packet[2],packet[3]));
        printf("%1d ",packet[4]);
        printf("%1d ",packet[5]);
        printf("%3d %3d %3d %3d ",combineByte(packet[6],packet[7]),combineByte(packet[8],packet[9]),combineByte(packet[10],packet[11]),combineByte(packet[12],packet[13]));
        printf("%6d %6d %6d ",combineByte(packet[14],packet[15]),combineByte(packet[16],packet[17]),combineByte(packet[18],packet[19]));
        printf("%6d %6d %6d ",combineByte(packet[20],packet[21]),combineByte(packet[22],packet[23]),combineByte(packet[24],packet[25]));
        printf("%6d %6d %6d ",combineByte(packet[26],packet[27]),combineByte(packet[28],packet[29]),combineByte(packet[30],packet[31]));
        printf("%4d ", combineByte(packet[32],packet[33]));
        printf("%4ld %6ld ",combine32Byte(packet[34],packet[35],packet[36],packet[37]),combine32Byte(packet[38],packet[39],packet[40],packet[41]));
        printf("\n");
    }
}
