#include "in4073.h"


void write_packet_flash(){
    uint8_t packet[20];
    uint32_t timestamp = get_time_us();
    static int j = 0;

    uint32_t add = 0x000000+j;

    if (add >0x01FFFF){
        flash_full = true;
    }
    else
        j=j+20;

    if (!flash_full && start_logging){
        packet[0] = fourthByte(timestamp);
        packet[1] = thirdByte(timestamp);
        packet[2] = secondByte(timestamp);
        packet[3] = lowByte(timestamp);
        // packet[4] = highByte(sp-sp_avg); //COMMENT THIS OUT
        // packet[5] = lowByte(sp-sp_avg);
        // packet[6] = highByte(sq-sq_avg);
        // packet[7] = lowByte(sq-sq_avg);
        packet[4] = highByte(p_kalman);
        packet[5] = lowByte(p_kalman);
        packet[6] = highByte(q_kalman);
        packet[7] = lowByte(q_kalman);
        packet[8] = highByte(sr - sr_avg);
        packet[9] = lowByte(sr - sr_avg);
        packet[10] = highByte(f_d.sr_filtered);
        packet[11] = lowByte(f_d.sr_filtered);
        packet[12] = highByte(f_d.phi_kalman);
        packet[13] = lowByte(f_d.phi_kalman);
        packet[14] = highByte(f_d.theta_kalman);
        packet[15] = lowByte(f_d.theta_kalman);
        packet[16] = highByte(sax - sax_avg);
        packet[17] = lowByte(sax - sax_avg);
        packet[18] = highByte(say - say_avg);
        packet[19] = lowByte(say - say_avg);

        flash_write_bytes(add, &packet[0],20);
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
        j=j+20;

	flash_read_bytes(add,&packet[0],20);

}

void print_to_terminal(uint8_t * packet){  //THIS HAS TO BE REPLACED BY THE PROTOCOL, FROM DRONE2PC
    if (combine32Byte(packet[0],packet[1],packet[2],packet[3]) == 0xffffffff){
        printf("READING COMPLETED!\n");
        read_completed = true;
    }
    else{
        printf("%10ld ",combine32Byte(packet[0],packet[1],packet[2],packet[3]));
        printf("%6d %6d %6d ",combineByte(packet[4],packet[5]),combineByte(packet[6],packet[7]),combineByte(packet[8],packet[9]));
        printf("%6d ", combineByte(packet[10],packet[11]));
        printf("%6d %6d ",combineByte(packet[12],packet[13]),combineByte(packet[14],packet[15]));
        printf("%6d %6d ",combineByte(packet[16],packet[17]),combineByte(packet[18],packet[19]));
        printf("\n");
    }
}
