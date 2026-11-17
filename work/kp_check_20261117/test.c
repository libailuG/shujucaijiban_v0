#include <assert.h>
#include <stdio.h>
#include "motor_protocol.h"
int main(void){
    for(unsigned id=126;id<=127;id++) {
        MotorFrame frame={0};uint8_t bytes[48];float decoded;
        frame.motor_id=id;frame.mode=MOTOR_TORQUE;frame.torque=3.0f;frame.KP=10.0f;
        motor_encode(bytes,&frame);
        assert(bytes[32]==0 && bytes[33]==0 && bytes[34]==0x20 && bytes[35]==0x41);
        memcpy(&decoded,bytes+32,4);assert(decoded==10.0f);
        assert(motor_u32(bytes+4)==id && motor_u32(bytes+8)==1);
        printf("ID %u mode1 KP=%.1f bytes[32..35]=%02X %02X %02X %02X\n",id,decoded,bytes[32],bytes[33],bytes[34],bytes[35]);
    }
    return 0;
}
