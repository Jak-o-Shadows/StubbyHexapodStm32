#include "serialServo/serialServo.h"


// Helper Functions

//  Prepare the actual data to send
//      Split into two functions because it needs to be two separate bytes
uint8_t prepareCommand(uint8_t command, uint16_t argument){
    uint8_t byte1 = (uint8_t) (command << 3) | (argument & 0x07);
    return byte1;
}
uint8_t prepareArgument(uint16_t argument){
    uint8_t byte2 = (uint8_t) (argument >> 3) | 0x80;
    return byte2;
}



// Interface Functions

void serialServo_setup(uint32_t usart){
    // Nothing much to do here

}



void serialServo_setServoPos(uint32_t usart, uint8_t servo, uint16_t pos){

    uint8_t byte1;
    uint8_t byte2;

    // Tell servo to listen
    byte1 = prepareCommand(serialServo_COMMANDlistenOnce, servo);
    byte2 = prepareArgument(servo);
    usart_send_blocking(usart, byte1);
    usart_send_blocking(usart, byte2);

    // Tell servo the value
    byte1 = prepareCommand(serialServo_COMMANDsetPosition, pos);
    byte2 = prepareArgument(pos);
    usart_send_blocking(usart, byte1);
    usart_send_blocking(usart, byte2);


}


void serialServo_setServoCachePos(uint32_t usart, uint8_t servo, uint16_t pos){


}


void serialServo_executeCachePos(uint32_t usart, uint8_t servo){


}