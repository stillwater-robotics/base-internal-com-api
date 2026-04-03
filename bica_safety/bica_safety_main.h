/*
 * #         __  |__                          #
 * #      __L L_|L L__    Stillwater Robotics #
 * #...[+(____________)          -2025-       #
 * #       C_________/                        #
 *  
 * bica_control.h
 * Created: Apr 2 2025
 * Last Edited: Apr 2 2026
 * 
 * This file defines control message constants used by both sides.
*/
#ifndef BICA_SAFETY_MAIN
#define BICA_SAFETY_MAIN

#ifndef ARDUINO
#include "stdint.h"
#endif
#include "../bica.h"

#ifndef EOK
#define EOK 0x0
#endif
#ifndef EBADBUFFER
#define EBADBUFFER 0x26
#endif
#ifndef EBADDATA 
#define EBADDATA 0x29
#endif

// 0x8 SAFETY_TRG
#define _BICA_SAFETY_TRG_12V_CUTOFF 0b1
#define _BICA_SAFETY_TRG_HARD_RESET 0b100
#define _BICA_SAFETY_TRG_CUT_MOTORS 0b10000

int _bica_safety_trigger_create(unsigned char * buffer, int buffer_len, void* data){
    if(!buffer || buffer == nullptr)
        return EBADBUFFER;
    if(buffer_len <= 3)
        return EBADBUFFER;
    if(!data) 
        return EBADDATA;

    buffer[0] = BICAM_SAFETY_TRG;
    buffer[1] = *((uint8_t*)data);

    return EOK;
}

int init_bica_safety_main(){
    return EOK;
}

#endif