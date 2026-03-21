
/*
 * #         __  |__                          #
 * #      __L L_|L L__    Stillwater Robotics #
 * #...[+(____________)          -2025-       #
 * #       C_________/                        #
 *  
 * bica_control.h
 * Created: Oct 20, 2025
 * Last Edited: Mar 14 2026
 * 
 * This file defines control message creation used in the internal communications system.
*/
#ifndef BICA_CONTROL_BCU_H
#define BICA_CONTROL_BCU_H
#include "../bica.h"

#include "stdint.h"
#include "bica_control_common.h"


_control_buffer* front;
_control_buffer* back;


// 0x44 and 0x45

int _bica_0x44_control_rep_create(unsigned char * buffer, int buffer_len, void* data){
    if(buffer_len < )
}

#endif