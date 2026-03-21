
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
 * This file defines control message constants used by both sides.
*/
#ifndef BICA_CONTROL_COMMON_H
#define BICA_CONTROL_COMMON_H

#ifndef ARDUINO
#include "stdint.h"
#endif
#include "../bica.h"

#define FLOAT_AS_LONG(num) *((uint32_t*)((float*)&num))
#define LONG_AS_FLOAT(num) *((float*)((uint32_t*)&num))

#define EOK         0x0
#define EFLOATSIZE  0x1
#define EMALLOC     0x2
#define EBADBUFFER  0x3

// State Constructor
#define STATE_LENGTH 8
#define ACC_LENGTH 3
#define INPUT_LENGTH 3

#define CTRL_TYPE_STATES 0b01
#define CTRL_TYPE_INPUTS 0b10

struct _control_buffer{
    uint8_t     rolling_count;
    uint8_t     control_type;
    uint32_t    update_status;
    uint32_t    data[2 * STATE_LENGTH + ACC_LENGTH];
};

#define VAR_PER_MSG (BICA_BUFFER_LEN-4)/4


#endif