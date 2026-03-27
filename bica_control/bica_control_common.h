
/*
 * #         __  |__                          #
 * #      __L L_|L L__    Stillwater Robotics #
 * #...[+(____________)          -2025-       #
 * #       C_________/                        #
 *  
 * bica_control_common.h
 * Created: Oct 20, 2025
 * Last Edited: Mar 24 2026
 * 
 * This file defines control message constants used by both sides.
*/
#ifndef BICA_CONTROL_COMMON_H
#define BICA_CONTROL_COMMON_H

#ifndef ARDUINO
#include "stdint.h"
#endif
#include "../bica.h"
#ifndef COMMON_HEADER
#include "controller/include/common.h"
#endif

// great stuff going on here lmfao
#define FLOAT_AS_LONG(num) *((uint32_t*)((float*)&num))
#define LONG_AS_FLOAT(num) *((float*)((uint32_t*)&num))

#ifndef EOK
#define EOK             0x0
#endif

#define EFLOATSIZE      0x24
#define EMALLOC         0x25
#define EBADBUFFER      0x26
#define EWINDOWMISMATCH 0x27
#define EINVALIDSETUP   0x28

// State Constructor
#define STATE_LENGTH 8
#define ACC_LENGTH 3
#define INPUT_LENGTH 3

#define CTRL_TYPE_STATES 0b01
#define CTRL_TYPE_INPUTS 0b10

#define CTRL_COMPLETE_STATES 0x07FFFF
#define CTRL_COMPLETE_INPUTS 0b111

#define CTRL_BUFFER_SIZE 2 * STATE_LENGTH + ACC_LENGTH

struct _control_buffer{
    uint8_t     rolling_count;
    uint8_t     control_type;
    uint32_t    update_status;
    uint32_t    data[CTRL_BUFFER_SIZE];
};

typedef int (*_bica_ctrl_send_callback)(unsigned char* buffer, int buffer_len);
typedef int (*_update_controller_callback_states)(State current, State desired, Pose acceleration);
typedef int (*_update_controller_callback_inputs)(Input input);

#define VAR_PER_MSG (BICA_BUFFER_LEN-4)/4


#endif