
/*
 * #         __  |__                          #
 * #      __L L_|L L__    Stillwater Robotics #
 * #...[+(____________)          -2025-       #
 * #       C_________/                        #
 *  
 * bica_control.h
 * Created: Oct 20, 2025
 * Last Edited: Mar 24 2026
 * 
 * This file defines control message constants used by both sides.
*/
#ifndef BICA_CONTROL_H
#define BICA_CONTROL_H

#ifndef ARDUINO
#include "stdint.h"
#endif
#include "../bica.h"
#ifndef COMMON_HEADER
#include "controller/include/common.h"
#endif

/* Define Useful Conversion Macros*/
#define FLOAT_AS_LONG(num) *((uint32_t*)((float*)&num))
#define LONG_AS_FLOAT(num) *((float*)((uint32_t*)&num))

/* Define Error IDs */
#ifndef EOK
#define EOK             0x0
#endif
#ifndef EFLOATSIZE
#define EFLOATSIZE      0x24
#endif
#ifndef EMALLOC
#define EMALLOC         0x25
#endif
#ifndef EBADBUFFER
#define EBADBUFFER      0x26
#endif
#ifndef EWINDOWMISMATCH
#define EWINDOWMISMATCH 0x27
#endif
#ifndef EINVALIDSETUP
#define EINVALIDSETUP   0x28
#endif

/* Define some simple info regarding sendable objects */
#define STATE_LENGTH 8
#define ACC_LENGTH 3
#define INPUT_LENGTH 3

/* Define some message parameters and mask utilities */
#define CTRL_TYPE_STATES 0b01
#define CTRL_TYPE_INPUTS 0b10
#define CTRL_COMPLETE_STATES 0x07FFFF
#define CTRL_COMPLETE_INPUTS 0b111
#define CTRL_BUFFER_SIZE (2 * STATE_LENGTH + ACC_LENGTH)
#define VAR_PER_MSG (BICA_BUFFER_LEN-4)/4

/* Define a structure to store sendable object data */
struct _control_buffer{
    uint8_t     rolling_count;
    uint8_t     control_type;
    uint32_t    update_status;
    uint32_t    data[CTRL_BUFFER_SIZE];
};

/* Define function and buffer pointer types */
typedef _control_buffer bica_control_buffer;
typedef int (*_bica_ctrl_send_callback)(unsigned char* buffer, int buffer_len);
typedef int (*_update_controller_callback_states)(State current, State desired, Pose acceleration);
typedef int (*_update_controller_callback_inputs)(Input input);
typedef Input (*_get_controller_inputs)();

/* Declare all user-facing functions */
//Init Functions
int init_bica_control_bcu(_bica_ctrl_send_callback _send_callback, 
    _update_controller_callback_states _ctrl_callback_states, 
    _update_controller_callback_inputs _ctrl_callback_inputs,
    _get_controller_inputs _ctrl_get_input);
int init_bica_control_main(_bica_ctrl_send_callback _send_callback, 
    _update_controller_callback_states _ctrl_callback_states, 
    _update_controller_callback_inputs _ctrl_callback_inputs);
// User-facing helper functions
int create_bica_control_buffer(bica_control_buffer **buf_ptr, Input input);
int create_bica_control_buffer(bica_control_buffer **buf_ptr, State current, State desired, Pose acceleration);
int send_bica_control_buffer(bica_control_buffer **buf_ptr);

/* Declare all internal functions */
// Process & Create Functions
int _bica_control_leader_create(unsigned char * buffer, int buffer_len, void* data, uint8_t id);    // 0x42 & 0x45
int _bica_control_leader_process(unsigned char * buffer, int buffer_len, void* data);
int _bica_control_follower_create(unsigned char * buffer, int buffer_len, void* data, uint8_t id);  // 0x41 & 0x44
int _bica_control_follower_process(unsigned char * buffer, int buffer_len, void* data);
int _bica_control_request_leader_create(unsigned char * buffer, int buffer_len, void* data);        // 0x43
int _bica_control_request_leader_process(unsigned char * buffer, int buffer_len, void* data);
// Helper Functions
int _reset_receive_buffer(uint8_t _rolling_count, uint8_t _control_type);

/* Internally Used Variables */
bool bcu_init = false;
bool main_init = false;
_control_buffer *current_window = nullptr;
_control_buffer recieve_buffer;

_bica_ctrl_send_callback            send_callback = nullptr;
_update_controller_callback_states  controller_callback_states = nullptr;
_update_controller_callback_inputs  controller_callback_inputs = nullptr;
_get_controller_inputs              controller_get_current_inputs = nullptr;

/* Helper Function Definitions */
int _reset_receive_buffer(uint8_t _rolling_count, uint8_t _control_type){
    recieve_buffer.control_type = _control_type;
    recieve_buffer.rolling_count = _rolling_count;
    recieve_buffer.update_status = 0;
    for(int i = 0; i < CTRL_BUFFER_SIZE; i++)
        recieve_buffer.data[i] = 0;
    return EOK;
}

/* Create & Process Functions */
int _bica_control_request_leader_create(unsigned char * buffer, int buffer_len, void* data){
    if(buffer_len < BICA_BUFFER_LEN)
        return EBADBUFFER;
    buffer[0] = BICAM_REQ_INPUT_UPD;
    return EOK;
}

int _bica_control_request_leader_process(unsigned char * buffer, int buffer_len, void* data){
    if(buffer_len < BICA_BUFFER_LEN)
        return EBADBUFFER;
    if(controller_get_current_inputs == nullptr)
        return EINVALIDSETUP;

    uint8_t callback_buffer[BICA_BUFFER_LEN];
    _bica_m_function_ptr _func = bica_get_function(BICAM_SEND_INPUT_UPD, BICAT_CREATE);
    if(_func == nullptr)
        return EINVALIDSETUP;
    _control_buffer * buf;
    create_bica_control_buffer(&buf, controller_get_current_inputs());
    
    int process_return = _func(callback_buffer, BICA_BUFFER_LEN, buf);
    if(process_return != EOK) return process_return;
    if(send_callback)
        send_callback(callback_buffer, BICA_BUFFER_LEN);
    return EOK;
}

int _bica_control_follower_create(unsigned char * buffer, int buffer_len, void* data, uint8_t id){
    if(buffer_len < BICA_BUFFER_LEN)
        return EBADBUFFER;
    buffer[0] = id;
    buffer[1] = ((recieve_buffer.rolling_count & 0b11) << 6)
              | ((recieve_buffer.update_status >> 16) & 0b111);
    buffer[2] = (recieve_buffer.update_status >> 8) & 0xFF;
    buffer[3] = (recieve_buffer.update_status) & 0xFF;

    for(int i = 4; i < buffer_len; i++)
        buffer[i] = 0;
    
    return EOK;
}

int _bica_control_follower_process(unsigned char * buffer, int buffer_len, void* data){
    if(buffer_len < BICA_BUFFER_LEN)
        return EBADBUFFER;

    if(send_callback == nullptr)
        return EINVALIDSETUP;

    uint8_t rc = (buffer[1] >> 6) & 0b11;
    if(rc != current_window->rolling_count)
        return EWINDOWMISMATCH;
    
    uint32_t processed = 0;
    processed |= ((uint32_t)buffer[1] & 0b111) << 16;
    processed |= ((uint32_t)buffer[2] & 0xFF) << 8;
    processed |= ((uint32_t)buffer[3] & 0xFF);

    current_window -> update_status = processed;

    if(current_window -> control_type == CTRL_TYPE_INPUTS
        && processed == CTRL_COMPLETE_INPUTS)
        return EOK;
    if(current_window -> control_type == CTRL_TYPE_STATES
        && processed == CTRL_COMPLETE_STATES)
        return EOK;

    uint8_t callback_buffer[BICA_BUFFER_LEN];
    _bica_m_function_ptr _func = bica_get_function(buffer[0]+1, BICAT_CREATE);
    if(_func == nullptr)
        return EINVALIDSETUP;
    int process_return = _func(callback_buffer, BICA_BUFFER_LEN, nullptr);
    if(process_return != EOK) return process_return;
    if(send_callback)
        send_callback(callback_buffer, BICA_BUFFER_LEN);
    
    return EOK;
}

int _bica_control_leader_create(unsigned char * buffer, int buffer_len, void* data, uint8_t id){
    if(buffer_len < BICA_BUFFER_LEN)
        return EBADBUFFER; 
    
    if(data != nullptr){
        // This is called by the user, so we need to update the current_window
        free(current_window);
        current_window = (_control_buffer*) data;
    }

    buffer[0] = id;
    buffer[1] = ((current_window->rolling_count & 0b11) << 6)
              | ((current_window->control_type & 0b11) << 4);

    int num_inputs = 0;
    if(current_window->control_type == CTRL_TYPE_INPUTS)
        num_inputs = INPUT_LENGTH;
    else if (current_window->control_type == CTRL_TYPE_STATES)
        num_inputs = CTRL_BUFFER_SIZE;
    
    uint8_t  total_packed = 0;
    uint32_t vars_packed = 0;

    for(uint16_t i = 1; i <= num_inputs; i++){
        // Check if the specific recieved bit is marked as received. If so, we can continue.
        if(((current_window->update_status >> (num_inputs - i)) & 0b1) == 1)
            continue;
        
        vars_packed |= 1l << (num_inputs - i);
        buffer[4*total_packed+4] = 0xFF & (current_window->data[i-1] >> 24);
        buffer[4*total_packed+5] = 0xFF & (current_window->data[i-1] >> 16);
        buffer[4*total_packed+6] = 0xFF & (current_window->data[i-1] >> 8);
        buffer[4*total_packed+7] = 0xFF & (current_window->data[i-1] );
        
        total_packed ++;
        if(total_packed >= VAR_PER_MSG)
            break;
    }
    buffer[1] |= 0b111 & (vars_packed >>16);
    buffer[2]  = 0xFF  & (vars_packed >>8);
    buffer[3]  = 0xFF  & (vars_packed);

    return EOK;
}

int _bica_control_leader_process(unsigned char * buffer, int buffer_len, void* data){
    if(buffer_len < BICA_BUFFER_LEN)
        return EBADBUFFER;
    
    uint8_t rc = (buffer[1] >> 6) & 0b11;
    uint8_t ct = (buffer[1] >> 4) & 0b11;
    if(rc != recieve_buffer.rolling_count || ct != recieve_buffer.control_type)
        _reset_receive_buffer(rc, ct);    
    
    int num_inputs = 0;
    if(ct == CTRL_TYPE_INPUTS)
        num_inputs = INPUT_LENGTH;
    else if (ct == CTRL_TYPE_STATES)
        num_inputs = CTRL_BUFFER_SIZE;
    
    uint32_t received_map = 0;
    received_map |= ((uint32_t)buffer[1] & 0b111) << 16;
    received_map |= ((uint32_t)buffer[2] & 0xFF) << 8;
    received_map |= ((uint32_t)buffer[3] & 0xFF);

    int num_processed = 0;

    for(int i = 1; i <= num_inputs; i++){
        if(((received_map >> (num_inputs - i)) & 0b1) == 0)
            continue;
        uint32_t received = 0;
        received |= ((uint32_t)buffer[4*num_processed+4]) << 24;
        received |= ((uint32_t)buffer[4*num_processed+5]) << 16;
        received |= ((uint32_t)buffer[4*num_processed+6]) << 8;
        received |= ((uint32_t)buffer[4*num_processed+7]);
        recieve_buffer.data[i-1] = received;
        
        recieve_buffer.update_status |= (0b1 << (num_inputs - i));
        num_processed ++;
    }

    //check if we have full frame and update control w callback if so
    if(recieve_buffer.control_type == CTRL_TYPE_STATES && 
    recieve_buffer.update_status == CTRL_COMPLETE_STATES &&
    controller_callback_states != nullptr){
        controller_callback_states(
            State(
                Pose(LONG_AS_FLOAT(recieve_buffer.data[0]),
                     LONG_AS_FLOAT(recieve_buffer.data[1]),
                     LONG_AS_FLOAT(recieve_buffer.data[2]),
                     LONG_AS_FLOAT(recieve_buffer.data[3])
                ),
                Pose(LONG_AS_FLOAT(recieve_buffer.data[4]),
                     LONG_AS_FLOAT(recieve_buffer.data[5]),
                     LONG_AS_FLOAT(recieve_buffer.data[6]),
                     LONG_AS_FLOAT(recieve_buffer.data[7])
                )
            ),
            State(
                Pose(LONG_AS_FLOAT(recieve_buffer.data[8]),
                     LONG_AS_FLOAT(recieve_buffer.data[9]),
                     LONG_AS_FLOAT(recieve_buffer.data[10]),
                     LONG_AS_FLOAT(recieve_buffer.data[11])
                ),
                Pose(LONG_AS_FLOAT(recieve_buffer.data[12]),
                     LONG_AS_FLOAT(recieve_buffer.data[13]),
                     LONG_AS_FLOAT(recieve_buffer.data[14]),
                     LONG_AS_FLOAT(recieve_buffer.data[15])
                )
            ),
            Pose(LONG_AS_FLOAT(recieve_buffer.data[16]),
                 LONG_AS_FLOAT(recieve_buffer.data[17]),
                 LONG_AS_FLOAT(recieve_buffer.data[18]),
                 0.00
            )
        );
    }
    if(recieve_buffer.control_type == CTRL_TYPE_INPUTS && 
    recieve_buffer.update_status == CTRL_COMPLETE_INPUTS &&
    controller_callback_inputs != nullptr){
        controller_callback_inputs(
            Input(LONG_AS_FLOAT(recieve_buffer.data[0]), 
                  LONG_AS_FLOAT(recieve_buffer.data[1]), 
                  LONG_AS_FLOAT(recieve_buffer.data[2])
            )
        );
    }
       
    uint8_t callback_buffer[BICA_BUFFER_LEN];
    _bica_m_function_ptr _func = bica_get_function(buffer[0]-1, BICAT_CREATE);
    if(_func == nullptr)
        return EINVALIDSETUP;
    int process_return = _func(callback_buffer, BICA_BUFFER_LEN, nullptr);
    if(process_return != EOK) return process_return;
    if(send_callback)
        send_callback(callback_buffer, BICA_BUFFER_LEN);

    return EOK;
}

/* User Functions */
int create_bica_control_buffer(struct _control_buffer **buf_ptr, Input input){
    (*buf_ptr) = (_control_buffer*) calloc(1, sizeof(struct _control_buffer));

    if((*buf_ptr) == nullptr)
        return EMALLOC;

    if(current_window == nullptr)
        (*buf_ptr) -> rolling_count = 0;
    else
        (*buf_ptr) -> rolling_count = (current_window->rolling_count + 1) % 4;
    
    (*buf_ptr) -> update_status = 0;
    (*buf_ptr) -> control_type = CTRL_TYPE_INPUTS;
    
    (*buf_ptr) -> data[0] = FLOAT_AS_LONG(input.left);
    (*buf_ptr) -> data[1] = FLOAT_AS_LONG(input.right);
    (*buf_ptr) -> data[2] = FLOAT_AS_LONG(input.ballast);
    
    return EOK;
}

int create_bica_control_buffer(struct _control_buffer **buf_ptr, State current, State desired, Pose acceleration){
    (*buf_ptr) = (_control_buffer*) calloc(1, sizeof(struct _control_buffer));

    if((*buf_ptr) == nullptr)
        return EMALLOC;

    if(current_window == nullptr)
        (*buf_ptr) -> rolling_count = 0;
    else
        (*buf_ptr) -> rolling_count = (current_window->rolling_count + 1) % 4;
    
    (*buf_ptr) -> update_status = 0;
    (*buf_ptr) -> control_type = CTRL_TYPE_STATES;
    
    (*buf_ptr) -> data[0] = FLOAT_AS_LONG(current.pose.x);
    (*buf_ptr) -> data[1] = FLOAT_AS_LONG(current.pose.y);
    (*buf_ptr) -> data[2] = FLOAT_AS_LONG(current.pose.z);
    (*buf_ptr) -> data[3] = FLOAT_AS_LONG(current.pose.theta);
    (*buf_ptr) -> data[4] = FLOAT_AS_LONG(current.velocity.x);
    (*buf_ptr) -> data[5] = FLOAT_AS_LONG(current.velocity.y);
    (*buf_ptr) -> data[6] = FLOAT_AS_LONG(current.velocity.z);
    (*buf_ptr) -> data[7] = FLOAT_AS_LONG(current.velocity.theta);

    (*buf_ptr) -> data[8]  = FLOAT_AS_LONG(desired.pose.x);
    (*buf_ptr) -> data[9]  = FLOAT_AS_LONG(desired.pose.y);
    (*buf_ptr) -> data[10] = FLOAT_AS_LONG(desired.pose.z);
    (*buf_ptr) -> data[11] = FLOAT_AS_LONG(desired.pose.theta);
    (*buf_ptr) -> data[12] = FLOAT_AS_LONG(desired.velocity.x);
    (*buf_ptr) -> data[13] = FLOAT_AS_LONG(desired.velocity.y);
    (*buf_ptr) -> data[14] = FLOAT_AS_LONG(desired.velocity.z);
    (*buf_ptr) -> data[15] = FLOAT_AS_LONG(desired.velocity.theta);

    (*buf_ptr) -> data[16] = FLOAT_AS_LONG(acceleration.x);
    (*buf_ptr) -> data[17] = FLOAT_AS_LONG(acceleration.y);
    (*buf_ptr) -> data[18] = FLOAT_AS_LONG(acceleration.z);
    
    return EOK;
}

int send_bica_control_buffer(bica_control_buffer **buf_ptr){
    if(!buf_ptr)
        return EBADBUFFER;
    if(!(*buf_ptr))
        return EBADBUFFER;
    if(!bcu_init && ! main_init)
        return EINVALIDSETUP;

    unsigned char buf[BICA_BUFFER_LEN];
    if((*buf_ptr)->control_type == CTRL_TYPE_INPUTS)
        bica_get_function(bcu_init? BICAM_SEND_INPUT_UPD: BICAM_SEND_CONTROL_UPD, BICAT_CREATE)(buf, BICA_BUFFER_LEN, *buf_ptr);
    else if((*buf_ptr)->control_type == CTRL_TYPE_INPUTS)
        bica_get_function(main_init? BICAM_SEND_CONTROL_UPD: BICAM_SEND_INPUT_UPD, BICAT_CREATE)(buf, BICA_BUFFER_LEN, *buf_ptr);
    
    send_callback(buf, BICA_BUFFER_LEN);
    return EOK;
}

int init_bica_control_bcu(_bica_ctrl_send_callback _send_callback, 
    _update_controller_callback_states _ctrl_callback_states, 
    _update_controller_callback_inputs _ctrl_callback_inputs,
    _get_controller_inputs _ctrl_get_input){
    // Fail if floats are not size 4. Yeah, we got not better solution on this one LMAO
    if( sizeof(float) != 4 )
        return EFLOATSIZE;

    // Fail if no valid function callback is set.
    if(_send_callback == nullptr)
        return EINVALIDSETUP;

    controller_callback_states = _ctrl_callback_states;
    controller_callback_inputs = _ctrl_callback_inputs;
    send_callback = _send_callback;
    controller_get_current_inputs = _ctrl_get_input;
    _reset_receive_buffer(0, 0);
    bica_set_hook(BICAM_SEND_INPUT_REP, BICAT_PROCESS, _bica_control_follower_process);
    bica_set_hook(BICAM_SEND_INPUT_UPD, BICAT_CREATE, [](unsigned char * buffer, int buffer_len, void* data)->int{
        return _bica_control_leader_create(buffer, buffer_len, data, BICAM_SEND_INPUT_UPD);});
    bica_set_hook(BICAM_REQ_INPUT_UPD, BICAT_PROCESS, _bica_control_request_leader_process);
    bica_set_hook(BICAM_SEND_CONTROL_REP, BICAT_CREATE, [](unsigned char * buffer, int buffer_len, void* data)->int{
        return _bica_control_follower_create(buffer, buffer_len, data, BICAM_SEND_CONTROL_REP);});
    bica_set_hook(BICAM_SEND_CONTROL_UPD, BICAT_PROCESS, _bica_control_leader_process);
    bcu_init = true;
    return EOK;
}

int init_bica_control_main(_bica_ctrl_send_callback _send_callback, 
    _update_controller_callback_states _ctrl_callback_states, 
    _update_controller_callback_inputs _ctrl_callback_inputs){
    // Fail if floats are not size 4. Yeah, we got no better solution on this one LMAO
    if( sizeof(float) != 4 )
        return EFLOATSIZE;

    // Fail if no valid function callback is set.
    if(_send_callback == nullptr)
        return EINVALIDSETUP;

    controller_callback_states = _ctrl_callback_states;
    controller_callback_inputs = _ctrl_callback_inputs;
    send_callback = _send_callback;
    _reset_receive_buffer(0, 0);
    bica_set_hook(BICAM_SEND_INPUT_REP, BICAT_CREATE, [](unsigned char * buffer, int buffer_len, void* data)->int{
        return _bica_control_follower_create(buffer, buffer_len, data, BICAM_SEND_INPUT_REP);});
    bica_set_hook(BICAM_SEND_INPUT_UPD, BICAT_PROCESS, _bica_control_leader_process);
    bica_set_hook(BICAM_REQ_INPUT_UPD, BICAT_CREATE, _bica_control_request_leader_create);
    bica_set_hook(BICAM_SEND_CONTROL_REP, BICAT_PROCESS, _bica_control_follower_process);
    bica_set_hook(BICAM_SEND_CONTROL_UPD, BICAT_CREATE, [](unsigned char * buffer, int buffer_len, void* data)->int{
        return _bica_control_leader_create(buffer, buffer_len, data, BICAM_SEND_CONTROL_UPD);});
    main_init = true;
    return EOK;
}

#endif