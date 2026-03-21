
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
#ifndef BICA_CONTROL_MAIN_H
#define BICA_CONTROL_MAIN_H
#include "bica_control_common.h"
#include "controller/include/common.h"

_control_buffer* current_window = nullptr;

int _bica_create_control_buffer(struct _control_buffer **buf_ptr, Input input){
    (*buf_ptr) = (_control_buffer*) calloc(1, sizeof(struct _control_buffer));

    if((*buf_ptr) == nullptr)
        return EMALLOC;

    if(current_window == nullptr)
        (*buf_ptr) -> rolling_count = 0;
    else
        (*buf_ptr) -> rolling_count = (current_window->rolling_count + 1) % 4;
    
    (*buf_ptr) -> update_status = 0;
    (*buf_ptr) -> control_type = CTRL_TYPE_INPUTS;
    
    (*buf_ptr) -> data[0] = FLOAT_AS_LONG(input.ballast);
    (*buf_ptr) -> data[1] = FLOAT_AS_LONG(input.left);
    (*buf_ptr) -> data[2] = FLOAT_AS_LONG(input.right);
    
    return EOK;
}

int _bica_create_control_buffer(struct _control_buffer **buf_ptr, State current, State desired, Pose acceleration){
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

//0x45 bica_send_control_upd
int _bica_0x44_control_rep_process(unsigned char * buffer, int buffer_len, void* data){
    if(buffer_len < BICA_BUFFER_LEN)
        return EBADBUFFER;
    uint8_t rc = (buffer[1] >> 6) & 0b11;
    
    return EOK;
}

int _bica_0x45_control_upd_create(unsigned char * buffer, int buffer_len, void* data){
    if(buffer_len < BICA_BUFFER_LEN)
        return EBADBUFFER; 
    
    if(data != nullptr){
        // This is called by the user, so we need to update the current_window
        free(current_window);
        current_window = (_control_buffer*) data;
    }

    buffer[0] = 0x45;
    buffer[1] = ((current_window->rolling_count & 0b11) << 6)
              | ((current_window->control_type & 0b11) << 4);

    int num_inputs = 0;
    if(current_window->control_type == CTRL_TYPE_INPUTS)
        num_inputs = INPUT_LENGTH;
    else if (current_window->control_type == CTRL_TYPE_STATES)
        num_inputs = 2*STATE_LENGTH+ACC_LENGTH;
    
    uint8_t  total_packed = 0;
    uint32_t vars_packed = 0;

    for(uint16_t i = 1; i <= num_inputs; i++){
        // Check if the specific recieved bit is marked as received. If so, we can continue.
        if((current_window->update_status << (num_inputs - i)) & 0b1 == 1)
            continue;
        
        vars_packed |= 1 << (num_inputs - i);
        buffer[4*total_packed+3] = 0xFF & (current_window->data[i]);
        buffer[4*total_packed+4] = 0xFF & (current_window->data[i] >> 8);
        buffer[4*total_packed+5] = 0xFF & (current_window->data[i] >> 16);
        buffer[4*total_packed+6] = 0xFF & (current_window->data[i] >> 24);

        total_packed ++;
        if(total_packed >= VAR_PER_MSG)
            break;
    }
    buffer[1] |= 0b111 & (vars_packed <<16);
    buffer[2]  = 0xFF  & (vars_packed <<8);
    buffer[3]  = 0xFF  & (vars_packed);

    return EOK;
}

int _bica_init_control_main(){
    // Fail if floats are not size 4. Yeah, we got not better solution on this one LMAO
    if( sizeof(float) != 4 )
        return EFLOATSIZE;
    
    // Setup our function hooks (yippee)
    bica_set_hook(0x44, BICAT_PROCESS, _bica_0x44_control_rep_process);
    bica_set_hook(0x45, BICAT_CREATE, _bica_0x45_control_upd_create);
}

#endif