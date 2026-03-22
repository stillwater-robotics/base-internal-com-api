
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
#include "bica_control_common.h"

// Usable Functions
int bica_init_bcu_control(_bica_ctrl_send_callback _send_callback, _update_controller_callback _ctrl_callback);

_control_buffer recieve_buffer;
_bica_ctrl_send_callback send_callback = nullptr;
_update_controller_callback controller_callback = nullptr;

int _reset_receive_buffer(uint8_t _rolling_count, uint8_t _control_type){
    recieve_buffer.control_type = _control_type;
    recieve_buffer.rolling_count = _rolling_count;
    recieve_buffer.update_status = 0;
    for(int i = 0; i < CTRL_BUFFER_SIZE; i++)
        recieve_buffer.data[i] = 0;
    return EOK;
}

int _bica_0x44_control_rep_create(unsigned char * buffer, int buffer_len, void* data){
    if(buffer_len < BICA_BUFFER_LEN)
        return EBADBUFFER;
    
    
    return EOK;
}

int _bica_0x45_control_upd_process(unsigned char * buffer, int buffer_len, void* data){
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
    received_map |= ((uint32_t)buffer[2] & 0b111) << 16;
    received_map |= ((uint32_t)buffer[3] & 0xFF) << 8;
    received_map |= ((uint32_t)buffer[4] & 0xFF);

    int num_processed = 0;

    for(int i = 0; i < num_inputs; i++){
        if((received_map << (num_inputs - i)) & 0b1 == 0)
            continue;
        uint32_t received = 0;
        received |= ((uint32_t)buffer[4*num_processed+3]) << 24;
        received |= ((uint32_t)buffer[4*num_processed+4]) << 16;
        received |= ((uint32_t)buffer[4*num_processed+5]) << 8;
        received |= ((uint32_t)buffer[4*num_processed+6]);
        recieve_buffer.data[i] = received;
        num_processed ++;
    }

    //check if we have full frame and update control w callback if so

    //send a rep message via the callback

    return EOK;
}

int bica_init_bcu_control(_bica_ctrl_send_callback _send_callback, _update_controller_callback _ctrl_callback){
    // Fail if floats are not size 4. Yeah, we got not better solution on this one LMAO
    if( sizeof(float) != 4 )
        return EFLOATSIZE;

    // Fail if no valid function callback is set.
    if(_send_callback == nullptr || _ctrl_callback == nullptr)
        return EINVALIDSETUP;

    controller_callback = _ctrl_callback;
    send_callback = _send_callback;
    _reset_receive_buffer();
    bica_set_hook(0x44, BICAT_CREATE, _bica_0x44_control_rep_create);
    bica_set_hook(0x45, BICAT_PROCESS, _bica_0x45_control_upd_process);
}

#endif