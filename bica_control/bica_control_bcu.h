
/*
 * #         __  |__                          #
 * #      __L L_|L L__    Stillwater Robotics #
 * #...[+(____________)          -2025-       #
 * #       C_________/                        #
 *  
 * bica_control_bcu.h
 * Created: Mar 14, 2025
 * Last Edited: Mar 24 2026
 * 
 * This file defines control message creation used in the internal communications system.
*/
#ifndef BICA_CONTROL_BCU_H
#define BICA_CONTROL_BCU_H
#include "bica_control_common.h"

// Usable Functions
int bica_init_bcu_control(_bica_ctrl_send_callback _send_callback, 
    _update_controller_callback_states _ctrl_callback_states, 
    _update_controller_callback_inputs _ctrl_callback_inputs);

_control_buffer recieve_buffer;
_bica_ctrl_send_callback send_callback = nullptr;
_update_controller_callback_states controller_callback_states = nullptr;
_update_controller_callback_inputs controller_callback_inputs = nullptr;

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
    buffer[0] = BICAM_SEND_CONTROL_REP;
    buffer[1] = ((recieve_buffer.rolling_count & 0b11) << 6)
              | ((recieve_buffer.update_status >> 16) & 0b111);
    buffer[2] = (recieve_buffer.update_status >> 8) & 0xFF;
    buffer[3] = (recieve_buffer.update_status) & 0xFF;

    for(int i = 4; i < buffer_len; i++)
        buffer[i] = 0;
    
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
    recieve_buffer.update_status == CTRL_COMPLETE_STATES){
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
    recieve_buffer.update_status == CTRL_COMPLETE_INPUTS ){
        controller_callback_inputs(
            Input(LONG_AS_FLOAT(recieve_buffer.data[0]), 
                  LONG_AS_FLOAT(recieve_buffer.data[1]), 
                  LONG_AS_FLOAT(recieve_buffer.data[2])
            )
        );
    }
       
    uint8_t callback_buffer[BICA_BUFFER_LEN];
    _bica_m_function_ptr _func = bica_get_function(BICAM_SEND_CONTROL_REP, BICAT_CREATE);
    if(_func == nullptr)
        return EINVALIDSETUP;
    int process_return = _func(callback_buffer, BICA_BUFFER_LEN, nullptr);
    if(process_return != EOK) return process_return;
    send_callback(callback_buffer, BICA_BUFFER_LEN);

    return EOK;
}

int bica_init_bcu_control(_bica_ctrl_send_callback _send_callback, 
    _update_controller_callback_states _ctrl_callback_states, 
    _update_controller_callback_inputs _ctrl_callback_inputs){
    // Fail if floats are not size 4. Yeah, we got not better solution on this one LMAO
    if( sizeof(float) != 4 )
        return EFLOATSIZE;

    // Fail if no valid function callback is set.
    if(_send_callback == nullptr || _ctrl_callback_states == nullptr || _ctrl_callback_inputs == nullptr)
        return EINVALIDSETUP;

    controller_callback_states = _ctrl_callback_states;
    controller_callback_inputs = _ctrl_callback_inputs;
    send_callback = _send_callback;
    _reset_receive_buffer(0, 0);
    bica_set_hook(BICAM_SEND_CONTROL_REP, BICAT_CREATE, _bica_0x44_control_rep_create);
    bica_set_hook(BICAM_SEND_CONTROL_UPD, BICAT_PROCESS, _bica_0x45_control_upd_process);
    return EOK;
}

#endif