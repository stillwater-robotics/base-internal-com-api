
/*
 * #         __  |__                          #
 * #      __L L_|L L__    Stillwater Robotics #
 * #...[+(____________)          -2025-       #
 * #       C_________/                        #
 *  
 * bica_control.h
 * Created: Oct 20, 2025
 * Last Edited: Oct 20, 2025
 * 
 * This file defines control message creation used in the internal communications system.
*/
#ifndef BICA_CONTROL_H
#include "../bica.h"

//0x41 & 0x42

//0x45 


//0x46 bica_send_control_upd
struct bica_struct_send_control_upd{
    int rolling_count;
    int more_in_frame;
    int resolution;
    uint64_t data;
};

#endif