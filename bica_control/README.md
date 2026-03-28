# BICA Control
This set of control messages is made to easily slot into the system, both on the main compute and bcu.

Here is an example of using the control library for BICA:
```cpp
#include "bica_control.h"

int transmit_function(unsigned char* buffer, int buffer_len);

void setup(){
    //Initialize your choice of MAIN or BCU functions
    init_bica_control_main(transmit_function, nullptr, nullptr);
    init_bica_control_bcu(transmit_function, nullptr, nullptr);
    // Set up the NULLPTRs as callbacks for receiving states and inputs respectively
    Serial.begin(BAUD_RATE);
}

void loop(){

    // 1. SENDING DATA
    /* Create data to send */
    Input to_send_1; 
    State to_send_2a, to_send_2b, to_send_2c;
    //YOUR CODE HERE: Propogate to_send as you want

    /* Format data into a control buffer */
    bica_control_buffer *control_buffer;
    create_bica_control_buffer(&control_buffer, to_send_1);
    //create_bica_control_buffer(&control_buffer, to_send_2a, to_send_2b, to_send_2c);
    send_bica_control_buffer(control_buffer); // Detects whether you are BCU or Main from init functions

    // 2. RECEIVING DATA
    /* We use a standard BICA processing loop here. */
    // If a full message is availble, you should process a message.
    // Use a while loop if you always want to process all available.
    if(Serial.available() >= BICA_BUFFER_LEN){
        // Read in a full message
        unsigned char read_in[BICA_BUFFER_LEN];
        for(int i = 0; i < BICA_BUFFER_LEN; i++)
            read_in[i] = Serial.read();
        // Process the message based on header byte
        get_bica_function(read_in[0], BICAT_PROCESS)(read_in, BICA_BUFFER_LEN, nullptr);
    }
}

// Define a function which transmits a buffer of size buffer_len
// Ideally, this should fill to a BICA_BUFFER_LEN size with extra 0 values.
// For BICA CONTROL only this is unnecessary.
int transmit_function(unsigned char* buffer, int buffer_len){
    for( int i = 0; i < buffer_len; i++ )
        Serial.write(buffer[i]);

    // Better Implementation:
    // for(int i = 0; i < BICA_BUFFER_LEN; i++)
    //     Serial.write((i < buffer_len)? buffer[i]: 0);
}

```