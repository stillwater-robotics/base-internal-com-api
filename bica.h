/*
 * #         __  |__                          #
 * #      __L L_|L L__    Stillwater Robotics #
 * #...[+(____________)          -2025-       #
 * #       C_________/                        #
 *  
 * bica.h
 * Created: Oct 2, 2025
 * Last Edited: Oct 10, 2025
 * 
 * This file defines messages used in the internal communications system.
*/
/* Check if C or C++ */
#ifndef __cpluscplus
/* Compiling as C */
  #define nullptr NULL
  #include <stdlib.h>
#else
/* Compiling as C++ */
  #include <cstdlib>
#endif

/* Check if Arduino */
#ifndef ARDUINO
#include <stdio.h>
#endif

/* Define Constants */
#define BICA_VERSION 1
#define BICA_BUFFER_LEN 8

 //##### Message ID Definitions #####
 // SAFETY
#define BICAM_SAFETY_OVERRIDE_REP 0x01
#define BICAM_SAFETY_OVERRIDE_REQ 0x02
#define BICAM_SAFETY_TAKEOVER_IND 0x06
#define BICAM_SAFETY_TRG          0x08
#define BICAM_SAFETY_SENSOR_REP   0x0d
#define BICAM_SAFETY_SENSOR_REQ   0x0e

 // SENSOR QUERY & COLLISION NOTICE
#define BICAM_COLLISION_IND 0x21
#define BICAM_SENSOR_REP 0x24
#define BICAM_SENSOR_REQ 0x25

 // CONTROL MESSAGES
#define BICAM_QUERY_CONTROL_REP 0x41
#define BICAM_QUERY_CONTROL_REQ 0x42
#define BICAM_SEND_STATE_ERROR_UPD 0x44
#define BICAM_SEND_STATE_DESIRED_UPD 0x45
#define BICAM_SEND_STATE_ESTIMATE_UPD 0x46

//ADMIN MESSAGES
#define BICAM_HANDSHAKE_REP 0xE1
#define BICAM_HANDSHAKE_REQ 0xE2

//TEST MESSAGES
#define BICAM_TEST_BLANK 0xFF


//##### Function Hook Definitions #####
#define BICAT_CREATE 0 
#define BICAT_PROCESS 1
typedef int (*_bica_m_function_ptr)(unsigned char* buffer, int buffer_len, void* data);
_bica_m_function_ptr bica_get_function(unsigned char message_id, int type);

struct _bica_m_lookup_entry{
unsigned char message_id;
_bica_m_function_ptr create, process;
};

//##### Processing Functions #####
// bicad stands for BICA Default
void _bicad_on_nullptr(unsigned char message_id, int type, int id_found);
int _bicad_testblank_create(unsigned char * buffer, int buffer_len, void* data);

// ADD NEW MESSAGES HERE, WITH nullptr ENTRIES!
// KEEP THIS SORTED.
#define BICA_NUM_MESSAGE_IDS 17 //Increment with new messages
struct _bica_m_lookup_entry _bica_m_lookup_table[] = {
{BICAM_SAFETY_OVERRIDE_REP,     nullptr,                    nullptr},
{BICAM_SAFETY_OVERRIDE_REQ,     nullptr,                    nullptr},
{BICAM_SAFETY_TAKEOVER_IND,     nullptr,                    nullptr},
{BICAM_SAFETY_TRG,              nullptr,                    nullptr},
{BICAM_SAFETY_SENSOR_REP,       nullptr,                    nullptr},
{BICAM_SAFETY_SENSOR_REQ,       nullptr,                    nullptr},
{BICAM_COLLISION_IND,           nullptr,                    nullptr},
{BICAM_SENSOR_REP,              nullptr,                    nullptr},
{BICAM_SENSOR_REQ,              nullptr,                    nullptr},
{BICAM_QUERY_CONTROL_REP,       nullptr,                    nullptr},
{BICAM_QUERY_CONTROL_REQ,       nullptr,                    nullptr},
{BICAM_SEND_STATE_ERROR_UPD,    nullptr,                    nullptr},
{BICAM_SEND_STATE_DESIRED_UPD,  nullptr,                    nullptr},
{BICAM_SEND_STATE_ESTIMATE_UPD, nullptr,                    nullptr},
{BICAM_HANDSHAKE_REP,           nullptr,                    nullptr},
{BICAM_HANDSHAKE_REQ,           nullptr,                    nullptr},
{BICAM_TEST_BLANK,              _bicad_testblank_create,    nullptr}
};

// Called when nothing is found.
void (*bica_on_nullptr)(unsigned char message_id, int type, int index_found) = _bicad_on_nullptr;
int bica_add_hook(int message_id, int type, _bica_m_function_ptr to_add);

// ##### Default Function Implementations #####
unsigned char* bica_allocate_buffer(){
  unsigned char * buf = (unsigned char *)calloc(BICA_BUFFER_LEN, sizeof(unsigned char));
  if(buf == nullptr) return nullptr;
  for (int i = 0; i < BICA_BUFFER_LEN; i++) buf[i] = 0;
  return buf;
} 

int _bica_get_index(int message_id){
  int upper_bound = BICA_NUM_MESSAGE_IDS - 1;
  int lower_bound = 0;
  int midpoint = 0;
  int index_found = -1;

  while(upper_bound >= lower_bound){
    midpoint = (upper_bound + lower_bound)/2;
    if(message_id == _bica_m_lookup_table[midpoint].message_id){
      index_found = midpoint;
      break;
    }

    if(message_id < _bica_m_lookup_table[midpoint].message_id)
      upper_bound = midpoint - 1;
    if(message_id > _bica_m_lookup_table[midpoint].message_id)
      lower_bound = midpoint + 1;
  }
  return index_found;
}

int bica_set_hook(int message_id, int type, _bica_m_function_ptr to_add){
  int index = _bica_get_index(message_id);
  if(index < 0 || index >= BICA_NUM_MESSAGE_IDS)
    return 0;
  
  if(type == BICAT_CREATE)
    _bica_m_lookup_table[index].create = to_add;
  if(type == BICAT_PROCESS)
    _bica_m_lookup_table[index].process = to_add;
  
  return 1;
}

_bica_m_function_ptr bica_get_function(unsigned char message_id, int type){
  _bica_m_function_ptr to_return = nullptr;

  int index = _bica_get_index(message_id);
  if(index < BICA_NUM_MESSAGE_IDS && index >= 0)
    to_return = (type == BICAT_CREATE)?_bica_m_lookup_table[index].create: _bica_m_lookup_table[index].process;

  if(to_return == nullptr)
    bica_on_nullptr(message_id, type, index);

  return to_return;
}

int _bicad_testblank_create(unsigned char * buffer, int buffer_len, void* data){
  for(int i = 0; i < buffer_len; i++)
    buffer[i] = (unsigned char) i;
  return 1;
}

void _bicad_on_nullptr(unsigned char message_id, int type, int id_found){
#ifndef ARDUINO
  printf("BICA Lookup failed to find function %2x, %s. \n", message_id, type == BICAT_CREATE? "CREATE": "PROCESS");
#endif
}
