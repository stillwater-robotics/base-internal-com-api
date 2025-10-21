# base-internal-com-api (BICA)
bas-internal-com-api, or BICA, is a shared header file library. It is used in stillwater robotic's MCU and BCU messaging implementations for internal communications.

BICA provides a set of messages to be implemented, a framework for implementing message creation and processing, and includes some default message creation and processing functions. It is made to be overridable per system and customizable.

BICA does *not* provide code for actually sending messages, only creating and receiving. Sending is left up to implementations, as BICA messages are designed to be work with many systems (CAN, Serial).

## Implementing BICA
### Function Lookup
The idea behind the bica framework is to use a 'lookup' to call message creation and processing functions. This lookup is called `bica_get_function(int message_id, int type)`. The power of this implementation is threefold:
- The programmer does not need to remember the function associated with each message, or create their own lookup.
- Functions can be overwritten or removed dynamically
- Implementations become specific to the needs of each platform, meaning little overhead (for Arduino).

Message IDs and bitmaps are available for reference in their own sections below.

Let's now take a look at function lookup and using bica functions.
```cpp
bica_func bica_get_function(int message_id, int type);
int message_id //any BICAM_*
int type       //any BICAT_*
return         //requested function pointer of type bica_func (internally referred to as _bica_m_function_ptr)
```
- valid `message_id`s are listed below in the Message Table section, and defined in `bica.h` as `BICAM_MESSAGE_NAME`
- valid types are `BICAT_CREATE` and `BICAT_PROCESS`, defined in `bica.h`
- returns `nullptr` if not found (If compiled for C, `nullptr` is automatically defined as `NULL`)

Looking ahead, let's say your code wants to process a message it just received. With BICA, this can be done as:
```cpp
// Assume this is populated with our message.
unsigned char message[BICA_BUFFER_LEN];
// The first byte is the message ID:
int message_id = message[0];
// Lookup our processing function:
bica_func process_msg = bica_get_function(message_id, BICAT_PROCESS);
// Check for nullptr & process
if(process_msg != nullptr)
    process_msg(message, BICA_BUFFER_LEN, NULL);
```

Similarly, we can create a test message (ID `0xFF`) as such:
```cpp
unsigned char message[BICA_BUFFER_LEN];
bica_func create_msg = bica_get_function(BICAM_TEST_DUMMY, BICAT_CREATE);
if(create_msg != nullptr)
    create_msg(message, BICA_BUFFER_LEN, NULL); //Populate the message buffer
```

### Layout of a BICA Message
BICA messages are represented by byte arrays. You may represent a byte with any data type which has size of 1UL, including `unsigned char` or `uint8_t`. 

BICA messages have a length of `BICA_BUFFER_LENGTH` by default. This includes the first byte, the message id header, and all data in the message. To reiterate, valid message ids are defined with the prefix `BICAM_*`.
```
0xX0 X1 X2 X3 X4 X5 .... XN
X0: Header
X1 ... XN: Defined per message.
```

You can use static or dynamically allocated arrays to store messages with BICA. BICA provides a function which automatically allocates a buffer of size `BICA_BUFFER_LEN`:
```cpp
unsigned char* bica_allocate_buffer();
return //pointer to a buffer of type unsigned char* with length BICA_BUFFER_LEN. Must be freed using free().
```

Note that BICA does not enforce the use of `BICA_BUFFER_LEN`, and provides `buffer_len` options for create and process functions. `BICA_BUFFER_LEN` is simply useful for implementations which choose to use fixed-length messaging.

### Function Registration
By default, BICA comes with a only a few creation functions defined. You will need to register your own processing functions and a few creation functions on your own.

All BICA functions (Processing or Creation) share the same template:
```cpp
typedef int (*bica_func)(unsigned char* buffer, int buffer_len, void* data);
unsigned char* buffer // A BICA message buffer, written to if BICAT_CREATE, read from if BICAT_PROCESS
int buffer_len        // The length of the message buffer
void* data            // Any arbitrary pointer, for implementations which may need it.
return (int)          // Whether the function succeeded or encountered an error. 
```

Program your processing/creation following the function template above. Message IDs and bitmaps are available for reference in their own sections below. Once this is done, you'll need to register them with BICA at the start of your program (or at anytime before you attempt to retrive them with `bica_get_function()`). \\
Registration is done via `bica_set_hook()`:
```cpp
int bica_set_hook(int message_id, int type, bica_func to_add);
int message_id      // The message id your function is designed for
int type            // BICAT_CREATE or BICAT_PROCESS
bica_func to_add    // Pointer to/address of your function
return (int)        // Whether or not it succeeded
```

## Message IDs
#### `0x0X` Safety Messages
| ID     | Name                    | Direction | Description             | 
| ------ | ----------------------- | --------- | ----------------------- |
| `0x01` | Safety Override Rep     | From BCU  | Replies to `0x02` acceptance or rejection of an override request. |
| `0x02` | Safety Override Req     | To BCU    | Main Compute requests to override the BCU's safety takeover with its own control. |
| `0x06` | Safety Takeover Ind     | From BCU  | The BCU's safety system notifies the main compute of its own safety takeover. |
| `0x08` | Safety Trg              | To BCU    | Trigger the BCU's safety protocol based on information from the Main Compute. |
| `0x0d` | Safety Sensor Rep       | From BCU  | Replies to `0x0e` with a reading of the safety system's sensors. |
| `0x0e` | Safety Sensor Req       | To BCU    | Requests sensor data from the safety system. |

#### `0x2X` Sensor Query & Collision Avoidance 
| ID     | Name                    | Direction | Description             | 
| ------ | ----------------------- | --------- | ----------------------- |
| `0x21` | Collision Ind           | From BCU  | Indicates that the Collision Avoidance filter is being applied to control inputs |
| `0x24` | Sensor Rep              | From BCU  | Replies to `0x24` with requested sensor data. |
| `0x25` | Sensor Req              | To BCU    | Requests body sensor data (non-safety related sensors). |

#### `0x4X` Control
| ID     | Name                    | Direction | Description             | 
| ------ | ----------------------- | --------- | ----------------------- |
| `0x41` | Query Control Rep       | From BCU  | Replies to `0x42` with the requested stored control data. |
| `0x42` | Query Control Req       | To BCU    | Requests for the BCU to send over some stored control data. |
| `0x44` | Send Control Rep        | From BCU  | Reply sent after receiving a full control frame. |
| `0x45` | Send Control Upd        | To BCU    | Sent as a frame (series of messages) updating multiple control inputs. |

#### `0xEX` Admin Messages
| ID     | Name                    | Direction | Description             | 
| ------ | ----------------------- | --------- | ----------------------- |
| `0xE1` | Handshake Rep           | All       | Reply to `0xE2` with the BICA version. |
| `0xE2` | Handshake Req           | All       | Sent by a joining computer to verify BICA version from other members. |

#### `0xFX` Testing Messages
| ID     | Name                    | Direction | Description             | 
| ------ | ----------------------- | --------- | ----------------------- |
| `0xFE` | Reserved                | All       | Used to test nullptr returns on lookup. |
| `0xFF` | Test Dummy Message      | All       | Test Message filled with sequential data. |

## Recommended Message Bitmaps
Bit numbers are big-endian. Byte numbers start at 1, as Byte 0 is reserved for the Message ID for all messages. \
**`BX`** represents Byte X. \
**`BX 0xF0`** represents the bits masked by `0xF0` at Byte X, etc. \
**`N`** Represents the last Byte
| ID     | Name                    | Bitmap             | 
| ------ | ----------------------- | -------------------|
| **CONTROL** | |
| `0x44` | Send Control Rep        | **`B1 0x3`:** 2 Bit Rolling Count of the replied to frame. <br> **`B2`** combined flags of all received control inputs. |
| `0x45` | Send Control Upd        | **`B1 0x3`:** 2 Bit Rolling Count, from `0` to `3` consistent to all messages in a frame. <br> **`B1 0x4`:** boolean, true if more messages are left in the frame. <br> **`B1 0xF0`:** Number of bytes in the transmitted control input <br> **`B2`:** Type of control input to set as a flag. <br> **`B3...BN`:** Big-Endian data to be sent/read into the control input. |
| **TEST** | |
| `0xFE` | Reserved | Reserved for unit tests to use. No assigned bitmap. |
| `0xFF` | Test Dummy Message      | **`B1...BN`:** written as the number `i`, where i is the current Byte Number. |

## Included Functions