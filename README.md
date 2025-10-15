# base-internal-com-api (BICA)
bas-internal-com-api, or BICA, is a shared header file library. It is used in stillwater robotic's MCU and BCU messaging implementations for internal communications.

BICA provides a set of messages to be implemented, a framework for implementing message creation and processing, and includes some default message creation and processing functions. It is made to be overridable per system and customizable.

BICA does *not* provide code for actually sending messages, only creating and receiving. Sending is left up to implementations, as BICA messages are designed to be work with many systems (CAN, Serial).

## Implementing BICA
The idea behind the bica framework is to use a 'lookup' to call message creation and processing functions. This lookup is called `bica_get_function(int message_id, int type)`. 
- valid `message_id`s are listed below in the Message Table section, and defined in `bica.h` as `BICAM_MESSAGE_NAME`
- valid types are `BICAT_CREATE` and `BICAT_PROCESS`, defined in `bica.h`

//TODO: More Here

## Message Tables
### Direction & Description
#### `0x0X` Safety Messages
| ID     | Name                    | Direction | Description             | 
| ------ | ----------------------- | --------- | ----------------------- |
| `0x01` | Safety Override Rep     | From BCU  | Replies to `0x03` acceptance or rejection of an override request. |
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
| `0x44` | Send State Error Upd    | To BCU    | To be rethought |
| `0x45` | Send State Desired Upd  | To BCU    | To be rethought |
| `0x46` | Send State Estimate Upd | To BCU    | To be rethought |

#### `0xEX` Admin Messages
| ID     | Name                    | Direction | Description             | 
| ------ | ----------------------- | --------- | ----------------------- |
| `0xE1` | Handshake Rep           | All       | Reply to `0xE2` with the BICA version. |
| `0xE2` | Handshake Req           | All       | Sent by a joining computer to verify BICA version from other members. |

#### `0xFX` Testing Messages
| ID     | Name                    | Direction | Description             | 
| ------ | ----------------------- | --------- | ----------------------- |
| `0xFF` | Test Message            | All       | Test Message filled with sequential data. |

### Bitmaps
## Contributing to BICA