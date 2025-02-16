/**
 * @file
 * @brief This file definies the serial communication protocol for tendon
 * control
 *
 */
#ifndef COMM_HPP
#define COMM_HPP

// #include "hardware/Motor/MotorControl.h"
// #include <stdint.h>

// /**
//  * @brief Maximum packet size acceptable for this application
//  */
// #define PKT_MAX_NUM_BYTES_IN_FRAME 32

// /**
//  * @brief Number of bytes consumer by packet header
//  */
// #define PKT_NUM_HEADER_BYTES 2

// /**
//  * @brief Number of bytes used for opcpde
//  */
// #define PKT_NUM_OPCODE_BYTES 1

// /**
//  * @brief Number of bytes used for motor ID
//  */
// #define PKT_NUM_ID_BYTES 1

// /**
//  * @brief Number of bytes used for packet length field
//  */
// #define PKT_NUM_LEN_BYTES 1

// /**
//  * @brief Number of bytes used for CRC
//  */
// #define PKT_NUM_CRC_BYTES 2

// /**
//  * @brief Maximum number of bytes in the parameters array
//  */
// #define PKT_MAX_NUM_PARAM_BYTES \
//   PKT_MAX_NUM_BYTES_IN_FRAME - PKT_NUM_HEADER_BYTES - PKT_NUM_OPCODE_BYTES -
//   \
//       PKT_NUM_ID_BYTES - PKT_NUM_LEN_BYTES

// #define MAKE_16B_WORD(a, b) ((uint16_t)a << 8) | ((uint16_t)b)
// #define GET_UPPER_16B(a) (uint8_t)(((uint16_t)a >> 8) & 0xFF)
// #define GET_LOWER_16B(a) (uint8_t)((uint16_t)a & 0xFF)

// /**
//  * @brief Enum defining opcodes for motor tendon control
//  *
//  * The protocol parses data packets (see the data packet structure) and
//  performs
//  * the following functions as follows:
//  *
//  * READ_STATUS: Reads the status of the motor specified by motor ID. If motor
//  ID
//  * is given to be 0xFF, then READ_STATUS will perform a multiple read
//  command.
//  * In this case you must pass the ID of each motor whose status you want to
//  * read. The order of the IDs does not matter.
//  *
//  * For instance if you would like to read the status of motors 2, 4, 6, the
//  * params would be:
//  *
//  * [ 0x02 ][ 0x04 ][ 0x06 ]
//  *
//  * READ_ANGLE: Reads the angle of the motor specified my motor ID. If the
//  * specified ID is 0xFF, then READ_ANGLE will perform a multiple read
//  command.
//  * You must pass the ID of each motor whose angle you want to read. The order
//  of
//  * the IDs does not matter.
//  *
//  * For instance if you would like to read the angles of motors 2, 4, 6, the
//  * params would be:
//  *
//  * [ 0x02 ][ 0x04 ][ 0x06 ]
//  *
//  * WRITE_ANGLE: Writes a goal angle (signed 16-bit integer) to the motor
//  * specifed by motor ID. This function requires 2 parameters in the following
//  * order:
//  *
//  * Param 1                  Param 2
//  * [ GOAL ANGLE BYTES HIGH ][ GOAL ANGLE BYTES LOW ]
//  *
//  * If the specified ID is 0xFE, then the request becomes a multiple write
//  * command. You must pass as a parameter the ID of each motor you want to
//  write
//  * immediately followed by the goal angle high and low bytes. The order of
//  motor
//  * IDs does not matter, however, the goal angle high and low bytes must
//  * immediately follow their corresponding motor ID.
//  *
//  * For instance, if you would like to write motor 2 to 0 degrees and motor 4
//  to
//  * 5 degrees, the params would be:
//  *
//  * Motor ID 1  Goal Angle 1 High   Goal Angle 1 Low    Motor ID 2  Goal Angle
//  2
//  * High   Goal Angle 2 Low [   0x02   ][       0x00       ][       0x00 ][
//  * 0x04   ][       0x00       ][       0x00       ]
//  *
//  * [ MOTOR ID 1 ][ MOTOR ID 1 GOAL ANGLE HIGH BYTES ][ MOTOR ID 2 GOAL ANGLE
//  * HIGH BYTES ]
//  *
//  * For each motor you wish to write.
//  *
//  * WRITE_PID: Writes the PID parameters (signed 16-bit integers) to the motor
//  * specifed by motor ID. This function requires 6 parameters in the following
//  * order:
//  *
//  * [ P BYTES HIGH ][ P BYTES LOW ][ I BYTES HIGH ][ I BYTES LOW ][ D BYTES
//  HIGH
//  * ][ D BYTES LOW ]
//  *
//  * This function does not support multiple write operations.
//  *
//  * *Note*: Look into the possibility of multiple write operations. Only
//  problem
//  * is that the messages can get very long. Need to test if this causes any
//  * significant input lag.
//  *
//  */
// typedef enum {
//   ECHO = 0x00,
//   WRITE_WHEEL_SPEED = 0x01,
//   WRITE_SERVO_ANGLE = 0x03,
//   READ_WHEEL_SPEED = 0x04,
//   READ_DEAD_WHEELS = 0x05,
//   READ_STATUS,
//   READ_ANGLE,
//   WRITE_ANGLE,
//   WRITE_PID,
// } tendon_opcode_t;

// /**
//  * @brief Enum defining comm results for tendon control
//  *
//  */
// typedef enum {
//   COMM_SUCCESS,
//   COMM_FAIL,
//   COMM_INSTRUCTION_ERROR,
//   COMM_CRC_ERROR,
//   COMM_ID_ERROR,
//   COMM_PARAM_ERROR
// } tendon_comm_result_t;

// /**
//  * @brief Struct to define tendon control data packet
//  *
//  * This struct defines the data packets defined by the tendon control
//  * communication protocol The structure of a packet is as follows:
//  *
//  * [ HEADER 1 ][ HEADER 2 ][ LENGTH ][ MOTOR ID ][ OPCODE ][ PARAMS ][ CRC
//  HIGH
//  * ][ CRC LOW ]
//  *
//  * HEADER 1+2: Used as a packet delimiter to signifify the start of a new
//  * packet. Always the bytes 0xFF 0x00. LENGTH: 8-bit integer used to specify
//  the
//  * length of the packet. The header and length fields are not taken into
//  account
//  * when calculating length, so the length is calculated as 4 + number of
//  params
//  * (4 comes from opcode, params, and both CRC fields). MOTOR ID: The id of
//  the
//  * motor to read/write. Motors are assumed to be 1 indexed, so 0x00 is used
//  to
//  * read/write all motors. OPCODE: 8-bit integer used to command the tendon
//  * controller to perform a certain action (e.g. read/write angles, write PID,
//  * etc.) PARAMS: An array of 8-bit integers. Used as the "arguments" for the
//  * opcode. CRC HIGH+LOW: Together, form a a 16-bit CRC used for checking data
//  * corruption
//  *
//  * NOTE: When modifying this struct, keep field order and byte-alignment in
//  * mind.
//  *
//  * This data structure takes advantage of byte-alignment, allowing for direct
//  * casting of the input buffer directly to this object. This also allows for
//  * accessing fields using either array or struct accessors.
//  */
// typedef struct {
//   union {
//     uint8_t data_packet[PKT_MAX_NUM_BYTES_IN_FRAME];

//     // Domenic Note. I think that this needs to be updated?
//     // not all commands with use a motor ID
//     struct {
//       uint8_t header[PKT_NUM_HEADER_BYTES];
//       uint8_t len;
//       uint8_t motorId;
//       uint8_t opcode;
//       uint8_t pkt_params[PKT_MAX_NUM_PARAM_BYTES];
//     } data_packet_s;
//   } data_packet_u;
// } Crobot_data_packet_s;

// typedef struct {
//   Crobot_data_packet_s *rx_packet;
//   Crobot_data_packet_s *tx_packet;

//   uint8_t pkt_params[PKT_MAX_NUM_PARAM_BYTES];

//   tendon_comm_result_t comm_result;

// } Crobot_packet_handler_t;

// /**
//  * @brief Function used to obtain 16-bit CRC
//  *
//  * @param crc_accum Used to input running crc
//  * @param data The data to check
//  * @param data_blk_size The number of bytes in the data
//  * @return The 16-bit CRC as a uint16_t
//  */
// uint16_t updateCRC(uint16_t crc_accum, uint8_t *data, uint16_t
// data_blk_size);

// /**
//  * @brief This function parses an rx packet from a stream of bytes and
//  validates
//  * CRC
//  *
//  * @param pkt_handler the packet handler
//  * @param buff the buffer stream to process
//  */
// void parsePacket(Crobot_packet_handler_t *pkt_handler, const char *buff);

// /**
//  * @brief This function builds a tx packet including CRC
//  *
//  * @param pkt_handler the packet handler
//  */
// void buildPacket(Crobot_packet_handler_t pkt_handler, ...);

// // function to execute packet
// void execute(Crobot_packet_handler_t *pkt_handler);

#include "ArduinoJson.h"
#include "hardware/Motor/MotorControl.h"

typedef struct {
  MotorControl *FL;
  MotorControl *FR;
  MotorControl *BL;
  MotorControl *BR;
} devices;

class jetsonComms {
public:
  // constructor to set the devices inside the class
  jetsonComms(devices &d);

  // Take incoming jetson packet and execute the command
  void execute(const char *packet);

  // Take the data from the sensors and return a packet to send to the jetson
  JsonDocument buildStatus();

private:
  devices *bigD = nullptr;
  JsonDocument doc;
};
#endif