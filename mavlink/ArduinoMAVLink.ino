/* MAVLInk_DroneLights
 *  by Juan Pedro López
 *  
 * This program was developed to connect an Arduino board with a Pixhawk via MAVLink 
 *   with the objective of controlling a group of WS2812B LED lights on board of a quad
 * 
 * The current version of the program is working properly.
 * 
 * TO DO:
 *  - Move STREAMS request to RC_CHANNELS to use values in logic
 *  - Add RC_CHANNLES_RAW messages monitoring: move #30 to RC_CHANNELS_RAW (#35)
 *      http://mavlink.org/messages/common#RC_CHANNELS_RAW
 *  - Look for message on low battery:
 *      To be tested: http://mavlink.org/messages/common#PARAM_REQUEST_READ
 *      To be checked: http://mavlink.org/messages/common#SYS_STATUS
 *  - Potential implementation of other alarms, like high intensity
 *      
 * You can restrict the maximum package size with this parameter in mavlink_types.h:

    #ifndef MAVLINK_MAX_PAYLOAD_LEN_
    // it is possible to override this, but be careful! Defa_
    #define **MAVLINK_MAX_PAYLOAD_LEN 255 ///< Maximum payload length_
    #endif_
 */


#include "mavlink.h"

#define px4_serial Serial1
#define debug_serial Serial

volatile int servo_in = 1500;

// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int heartbeat_count = num_hbs;
const float MIN_BAT_VOLTAGE = 13.5;

volatile float battery_voltage = 0;


#define debug_serial Serial

void setup() {
  debug_serial.begin(115200);
  debug_serial.println("MAVLink starting.");
  px4_serial.begin(38400);
  initLeds();

}


void loop() {
 
  if (battery_voltage < MIN_BAT_VOLTAGE) {
    wipeHue(60);
    delay(200);
    wipeAll(0);
    delay(200);
  } else {
    wipePwm(servo_in);
  }
  // MAVLink
  /* The default UART header for your MCU */ 
  int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
  // TODO(johmathe): Whi is this 0
  int compid = 0;                ///< The component sending the message
  int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing
 
  // Define the system type, in this case an airplane -> on-board controller
  // TODO(johmathe): is this niecessary
  // uint8_t system_type = MAV_TYPE_FIXED_WING;
  // uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
 
  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_heartbeat_pack(sysid, compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 
  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    // Keep the last time the mode changed
    previousMillisMAVLink = currentMillisMAVLink;
    px4_serial.write(buf, len);
    heartbeat_count++;
    
    if(heartbeat_count >= num_hbs) {
      // Request streams from Pixhawk
      debug_serial.println("Streams requested!");
      Mav_Request_Data();
      heartbeat_count=0;
    }

  }

  // Check reception buffer
  comm_receive();
}

void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // To be setup according to the needed information to be requested from the Pixhawk
  const int  maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_RC_CHANNELS};
  const uint16_t MAVRates[maxStreams] = {0x01,0x08};

  for (int i=0; i < maxStreams; i++) {
    /*
     * mavlink_msg_request_data_stream_pack(system_id, component_id, 
     *    &msg, 
     *    target_system, target_component, 
     *    MAV_DATA_STREAM_POSITION, 10000000, 1);
     *    
     * mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, 
     *    mavlink_message_t* msg,
     *    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, 
     *    uint16_t req_message_rate, uint8_t start_stop)
     * 
     */
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    px4_serial.write(buf, len);
  }
}



/**
 * @brief Pack a mission_item message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param seq  Sequence
 * @param frame  The coordinate system of the waypoint.
 * @param command  The scheduled action for the waypoint.
 * @param current  false:0, true:1
 * @param autocontinue  Autocontinue to next waypoint
 * @param param1  PARAM1, see MAV_CMD enum
 * @param param2  PARAM2, see MAV_CMD enum
 * @param param3  PARAM3, see MAV_CMD enum
 * @param param4  PARAM4, see MAV_CMD enum
 * @param x  PARAM5 / local: X coordinate, global: latitude
 * @param y  PARAM6 / local: Y coordinate, global: longitude
 * @param z  PARAM7 / local: Z coordinate, global: altitude (relative or absolute, depending on frame).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
// static inline uint16_t mavlink_msg_mission_item_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,//
//                               uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z)

//mavlink_msg_mission_count_pack(2, 200, &msg, 1, 1, 2);
//
//// Wait for mission_request
//mavlink_msg_mission_item_pack(2, 200, &msg, 1, 1, 0, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 2, 0, 0, 52.464217, -1.280222, 0);
//// Wait for mission_request
//mavlink_msg_mission_item_pack(2, 200, &msg, 1, 1, 1, MAV_FRAME_GLOBAL, MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0);
//// Wait for mission_ack
//mavlink_msg_mission_ack_pack(2, 200, &msg, 1, 1, 0);
//
//
//
//MAV_CMD_NAV_RETURN_TO_LAUNCH

void comm_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;
 
  while(px4_serial.available() > 0) {
    uint8_t c = px4_serial.read();

    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            debug_serial.println("PX HB");
          }
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
             */
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
            battery_voltage = sys_status.voltage_battery;
          }
          break;

        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
//             */
//            mavlink_param_value_t param_value;
//            mavlink_msg_param_value_decode(&msg, &param_value);
//            debug_serial.println("PX PARAM_VALUE");
//            debug_serial.println(param_value.param_value);
//            debug_serial.println(param_value.param_count);
//            debug_serial.println(param_value.param_index);
//            debug_serial.println(param_value.param_id);
//            debug_serial.println(param_value.param_type);
//            debug_serial.println("------ End -------");
          }
          break;


        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
        {
          
           // mavlink_msg_servo_output_raw_decode(const mavlink_message_t* msg, mavlink_servo_output_raw_t* servo_output_raw);
          mavlink_servo_output_raw_t raw_servos;
          mavlink_msg_servo_output_raw_decode(&msg, &raw_servos);
          servo_in = raw_servos.servo1_raw;
        }
 
       default:
          break;
      }
    }
  }
}
