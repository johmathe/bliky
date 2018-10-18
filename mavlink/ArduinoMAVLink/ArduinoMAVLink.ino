#include "mavlink.h"

#define px4_serial Serial1
#define debug_serial Serial

// top left 12
// top right 17
// center left 23
// center right  18
// bottom right 19
// bottom left 22
#define BTW_SWIPE 12
#define BTN_GO_RANDOM 19
#define BTN_GO_HOME 22
#define BTN_COLOR 23
#define BTN_GNE 18
#define BTN_PLASMA 17

enum State {
  INITIAL_STATE,
  HOME_MODE,
  RANDOM_MODE
};

const float home_lat = 40.7908011;
const float home_lon = -119.2141898;

State state = INITIAL_STATE;

volatile int servo_in = 1500;

unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 10;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int heartbeat_count = num_hbs;
const float MIN_BAT_VOLTAGE = 14500.5;

volatile float battery_voltage = 15000;


#define debug_serial Serial

void setup() {
  randomSeed(analogRead(14));
  pinMode(BTW_SWIPE, INPUT_PULLUP);
  pinMode(BTN_GO_RANDOM, INPUT_PULLUP);
  pinMode(BTN_GO_HOME, INPUT_PULLUP);
  pinMode(BTN_COLOR, INPUT_PULLUP);
  pinMode(BTN_GNE, INPUT_PULLUP);
  pinMode(BTN_PLASMA, INPUT_PULLUP);
  debug_serial.begin(115200);
  debug_serial.println("MAVLink starting.");
  px4_serial.begin(38400);
  initLeds();
}

double random_float(double value_min, double value_max) {
  double num = random(0, 16384);
  double coef = (value_max - value_min) / 16384;
  return num * coef + value_min / 16384;
}

const float pi = 3.141592;

void get_random_coordinate(double* lat, double* lon) {
  const float brc_center_lat = 40.78684;
  const float brc_center_lon = -119.20610;
  const double r_earth = 6371393;
  double r = random_float(0.0, 2200.0);
  double offset = pi / 9.3;
  double angle = random_float(offset, 2 * pi - (2. / 3.) * pi + offset );
  double dx = 1.0 * r * cos(angle);
  double dy = 0.4 * r * sin(angle);
  *lat = brc_center_lat  + (dx / r_earth) * (180.0 / pi);
  *lon = brc_center_lon + (dy / r_earth) * (180.0 / pi) / cos(brc_center_lat * pi / 180.0);
}

void handle_leds() {
  if (digitalRead(BTN_PLASMA) == 0) {
    debug_serial.println("plasma");
    plasmaLoop();
    return;
  }
  else if (digitalRead(BTW_SWIPE) == 0) {
    debug_serial.println("swipe");
    swipeLoop(0xAA4400);
    return;
  }
  else {
    debug_serial.println("swipe color");
    swipeColor(0);
    return;
  }
}

volatile bool mission_set = false;
void loop() {
  
  if (false) { // 21  ` battery_voltage < MIN_BAT_VOLTAGE) {
    debug_serial.println("BATTERY IS DEAD");
    wipeAll(0x00000);
    delay(1000);
  } else {
    if (digitalRead(BTN_GO_HOME) == 0) {
      if (state != HOME_MODE) {
        debug_serial.println("HOME MODE ACTIVATED");
        mav_set_mission(home_lat, home_lon);
        state = HOME_MODE;
      }
    } else if (digitalRead(BTN_GO_RANDOM) == 0) {
      if (state != RANDOM_MODE) {
        debug_serial.println("RANDOM MODE ACTIVATED");
        double random_lat = 0.0;
        double random_lon = 0.0;
        get_random_coordinate(&random_lat, &random_lon);
        debug_serial.println(random_lat);
        debug_serial.println(random_lon);
        mav_set_mission(random_lat, random_lon);
        state = RANDOM_MODE;
      } 
    } else {
      state = INITIAL_STATE;
      debug_serial.println("handling leds");
      handle_leds();
      return;
    }
  }
  
  wipePwm(servo_in);

  // MAVLink
  int sysid = 1;
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

    if (heartbeat_count >= num_hbs) {
      debug_serial.println("Streams requested!");
      mav_start_streams();
      heartbeat_count = 0;
    }

    if (heartbeat_count == (num_hbs - 2) && !mission_set) {
      debug_serial.println("Mission setting:");
      mav_set_mission(home_lat, home_lon);
      mission_set = true;
    }
  }
  comm_receive();
}

void mav_start_streams()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const int  max_streams = 2;
  const uint8_t MAVStreams[max_streams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_RC_CHANNELS};
  const uint16_t MAVRates[max_streams] = {0x01, 0x08};

  for (int i = 0; i < max_streams; ++i) {
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    px4_serial.write(buf, len);
  }
}

void mav_set_mission(double lat, double lon) {
  debug_serial.print("setting mission with lat ");
  debug_serial.print(lat);
  debug_serial.print(" lon ");
  debug_serial.println(lon);
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_mission_clear_all_pack(2, 200, &msg, 1, 1);
  uint16_t  len = mavlink_msg_to_send_buffer(buf, &msg);
  px4_serial.write(buf, len);
  wait_for_mission_ack();
  mavlink_msg_mission_count_pack(2, 200, &msg, 1, 1, 4);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  px4_serial.write(buf, len);
  wait_for_mission_ack();
  mavlink_msg_mission_item_pack(2, 200, &msg, 1, 1, 0, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 2, 0, 0, lat, lon, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  px4_serial.write(buf, len);
  wait_for_mission_ack();
  mavlink_msg_mission_item_pack(2, 200, &msg, 1, 1, 1, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 2, 0, 0, lat, lon, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  px4_serial.write(buf, len);
  wait_for_mission_ack();
  mavlink_msg_mission_item_pack(2, 200, &msg, 1, 1, 2, MAV_FRAME_GLOBAL, MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  px4_serial.write(buf, len);
  wait_for_mission_ack();
  mavlink_msg_mission_item_pack(2, 200, &msg, 1, 1, 3, MAV_FRAME_GLOBAL, MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  px4_serial.write(buf, len);
  wait_for_mission_ack();
  mavlink_msg_mission_ack_pack(2, 200, &msg, 1, 1, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  px4_serial.write(buf, len);
}

void wait_for_mission_ack() {
  mavlink_message_t msg;
  mavlink_status_t status;
  while (true) {
    while (px4_serial.available() > 0) {
      uint8_t c = px4_serial.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        switch (msg.msgid) {
          case MAVLINK_MSG_ID_MISSION_ACK: 
            {
              debug_serial.println("MISSION ACK");
              return;
            }
            break;
          case MAVLINK_MSG_ID_MISSION_REQUEST:
            {
              debug_serial.println("MISSION REQUEST");
              return;
            }
            break;
          default:
            {
            }
            break;
        }
      }
    }
  }
}

void comm_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;
  
  while (px4_serial.available() > 0) {
    uint8_t c = px4_serial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
          {
          }
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:
          {
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
            battery_voltage = sys_status.voltage_battery;
          }
          break;

        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
          {
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
