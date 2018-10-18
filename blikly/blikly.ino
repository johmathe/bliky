
#include "hsv2rgb.h"
#include <avr/wdt.h>
#include <OctoWS2811.h>
#include <MedianFilter.h>
#include "mavlink.h"

#define LED_PIN     13

#define SANITY_DELAY 1000

// LED Stuff.
#define NUM_LEDS    300
#define BRIGHTNESS  200
#define SATURATION 255
#define VALUE 180

volatile int led_in = -1;
volatile unsigned long led_time = 0;
volatile boolean new_led = false;
volatile bool fire_on = false;
const int ledsPerStrip = 300;
DMAMEM int displayMemory[ledsPerStrip * 6];
int drawingMemory[ledsPerStrip * 6];
const int config = WS2811_GRB | WS2811_800kHz;
OctoWS2811 leds(ledsPerStrip, displayMemory, drawingMemory, config);

void led_handler() {
  if (digitalRead(LED_SIGNAL_IN) == HIGH) {
    led_time = micros();
  }
  else {
    if (led_time && (new_led == false)) {
      //LedIn.in((int)(micros() - led_time));
      led_in = (int)(micros() - led_time);
      led_time = 0;
      new_led = true;
    }
  }
}


void setup() {
  wdt_enable(WDTO_1S);     // enable the watchdog
  // Interrupts
  pinMode(LED_SIGNAL_IN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LED_SIGNAL_IN), led_handler, CHANGE);
  Serial.begin(115200);
  // pixhawk serial link
  Serial2.begin(57600);
  leds.begin();
  leds.show();
}

void wipeAll(int hue) {
  for (int i = 0; i < NUM_LEDS; ++i) {
    leds.setPixel(i, hue);
  }
  leds.show();
}

int pwmToHue(int pwm) {
  return int(255 * ((float(pwm) - 1000.0) / 1000.0));
}



const byte dim_curve[] = {
    0,   1,   1,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   3,
    3,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   4,   4,   4,   4,
    4,   4,   4,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   6,   6,   6,
    6,   6,   6,   6,   6,   7,   7,   7,   7,   7,   7,   7,   8,   8,   8,   8,
    8,   8,   9,   9,   9,   9,   9,   9,   10,  10,  10,  10,  10,  11,  11,  11,
    11,  11,  12,  12,  12,  12,  12,  13,  13,  13,  13,  14,  14,  14,  14,  15,
    15,  15,  16,  16,  16,  16,  17,  17,  17,  18,  18,  18,  19,  19,  19,  20,
    20,  20,  21,  21,  22,  22,  22,  23,  23,  24,  24,  25,  25,  25,  26,  26,
    27,  27,  28,  28,  29,  29,  30,  30,  31,  32,  32,  33,  33,  34,  35,  35,
    36,  36,  37,  38,  38,  39,  40,  40,  41,  42,  43,  43,  44,  45,  46,  47,
    48,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,
    63,  64,  65,  66,  68,  69,  70,  71,  73,  74,  75,  76,  78,  79,  81,  82,
    83,  85,  86,  88,  90,  91,  93,  94,  96,  98,  99,  101, 103, 105, 107, 109,
    110, 112, 114, 116, 118, 121, 123, 125, 127, 129, 132, 134, 136, 139, 141, 144,
    146, 149, 151, 154, 157, 159, 162, 165, 168, 171, 174, 177, 180, 183, 186, 190,
    193, 196, 200, 203, 207, 211, 214, 218, 222, 226, 230, 234, 238, 242, 248, 255,
};

void hsv_to_rgb(int hue, int sat, int val, int colors[3]) { 
  val = dim_curve[val];
  sat = 255-dim_curve[255-sat];
 
  int r;
  int g;
  int b;
  int base;
 
  if (sat == 0) { // Acromatic color (gray). Hue doesn't mind.
    colors[0]=val;
    colors[1]=val;
    colors[2]=val;  
  } else  { 
 
    base = ((255 - sat) * val)>>8;
 
    switch(hue/60) {
    case 0:
        r = val;
        g = (((val-base)*hue)/60)+base;
        b = base;
    break;
 
    case 1:
        r = (((val-base)*(60-(hue%60)))/60)+base;
        g = val;
        b = base;
    break;
 
    case 2:
        r = base;
        g = val;
        b = (((val-base)*(hue%60))/60)+base;
    break;
 
    case 3:
        r = base;
        g = (((val-base)*(60-(hue%60)))/60)+base;
        b = val;
    break;
 
    case 4:
        r = (((val-base)*(hue%60))/60)+base;
        g = base;
        b = val;
    break;
 
    case 5:
        r = val;
        g = base;
        b = (((val-base)*(60-(hue%60)))/60)+base;
    break;
    }
 
    colors[0]=r;
    colors[1]=g;
    colors[2]=b; 
  }   
}

void loop() {
  // Some basic state machine to arm the system.
if(new_led) {
   Serial.println(led_in);  
   int rgb[3];
   int hue = pwmToHue(led_in);
   hsv_to_rgb(hue, SATURATION, VALUE, rgb);
   Serial.print("hue: ");
   Serial.print(hue);
   Serial.print(" r: ");
   Serial.print(rgb[0]);
   Serial.print(" g: ");
   Serial.print(rgb[1]);
   Serial.print(" b: ");
   Serial.print(rgb[2]);
   Serial.print("\n ");
   new_led = false;
   int total_rgb = (rgb[1] << 16) | (rgb[0] << 8) | (rgb[2]);
   Serial.print(total_rgb);
   wipeAll(total_rgb);
 }
  wdt_reset();
}

void loop() {

 command_heartbeat();
 command_location(52.464217,-1.280222, 200);
}

void command_location(int32_t lat, int32_t lon, int32_t alt) {

  //TARGET DRONE
  uint8_t _target_system = 1; // Target drone id
  uint8_t _target_component = 100; // component id set to 100 for FLIR cameras

  uint16_t seq = 0; // Sequence is always set to 0
  uint8_t frame = MAV_FRAME_GLOBAL; // Set target frame to global default
  uint16_t command = GLOBAL_POSITION_INT; // Specific command for Duo
  uint8_t current = 2; // Guided mode waypoint
  uint8_t autocontinue = 0; // Always 0
  uint32_t upTime = 0; //time since system boot, ignored by camera
  int16_t velx = 1; //x speed
  int16_t vely = 1; //y speed
  int16_t velz = 1; //z speed
  uint16_t heading = 180; //heading


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_mission_item_pack(1,1, &msg, _target_system, _target_component, seq, frame, command, current, autocontinue, upTime, lat, lon, alt, alt, velx, vely, velz, heading);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  delay(1000);
  Serial.write(buf, len);
}

/************************************************************
* @brief Sends a heartbeat message every second.
* @param NONE
* @return void
*************************************************************/

void command_heartbeat() {

  //< ID 1 for this system
  int sysid = 1;                   
  //< The component sending the message.
  int compid = MAV_COMP_ID_MISSIONPLANNER;    
  
  // Define the system type, in this case ground control station
  uint8_t system_type = MAV_TYPE_GCS;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  
  uint8_t system_mode = 0; 
  uint32_t custom_mode = 0;                
  uint8_t system_state = 0;
  
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Pack the message
  mavlink_msg_heartbeat_pack(sysid,compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
  
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message 
  delay(1000);
  Serial.write(buf, len);
}

/************************************************************
* @brief Sends a waypoint command
* @param NONE
* @return void
*************************************************************/

void command_waypoint() {

  //TARGET DRONE
  uint8_t _target_system = 1; // Target drone id
  uint8_t _target_component = 0; // Target component, 0 = all

  uint16_t seq = 0; // Sequence is always set to 0
  uint8_t frame = MAV_FRAME_GLOBAL; // Set target frame to global default
  uint16_t command = MAV_CMD_NAV_WAYPOINT; // Specific command for PX4 
  uint8_t current = 2; // Guided mode waypoint
  uint8_t autocontinue = 0; // Always 0
  float param1 = 0; // Loiter time
  float param2 = 1; // Acceptable range from target - radius in meters
  float param3 = 0; // Pass through waypoint
  float param4 = 0; // Desired yaw angle
  float x = 52.464217; // Latitude - degrees
  float y = -1.280222; // Longitude - degrees
  float z = 200; // Altitude - meters

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_mission_item_pack(1,1, &msg, _target_system, _target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  delay(1000);
  Serial.write(buf, len);
}

