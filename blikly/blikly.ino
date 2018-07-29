
#include "hsv2rgb.h"
#include <avr/wdt.h>
#include <OctoWS2811.h>
#include <MedianFilter.h>

#define FILTER_SIZE 21
#define RAIL_12V_GND 6
#define RAIL_12V_POS 7
#define LEFT_FIRE_OUT 4
#define RIGHT_FIRE_OUT 5
#define MAX_FIRING_TIME 3000

// Avail interrupts on atmega
// 2, 3, 18, 19, 20, 21
#define ARMING_SIGNAL_IN 19
#define FIRE_SIGNAL_IN 20
#define LED_SIGNAL_IN 12

#define LED_PIN     13

#define LEFT_FIRE_THRESHOLD 1500
#define RIGHT_FIRE_THRESHOLD 1750
#define ARM_THRESHOLD  1250
#define SANITY_DELAY 1000

#define ARM1 36
#define ARM2 35
#define DISARM 34

#define NUM_LEDS    300
#define BRIGHTNESS  200
#define SATURATION 255
#define VALUE 180

volatile int led_in = -1;

volatile unsigned long led_time = 0;

volatile boolean new_led = false;
// volatile MedianFilter LedIn(11, 1000);
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

