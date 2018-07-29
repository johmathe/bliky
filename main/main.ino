#define THROTTLE_SIGNAL_IN 12
#define THROTTLE_SIGNAL_IN_PIN 12
#define POT_PIN 9
#define RAIL_12V 10
#define SATURATION
#define RED 0
#define SATURATION 100
#define VALUE 200

volatile int arm_in = -1;
volatile int fire_in = -1;
volatile int led_in = -1;
volatile unsigned long arm_time = 0;
volatile unsigned long fire_time = 0;
volatile unsigned long led_time = 0;
volatile boolean new_arm = false;
volatile boolean new_fire = false;
volatile boolean new_led = false;

#include <OctoWS2811.h>

const int ledsPerStrip = 300;

DMAMEM int displayMemory[ledsPerStrip * 6];
int drawingMemory[ledsPerStrip * 6];

const int config = WS2811_GRB | WS2811_800kHz;

OctoWS2811 leds(ledsPerStrip, displayMemory, drawingMemory, config);

void colorWipe(int color)
{
  for (int i = 0; i < leds.numPixels(); i++) {
    leds.setPixel(i, color);

  }
  leds.show();
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
  sat = 255 - dim_curve[255 - sat];

  int r;
  int g;
  int b;
  int base;

  if (sat == 0) { // Acromatic color (gray). Hue doesn't mind.
    colors[0] = val;
    colors[1] = val;
    colors[2] = val;
  } else  {

    base = ((255 - sat) * val) >> 8;

    switch (hue / 60) {
      case 0:
        r = val;
        g = (((val - base) * hue) / 60) + base;
        b = base;
        break;

      case 1:
        r = (((val - base) * (60 - (hue % 60))) / 60) + base;
        g = val;
        b = base;
        break;

      case 2:
        r = base;
        g = val;
        b = (((val - base) * (hue % 60)) / 60) + base;
        break;

      case 3:
        r = base;
        g = (((val - base) * (60 - (hue % 60))) / 60) + base;
        b = val;
        break;

      case 4:
        r = (((val - base) * (hue % 60)) / 60) + base;
        g = base;
        b = val;
        break;

      case 5:
        r = val;
        g = base;
        b = (((val - base) * (60 - (hue % 60))) / 60) + base;
        break;
    }

    colors[0] = r;
    colors[1] = g;
    colors[2] = b;
  }
}

void setup() {
  attachInterrupt(ARMING_SIGNAL_IN, arm_handler, CHANGE);
  attachInterrupt(FIRE_SIGNAL_IN, fire_handler, CHANGE);
  attachInterrupt(LED_SIGNAL_IN, led_handler, CHANGE);
  Serial.begin(115200);
  leds.begin();
  leds.show();
  analogReference(DEFAULT);
}

int pwm_to_hue(int pwm) {
  return int(359.0 * ((float(pwm) - 1000.0) / 1000.0));
}

int rgb_to_ledcolor(int rgb[3]) {
  return ((rgb[1] << 16) & 0xff0000) | ((rgb[0] << 8) & 0xff00) | ((rgb[2 ]) & 0xff);
}

void handleColor() {
    Serial.println(led_in);
    int hue = pwm_to_hue(led_in);
    int rgb[3];
    int val = 0.;
    hsv_to_rgb(hue, SATURATION, VALUE, rgb);
    colorWipe(rgb_to_ledcolor(rgb));
    new_signal = false;
}

void loop() {
  if (arm_time > ARM_TIME_THRESHOLD) {
    digitalWrite(RAIL_12V, HIGH);
    hsv_to_rgb(RED, SATURATION, VALUE, rgb);
    colorWipe(rgb_to_ledcolor(rgb));
  } else {
    digitalWrite(RAIL_12V, LOW);
    handleColor();
  }
  
  if (arm_time > RIGHT_FIRE_THRESHOLD) {
    digitalWrite(RIGHT_FIRE, HIGH);
  } else {
    digitalWrite(RIGHT_FIRE, LOW);
  }

  if (fire_time > LEFT_FIRE_THRESHOLD) {
    digitalWrite(LEFT_FIRE, HIGH);
  } else {
    digitalWrite(LEFT_FIRE, LOW);
  }

  }
}

void arm_handler() {
  if (digitalRead(ARMING_SIGNAL_IN) == HIGH) {
    arm_time = micros();
  }
  else {
    if (arm_time && (new_arm == false)) {
      throttle_in = (int)(micros() - arm_time);
      arm_time = 0;
      new_arm = true;
    }
  }
}

void fire_handler() {
  if (digitalRead(ARMING_SIGNAL_IN) == HIGH) {
    fire_time = micros();
  }
  else {
    if (start_time && (new_signal == false)) {
      throttle_in = (int)(micros() - fire_time);
      fire_time = 0;
      new_fire = true;
    }
  }
}

void led_handler() {
  if (digitalRead(ARMING_SIGNAL_IN) == HIGH) {
    start_time = micros();
  }
  else {
    if (start_time && (new_signal == false)) {
      led_in = (int)(micros() - led_time);
      led_time = 0;
      new_led = true;
    }
  }
}


