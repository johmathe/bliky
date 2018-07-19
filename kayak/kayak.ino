#include "FastLED.h"
#include "hsv2rgb.h"

#define RAIL_12V_GND 4
#define RAIL_12V_POS 5
#define LEFT_FIRE_OUT 6
#define RIGHT_FIRE_OUT 7

#define POT_PIN 9
#define SATURATION
#define RED 0
#define SATURATION 100
#define VALUE 200

#define ARMING_SIGNAL_IN 2
#define FIRE_SIGNAL_IN 16
#define LED_SIGNAL_IN 17

#define LED_PIN     48
#define LED_PIN     1
#define COLOR_ORDER GRB
#define CHIPSET     WS2811
#define NUM_LEDS    300
#define BRIGHTNESS  200
#define FRAMES_PER_SECOND 60

#define LEFT_FIRE_THRESHOLD 1500
#define RIGHT_FIRE_THRESHOLD 1600
#define ARM_THRESHOLD  1300
#define SANITY_DELAY 1000

#define ARM1 20
#define ARM2 21
#define DISARM 19

#define LED_PIN 10


enum state {
  INIT,
  INIT_PHASE_2,
  INIT_PHASE_3,
  INIT_PHASE_4,
  ARMED
};

CRGB leds[NUM_LEDS];

volatile int arm_in = -1;
volatile int fire_in = -1;
volatile int led_in = -1;
volatile unsigned long arm_time = 0;
volatile unsigned long fire_time = 0;
volatile unsigned long led_time = 0;
volatile boolean new_arm = false;
volatile boolean new_fire = false;
volatile boolean new_led = false;
volatile int state = INIT;

void arm_handler() {
  if (digitalRead(ARMING_SIGNAL_IN) == HIGH) {
    arm_time = micros();
  }
  else {
    if (arm_time && (new_arm == false)) {
      arm_in = (int)(micros() - arm_time);
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
    if (fire_time && (new_fire == false)) {
      fire_in = (int)(micros() - fire_time);
      fire_time = 0;
      new_fire = true;
    }
  }
}

void led_handler() {
  if (digitalRead(ARMING_SIGNAL_IN) == HIGH) {
    led_time = micros();
  }
  else {
    if (led_time && (new_led == false)) {
      led_in = (int)(micros() - led_time);
      led_time = 0;
      new_led = true;
    }
  }
}


void setup() {
  pinMode(RAIL_12V_GND, OUTPUT);
  pinMode(RAIL_12V_POS, OUTPUT);
  pinMode(LEFT_FIRE_OUT, OUTPUT);
  pinMode(RIGHT_FIRE_OUT, OUTPUT);
  allOutputsLow();
  
  attachInterrupt(digitalPinToInterrupt(ARMING_SIGNAL_IN), arm_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FIRE_SIGNAL_IN), fire_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LED_SIGNAL_IN), led_handler, CHANGE);
  
  Serial.begin(115200);
  analogReference(DEFAULT);
  delay(SANITY_DELAY);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
}

void wipeAll(int hue) {
  for (int i = 0; i < NUM_LEDS; ++i) {
    leds[i] = CHSV(hue, SATURATION, VALUE);
  }
  FastLED.show();
}

int pwmToHue(int pwm) {
  return int(255 * ((float(pwm) - 1000.0) / 1000.0));
}

void handleColor() {
    wipeAll(pwmToHue(led_in));
}

void serviceOutputs() {
    if (arm_in > ARM_THRESHOLD) {
    digitalWrite(RAIL_12V_GND, HIGH);
    digitalWrite(RAIL_12V_POS, HIGH);
    wipeAll(HUE_RED);
  } else {
    digitalWrite(RAIL_12V_GND, LOW);
    digitalWrite(RAIL_12V_POS, LOW);
    handleColor();
  }
  
  if (arm_in > RIGHT_FIRE_THRESHOLD) {
    digitalWrite(RIGHT_FIRE_OUT, HIGH);
  } else {
    digitalWrite(RIGHT_FIRE_OUT, LOW);
  }

  if (fire_time > LEFT_FIRE_THRESHOLD) {
    digitalWrite(LEFT_FIRE_OUT, HIGH);
  } else {
    digitalWrite(LEFT_FIRE_OUT, LOW);
  }
}

void allOutputsLow() {
  digitalWrite(LEFT_FIRE_OUT, LOW);
  digitalWrite(RIGHT_FIRE_OUT, LOW);
  digitalWrite(RAIL_12V_GND, LOW);
  digitalWrite(RAIL_12V_POS, LOW); 
}

void loop() {
  // Some basic state machine to arm the system.
  if (state == INIT && digitalRead(ARM1) == LOW) {
    state = INIT_PHASE_2;
  }
  if (state == INIT_PHASE_2 && digitalRead(ARM2) == LOW) {
    state = INIT_PHASE_3;
  }
  if (state == INIT_PHASE_3 && digitalRead(ARM1) == HIGH) {
    state = INIT_PHASE_4;
  }
  if (state == INIT_PHASE_4 && digitalRead(ARM1) == LOW) {
    state = ARMED;
  }
  if (digitalRead(DISARM) == LOW) {
    state = INIT;
  }
  
  if (state == ARMED) {
    serviceOutputs();
  } else {
    allOutputsLow();
  }
}

