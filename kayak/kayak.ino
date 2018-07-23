#include "FastLED.h"
#include "hsv2rgb.h"
#include <avr/wdt.h>
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
#define LED_SIGNAL_IN 21

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
#define SATURATION 200
#define RED 0
#define SATURATION 100
#define VALUE 100

enum state {
  INIT,
  INIT_PHASE_2,
  INIT_PHASE_3,
  INIT_PHASE_4,
  ARMED
};

CRGB leds[NUM_LEDS];

volatile unsigned long hot_time = 0;
volatile unsigned long fire_start_time = 0;
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
volatile MedianFilter ArmIn(FILTER_SIZE, 1000); 
volatile MedianFilter FireIn(FILTER_SIZE, 1000);
volatile MedianFilter LedIn(11, 1000);
volatile bool fire_on = false;

void arm_handler() {
  if (digitalRead(ARMING_SIGNAL_IN) == HIGH) {
    arm_time = micros();
  }
  else {
    if (arm_time && (new_arm == false)) {
      ArmIn.in((int)(micros() - arm_time));
      arm_time = 0;
      new_arm = true;
    }
  }
}

void fire_handler() {
  if (digitalRead(FIRE_SIGNAL_IN) == HIGH) {
    fire_time = micros();
  }
  else {
    if (fire_time && (new_fire == false)) {
      FireIn.in((int)(micros() - fire_time));
      fire_time = 0;
      new_fire = true;
    }
  }
}

void led_handler() {
  if (digitalRead(LED_SIGNAL_IN) == HIGH) {
    led_time = micros();
  }
  else {
    if (led_time && (new_led == false)) {
      LedIn.in((int)(micros() - led_time));
      led_time = 0;
      new_led = true;
    }
  }
}


void setup() {
  
  delay( 3000 ); // power-up safety delay
  wdt_enable(WDTO_1S);     // enable the watchdog
  pinMode(RAIL_12V_POS, OUTPUT);
  pinMode(LEFT_FIRE_OUT, OUTPUT);
  pinMode(RIGHT_FIRE_OUT, OUTPUT);

  // Interrupts
  pinMode(ARMING_SIGNAL_IN, INPUT);
  pinMode(FIRE_SIGNAL_IN, INPUT);
  pinMode(LED_SIGNAL_IN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ARMING_SIGNAL_IN), arm_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FIRE_SIGNAL_IN), fire_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LED_SIGNAL_IN), led_handler, CHANGE);
  Serial.begin(115200);
  //pinMode(LED_PIN, OUTPUT);
  //FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
}

void wipeAll(int hue) {
  for (int i = 0; i < NUM_LEDS; ++i) {
    //leds[i] = CHSV(hue, 255, 40);
  }
  //FastLED.show();
}

int pwmToHue(int pwm) {
  return int(255 * ((float(pwm) - 1000.0) / 1000.0));
}

void handleColor() {
  wipeAll(pwmToHue(LedIn.out()));
}

void serviceOutputs() {
  Serial.print(hot_time);
  if (hot_time > MAX_FIRING_TIME) {
     allOutputsLow();
     delay(10000);
  }
  if (fire_on) {
    hot_time = millis() - fire_start_time;
  }
  if (ArmIn.out() > ARM_THRESHOLD) {
    digitalWrite(RAIL_12V_POS, HIGH);
  } else {
    handleColor();
    digitalWrite(RAIL_12V_POS, LOW);
  }

  if (ArmIn.out() > RIGHT_FIRE_THRESHOLD && hot_time < MAX_FIRING_TIME) {
    digitalWrite(RIGHT_FIRE_OUT, HIGH);
    if (!fire_on) {
      fire_start_time = millis();
    }
    fire_on = true;
  } else {
    digitalWrite(RIGHT_FIRE_OUT, LOW);
  }

  if (ArmIn.out() > ARM_THRESHOLD && FireIn.out() > LEFT_FIRE_THRESHOLD && hot_time < MAX_FIRING_TIME) {
    digitalWrite(LEFT_FIRE_OUT, HIGH);
    if (!fire_on) {
      fire_start_time = millis();
    }
    fire_on = true;
  } else {
    digitalWrite(LEFT_FIRE_OUT, LOW);
  }
  
  if (digitalRead(LEFT_FIRE_OUT) == LOW && digitalRead(RIGHT_FIRE_OUT) == LOW) {
    fire_on = false; 
    fire_time = 0;
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
  if (new_arm || new_fire || new_led) {
    new_arm = false;
    new_led = false;
    new_fire = false;
  }
  serviceOutputs();
  wdt_reset();   
  delay(50);
}

