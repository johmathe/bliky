#define debug_serial Serial

// top left 12
// top right 17
// center left 23
// center right  18
// bottom right 19
// bottom left 22
#define BTN_1 18
#define BTN_3 1
#define BTN_6 23
#define BTN_4 19
#define BTN_9 22
#define BTN_7 17

#define RELAY_0 0
#define RELAY_1 12

#define debug_serial Serial

void setup() {
  randomSeed(analogRead(14));
  pinMode(BTN_1, INPUT_PULLUP);
  pinMode(BTN_3, INPUT_PULLUP);
  pinMode(BTN_4, INPUT_PULLUP);
  pinMode(BTN_6, INPUT_PULLUP);
  pinMode(BTN_7, INPUT_PULLUP);
  pinMode(BTN_9, INPUT_PULLUP);
  pinMode(BTN_1, INPUT_PULLUP);
  pinMode(RELAY_0, OUTPUT);
  pinMode(RELAY_1, OUTPUT);

  debug_serial.begin(115200);
  initLeds();
}

double random_float(double value_min, double value_max) {
  double num = random(0, 16384);
  double coef = (value_max - value_min) / 16384;
  return num * coef + value_min / 16384;
}

bool read_button(int button) {
  return digitalRead(button) == 0;
}

const float pi = 3.141592;

void print_button_state(int button) {
  if (read_button(button)) {
    debug_serial.print("1");
  } else {
    debug_serial.print("0");
  }
}

void handle_leds() {
  print_button_state(BTN_1);
  print_button_state(BTN_3);
  print_button_state(BTN_4);
  print_button_state(BTN_6);
  print_button_state(BTN_7);
  print_button_state(BTN_9);
  debug_serial.println(" ");
  if (read_button(BTN_1)) {
    digitalWrite(RELAY_0, HIGH);
  } else {
    digitalWrite(RELAY_0, LOW);
  }


  if (read_button(BTN_3)) {
    digitalWrite(RELAY_1, HIGH);
  } else {
    digitalWrite(RELAY_1, LOW);
  }
}

volatile bool mission_set = false;
void loop() {
  int size = 20;
  for (int i = 0; i < 75; ++i) {
    stripe(i, size, 0xFF0000);
  }
  delay(1);
  digitalWrite(RELAY_0, HIGH);
  delay(3000);
  digitalWrite(RELAY_0, LOW);
}
