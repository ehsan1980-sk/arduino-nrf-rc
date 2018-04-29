#define FWD_PIN 9
#define JOY_ENG_PIN 0

#define BAUDRATE 9600

struct JOY {
  uint16_t y_middle;
  uint16_t x_middle;
  uint16_t zero;
  uint16_t maximum;
};

JOY joy_eng = {.y_middle = 528, .x_middle=512, .zero = 0, .maximum = 1023};

void setup() {
  Serial.begin(BAUDRATE);
  
  while (!Serial) {}
}

void loop() {
  int trtl_read = analogRead(JOY_ENG_PIN);
  Serial.print(trtl_read);
  Serial.print(", ");

  // forward
  if (trtl_read > joy_eng.y_middle) {
     trtl_read = map(trtl_read, joy_eng.y_middle, joy_eng.maximum, 0, 255);
     analogWrite(FWD_PIN, trtl_read);
       Serial.print(trtl_read);
  }
  Serial.println("");
  delay(50);
}
