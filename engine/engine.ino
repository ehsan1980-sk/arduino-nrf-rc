#include <ServoTimer2.h>
#include <kalman.h>

#define BAUDRATE 9600

#define ENG_PWM_PIN 9
#define FWD_PIN 2
#define BCWD_PIN 3

#define STEARING_SERVO_PIN 10

#define JOY_ENG_PIN 0
#define JOY_STEARING_PIN 1

#define JOY_THRESHOLD 1
#define PWM_LIMIT 128  // limit for PWM of engine (0-255)

#define LEFT 1135  // 57
#define RIGTH 1565  // 103
#define ZERO 1398

struct JOY {
  uint16_t y_middle;
  uint16_t x_middle;
  uint16_t zero;
  uint16_t maximum;
};

JOY joy_eng = {.y_middle = 528, .x_middle=512, .zero = 0, .maximum = 1023};

ServoTimer2 myservo;  // create servo object to control a servo
kalman_t filtered_k = {.varVolt = 1.5, .varProcess = 0.15, .P = 1.0};

int p_stearing_val = 0;  


void setup() {
  pinMode(ENG_PWM_PIN, OUTPUT);
  pinMode(FWD_PIN, OUTPUT);
  pinMode(BCWD_PIN, OUTPUT);

  pinMode(JOY_ENG_PIN, INPUT);
  pinMode(JOY_STEARING_PIN, INPUT);

  myservo.attach(STEARING_SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  
  Serial.begin(BAUDRATE);
  
  while (!Serial) {}
}

void loop() {
  uint16_t eng_pwm = 0;      // pwm for engine
  int8_t eng_direction = 0;  // 1 - fwd, -1 - bcwrd, 0 - none

  int stearing_read = analogRead(JOY_STEARING_PIN);
  int fstr_val = (int) kalman_filter(&filtered_k, stearing_read);
  
  int stearing_servo_val = map(fstr_val, 0, 1023, LEFT, RIGTH);
  
  uint16_t acc_read = analogRead(JOY_ENG_PIN);  // acceleration control

  // for debug purpose, controller connected directly to rc board
  // forward
  if (acc_read > (joy_eng.y_middle + JOY_THRESHOLD)) {
    eng_direction = 1;
    eng_pwm = map(acc_read, joy_eng.y_middle, joy_eng.maximum, 0, PWM_LIMIT);
  }
  // backward
  else if (acc_read < (joy_eng.y_middle - JOY_THRESHOLD)) {
    eng_direction = -1;
    eng_pwm = map(acc_read, joy_eng.y_middle, 0, 0, PWM_LIMIT);
  }

  if (abs(stearing_servo_val - p_stearing_val) > 1) {  
    myservo.write(stearing_servo_val);                  // sets the servo position according to the scaled value
  }
  p_stearing_val = stearing_servo_val;

  if (eng_direction != 0) {
    digitalWrite((eng_direction > 0) ? FWD_PIN : BCWD_PIN, HIGH);
  }
  else {
    digitalWrite(FWD_PIN, LOW);
    digitalWrite(BCWD_PIN, LOW);
  }

  char t[100];
  snprintf(t, 100, "%i;%i;%i", stearing_read, fstr_val, stearing_servo_val);
  Serial.println(t);
  
  analogWrite(ENG_PWM_PIN, eng_pwm);

  // send debug info
  Serial.print(acc_read);
  Serial.print(", ");
  Serial.print(eng_pwm);
  Serial.print(", ");
  Serial.print(eng_direction);
  Serial.println();
  
  delay(50);
}
