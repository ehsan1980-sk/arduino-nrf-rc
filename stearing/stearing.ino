#include <ServoTimer2.h>
#include <kalman.h>

#define BADRATE 9600

#define POT_PIN 0
#define SERVO_PIN 9

#define LEFT 1135  // 57
#define RIGTH 1565  // 103
#define ZERO 1398


ServoTimer2 myservo;  // create servo object to control a servo
kalman_t filtered_k = {.varVolt = 1.5, .varProcess = 0.15, .P = 1.0};

static long int _time = 0;
static int p_servo_val = 0;


void setup() {
  Serial.begin(BADRATE);
  
  while (!Serial) {}
  
  myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  
  
  int val = analogRead(POT_PIN);            // reads the value of the potentiometer (value between 0 and 1023)

  int fval = (int) kalman_filter(&filtered_k, val);
  
  int servo_val = map(fval, 0, 1023, LEFT, RIGTH);     // scale it to use it with the servo (value between 0 and 180)

  char t[100];
  snprintf(t, 100, "%li;%i;%i;%i", _time, val, fval, servo_val);
  Serial.println(t);
  
  if (abs(servo_val - p_servo_val) > 1) {  
    myservo.write(servo_val);                  // sets the servo position according to the scaled value
  }
  p_servo_val = servo_val;

  _time += 10;
  delay(10);
}

