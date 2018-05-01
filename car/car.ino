#include "nRF24L01.h"
#include "RF24.h"

#include <ServoTimer2.h>

#define BAUDRATE 9600

#define NRF_CE 9
#define NRF_CS 10
#define NRF_ADDR 0
#define NRF_CHAN 0x60
#define NRF_PAYLOAD_SIZE 32

#define ENG_PWM_PIN 2
#define FWD_PIN 3
#define BCWD_PIN 4
#define STEARING_SERVO_PIN 5

#define LEFT 1135  // 57
#define RIGTH 1565  // 103
#define ZERO 1398

RF24 radio(NRF_CE, NRF_CS);
uint8_t address[][6] = {"1Node","2Node","3Node","4Node","5Node","6Node"};
uint8_t pipeNo;

uint8_t payload[NRF_PAYLOAD_SIZE];

ServoTimer2 myservo;  // create servo object to control a servo

int8_t eng_direction;  // 1 - fwd, 2 - bcwrd, 0 - none
uint8_t eng_pwm;      // pwm for engine
uint16_t p_stearing_val = 0;
uint16_t stearing_servo_val;


void setup() {
  Serial.begin(BAUDRATE);
  while (!Serial) {}

  // radio setup
  radio.begin();
  radio.setAutoAck(1);
  radio.setRetries(0, 15);  // delay, count of retry

  radio.setPayloadSize(NRF_PAYLOAD_SIZE);  // package size

  radio.openReadingPipe(1, address[NRF_ADDR]);  // addresss
  radio.setChannel(NRF_CHAN);  // chanel

  radio.setPALevel(RF24_PA_MAX);  // RF24_PA_MIN | RF24_PA_LOW | RF24_PA_HIGH | RF24_PA_MAX
  radio.setDataRate(RF24_1MBPS);  // RF24_2MBPS | RF24_1MBPS, | RF24_250KBPS

  radio.powerUp();
  radio.startListening();  // listen

  // pins setup
  pinMode(ENG_PWM_PIN, OUTPUT);
  pinMode(FWD_PIN, OUTPUT);
  pinMode(BCWD_PIN, OUTPUT);

  // stearing servo
  myservo.attach(STEARING_SERVO_PIN);  // attaches the servo on pin 9 to the servo object
}

void loop() { 
  while (radio.available(&pipeNo)){
    radio.read(&payload, sizeof(payload));

    uint8_t command = payload[0];

    switch (command) {
      // move command
      case 1:
        stearing_servo_val = map(payload[1], 0, 255, LEFT, RIGTH);
        eng_direction = payload[2];  // fixme
        eng_pwm = payload[3];

        break;
    }

    // strearing
    if (abs(stearing_servo_val - p_stearing_val) > 1) {  
       myservo.write(stearing_servo_val);                  // sets the servo position according to the scaled value
    }
    p_stearing_val = stearing_servo_val;

    // engine
    switch (eng_direction) {
      // none
      case 0:
        digitalWrite(FWD_PIN, LOW);
        digitalWrite(BCWD_PIN, LOW);
        break;

      // forward
      case 1:
        digitalWrite(FWD_PIN, HIGH);
        digitalWrite(BCWD_PIN, LOW);
        break;

      // backward
      case 2:
        digitalWrite(BCWD_PIN, HIGH);
        digitalWrite(FWD_PIN, LOW);
        break;
    }

    analogWrite(ENG_PWM_PIN, eng_pwm);  // send PWM signal to engine

    // send debug info
    char t[100];
    snprintf(t, 100, "msg: %d_%d_%d_%d", command, stearing_servo_val, eng_direction, eng_pwm);
    // Serial.println(t);
    snprintf(t, 100, "eng_pwm: %i, eng_dir: %i, stear: %i", eng_pwm, eng_direction, stearing_servo_val);
    Serial.println(t);
      
  }  
}
