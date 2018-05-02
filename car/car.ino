#include "nRF24L01.h"
#include "RF24.h"

#include <ServoTimer2.h>

#define BAUDRATE 9600

#define NRF_CE 9
#define NRF_CS 10
#define NRF_ADDR 0
#define NRF_CHAN 0x60
#define NRF_PAYLOAD_SIZE 32

#define ENG_PWM_PIN 6
#define FWD_PIN 3
#define BCWD_PIN 4
#define STEARING_SERVO_PIN 5

#define ZERO 1375
#define LEFT ZERO - 320
#define RIGTH ZERO + 320


RF24 radio(NRF_CE, NRF_CS);
uint8_t address[][6] = {"1Node","2Node","3Node","4Node","5Node","6Node"};
uint8_t pipeNo;

uint8_t payload[NRF_PAYLOAD_SIZE];

ServoTimer2 stearing_servo;  // create servo object to control stearing

int8_t eng_direction;  // 1 - fwd, 2 - bcwrd, 0 - none
uint8_t eng_pwm;      // pwm for engine
uint16_t p_stearing_val = 0;
uint16_t stearing_servo_val;  // stearing servo timing

uint8_t delay_ms = 20;  // delay between commands from controller
long int recieve_package_time = millis();  // last command recieved time

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
  stearing_servo.attach(STEARING_SERVO_PIN);
  stearing_servo.write(ZERO); 
}

void loop() {  
  while (radio.available(&pipeNo)){
    recieve_package_time = millis();
    radio.read(&payload, sizeof(payload));

    uint8_t command = payload[0];
    delay_ms = payload[7];

    switch (command) {
      // move command
      case 1:
        stearing_servo_val = map(payload[1], 0, 255, LEFT, RIGTH);  // map 0 - 255 to servo timing
        eng_direction = payload[2];  // fixme
        eng_pwm = payload[3];

        break;
    }

    // strearing
    if (abs(stearing_servo_val - p_stearing_val) > 1) {  
       stearing_servo.write(stearing_servo_val);  // sets the servo position according to the scaled value
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
    snprintf(t, 100, "eng_pwm: %i, eng_dir: %i, stear: %i", eng_pwm, eng_direction, stearing_servo_val);
    Serial.println(t);
  }
  
  // in case of lose connection
  if ((millis() - recieve_package_time) > delay_ms) {
    Serial.println("Reset state");

    digitalWrite(FWD_PIN, LOW);
    digitalWrite(BCWD_PIN, LOW);
    stearing_servo.write(ZERO);
    
    delay(delay_ms);  // wait some time...
  }
}
