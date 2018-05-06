#include "nRF24L01.h"
#include "RF24.h"

#include <ServoTimer2.h>

#define BAUDRATE 115200

// NRF24 pins
#define NRF_CE 9
#define NRF_CS 10
#define NRF_ADDR 0
#define NRF_CHAN 0x60
#define NRF_PAYLOAD_SIZE 32

// L298 and steering servo pins
#define ENG_PWM_PIN 6
#define FWD_PIN 3
#define BCWD_PIN 4
#define STEERING_SERVO_PIN 5

// Steerings params
#define ZERO 1325
#define RIGTH ZERO - 350
#define LEFT ZERO + 350

// Engine mode control params
#define FREE 0
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3

// Commands
#define CMD_NONE 0
#define CMD_MOVE 1
#define CMD_BRAKE 2


RF24 radio(NRF_CE, NRF_CS);
uint8_t address[][6] = {"1Node","2Node","3Node","4Node","5Node","6Node"};
uint8_t pipeNo;

uint8_t payload[NRF_PAYLOAD_SIZE];

ServoTimer2 steering_servo;  // create servo object to control stearing

int8_t eng_direction;  // 1 - fwd, 2 - bcwrd, 0 - none
uint8_t eng_pwm;      // pwm for engine
uint16_t steering_servo_val;  // stearing servo timing

uint8_t delay_ms = 20;  // delay between commands from controller
long int recieve_package_time = millis();  // last command recieved time


/* 
 *  Set steering servo position.
 *  timing - between LEFT and RIGTH
*/
void set_steering(uint16_t timing) {
    static uint16_t p_timing = 0;

    if (abs(timing - p_timing) > 1) {  
      steering_servo.write(timing);  // sets the servo position according to the scaled value
    }
    p_timing = timing;
}

/*
 * Set engine state for L298. Control both direction and PWM.
 * In case FREE and BRAKE direction commands pwm value ignores;
 * 
 * Direction - one of FREE | FORWARD | BACKWARD | BRAKE
 * 
 * FREE - disconnect engine from circuit
 * BRAKE - short engine terminals
 */
void set_engine(uint8_t eng_direction, uint8_t eng_pwm) {
   switch (eng_direction) {
      case FREE:
        // set both INs to same (low or high) AND enable to low;
        digitalWrite(FWD_PIN, LOW);
        digitalWrite(BCWD_PIN, LOW);
        eng_pwm = 0;
        break;

      case FORWARD:
        digitalWrite(FWD_PIN, HIGH);
        digitalWrite(BCWD_PIN, LOW);
        break;

      case BACKWARD:
        digitalWrite(BCWD_PIN, HIGH);
        digitalWrite(FWD_PIN, LOW);
        break;

      case BRAKE:
        // set both INs to same (low or high) AND enable to 1;
        digitalWrite(BCWD_PIN, LOW);
        digitalWrite(FWD_PIN, LOW);
        eng_pwm = 255;
        break;
    }

    analogWrite(ENG_PWM_PIN, eng_pwm);  // send PWM signal to engine
};

/*
 * Simple greeting to determine car status.
 */
void hello() {
  set_steering(ZERO);
  delay(100);
  set_steering(RIGTH);
  delay(100);
  set_steering(ZERO);
};

void setup() {
  Serial.begin(BAUDRATE);
  while (!Serial) {}

  // radio setup
  radio.begin();
  radio.setAutoAck(1);
  radio.setRetries(0, 15);  // delay, count of retry

  radio.setPayloadSize(NRF_PAYLOAD_SIZE);  // package size

  radio.openReadingPipe(1, address[NRF_ADDR]);  // address
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
  steering_servo.attach(STEERING_SERVO_PIN);

  hello();  
}

void loop() {  
  while (radio.available(&pipeNo)){
    recieve_package_time = millis();
    radio.read(&payload, sizeof(payload));

    uint8_t command = payload[0];
    steering_servo_val = map(payload[1], 0, 255, LEFT, RIGTH);  // map 0 - 255 to servo timing
    delay_ms = payload[7];

    switch (command) {
      case CMD_NONE:
        set_engine(FREE, 0);
        break;

      case CMD_MOVE:
        eng_direction = payload[2];
        eng_pwm = payload[3];
    
        // engine
        set_engine(eng_direction, eng_pwm);

        break;

      case CMD_BRAKE:
        set_engine(BRAKE, 255);

        break;
    }

    // streering
    set_steering(steering_servo_val);

    // send debug info
    char t[100];
    snprintf(t, 100, "eng_pwm: %i, eng_dir: %i, stear: %i", eng_pwm, eng_direction, steering_servo_val);
    Serial.println(t);
  }
  
  // in case of lose connection
  if ((millis() - recieve_package_time) > delay_ms * 5) {
    Serial.println("Reset state");

    set_engine(BRAKE, 0);  // brake by engine
    
    set_steering(ZERO);  // set steering to center point
    
    delay(delay_ms);  // wait some time...
  }
}
