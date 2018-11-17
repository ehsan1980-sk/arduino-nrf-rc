#include "nRF24L01.h"
#include "RF24.h"
#include <ServoTimer2.h>


#define BAUDRATE 115200

// Radio Commands
#define CMD_NONE 1
#define CMD_ACTIVE 2

// Engine mode control params
#define FREE 0
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3


// NRF24 pins
#define NRF_CE 3
#define NRF_CS 10

// NRF24 settings
#define NRF_ADDR 0
#define NRF_CHAN 0x60
#define NRF_PAYLOAD_SIZE 32

// L298 or MX1508 driver pins
#define ENG_PWM_PIN -1  // -1 is MX1508 driver
#define FWD_PIN 9
#define BCWD_PIN 6

// Steering servo pin
#define STEERING_SERVO_PIN 5

// Steerings params
#define ZERO 1390
#define RIGTH ZERO - 280
#define LEFT ZERO + 280

// Engine PWM controls
#define PWM_LOW_LIMIT 50  // low limit of PWM
#define PWM_LIMIT 255     // limit for PWM of engine (0-255)


RF24 radio(NRF_CE, NRF_CS);
uint8_t address[][6] = {"1Node","2Node","3Node","4Node","5Node","6Node"};
uint8_t pipeNo;

struct payload_t {
  uint8_t command;
  uint16_t stick1_x;
  uint16_t stick1_y;
  bool stick1_p;
  uint16_t stick2_x;
  uint16_t stick2_y;
  bool stick2_p;
  uint8_t delay_ms;
} p;

long int recieve_package_time = millis();  // last command recieved time

ServoTimer2 steering_servo;  // create servo object to control stearing


/* 
 *  Set steering servo position.
 *  timing - between LEFT and RIGTH
*/
void set_steering(uint16_t timing) {
    static uint16_t p_timing = 0;
    
    if (abs(timing - p_timing) > 0) {  
      steering_servo.write(timing);  // sets the servo position according to the scaled value
    }

    p_timing = timing;
}


void set_engine(uint8_t eng_direction, uint8_t eng_pwm) {
    if (ENG_PWM_PIN == -1) {
        set_MX1508_engine(eng_direction, eng_pwm);
    } else {
        set_L298_engine(eng_direction, eng_pwm);
    }
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
void set_L298_engine(uint8_t eng_direction, uint8_t eng_pwm) {
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


void set_MX1508_engine(uint8_t eng_direction, uint8_t eng_pwm) {
    switch (eng_direction) {
      case FREE:
        digitalWrite(FWD_PIN, LOW);
        digitalWrite(BCWD_PIN, LOW);
        break;

      case FORWARD:
        analogWrite(FWD_PIN, eng_pwm);
        digitalWrite(BCWD_PIN, LOW);
        break;

      case BACKWARD:
        digitalWrite(FWD_PIN, LOW);
        analogWrite(BCWD_PIN, eng_pwm);
        break;

      case BRAKE:
        digitalWrite(FWD_PIN, HIGH);
        digitalWrite(BCWD_PIN, HIGH);
        break;
    }
}


void debug(uint8_t eng_direction, uint8_t eng_pwm, uint16_t steering_servo_value) {
  char t[100];
  snprintf(t, 100, "car. eng_direction: %i, eng_pwm: %i, steering: %i", 
                    eng_direction, eng_pwm, steering_servo_value);
  Serial.println(t);
}


/*
 * Simple greeting to determine car status.
 */
void hello() {
  set_steering(LEFT);
  delay(100);
  set_steering(RIGTH);
  delay(100);
  set_steering(ZERO);
};


void setup() {
  Serial.begin(BAUDRATE);
  while (!Serial) {}

  Serial.println("setup start");

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
  if (ENG_PWM_PIN != -1) pinMode(ENG_PWM_PIN, OUTPUT);

  pinMode(FWD_PIN, OUTPUT);
  pinMode(BCWD_PIN, OUTPUT);

  // steering servo
  steering_servo.attach(STEERING_SERVO_PIN);

  hello();  

  Serial.println("setup end");
}


void loop() {
  while (radio.available(&pipeNo)){
    recieve_package_time = millis();
    radio.read(&p, sizeof(payload_t));

    // steering
    uint16_t steering_servo_value = map(p.stick2_x, 0, 1023, LEFT, RIGTH);  // map 0 - 1023 to servo timing
    set_steering(steering_servo_value);

    switch (p.command) {
      case CMD_NONE:
        set_engine(FREE, 0);
        break;

      case CMD_ACTIVE:
        int8_t eng_direction = FREE;
        uint8_t eng_pwm = 0;

        if (p.stick1_p) {
          eng_direction = BRAKE; 
        } else {
          if (p.stick1_y > 512) {
            eng_direction = FORWARD;
            eng_pwm = map(p.stick1_y, 512, 1023, PWM_LOW_LIMIT, PWM_LIMIT);
          }
          else if (p.stick1_y < 512) {
            eng_direction = BACKWARD;
            eng_pwm = map(p.stick1_y, 512, 0, PWM_LOW_LIMIT, PWM_LIMIT);
          }  
        }

        debug(eng_direction, eng_pwm, steering_servo_value);
        set_engine(eng_direction, eng_pwm);

        break;
    }
  }

  
  // in case of lose connection
  if ((millis() - recieve_package_time) > p.delay_ms * 5) {
    Serial.println("Reset state");

    set_engine(BRAKE, 0);  // brake by engine
    set_steering(ZERO);  // set steering to center point
    delay(p.delay_ms);  // wait some time...
  }
}
