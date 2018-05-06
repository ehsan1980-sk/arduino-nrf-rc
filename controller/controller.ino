#include "nRF24L01.h"
#include "RF24.h"

#define BAUDRATE 115200

// NRF24 pins
#define NRF_CE 9
#define NRF_CS 10
#define NRF_ADDR 0
#define NRF_CHAN 0x60
#define NRF_PAYLOAD_SIZE 32

// Joystics pins
#define JOY_ENG_PIN 0
#define JOY_STEERING_PIN 1

#define JOY_FIRST_BUTTON 2

// Engine PWM controls
#define PWM_LOW_LIMIT 50  // low limit of PWM
#define PWM_LIMIT 255     // limit for PWM of engine (0-255)

// Engine mode control params
#define FREE 0
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3

// Commands
#define CMD_NONE 0
#define CMD_MOVE 1
#define CMD_BRAKE 2


struct JOY {
  uint16_t middle;
  uint16_t zero;
  uint16_t maximum;
};

JOY joy_eng = {.middle = 528, .zero = 0, .maximum = 1023};
JOY joy_steering = {.middle = 527, .zero = 0, .maximum = 1023};

RF24 radio(NRF_CE, NRF_CS);
uint8_t address[][6] = {"1Node","2Node","3Node","4Node","5Node","6Node"};

const uint8_t delay_ms = 1000 / 200;  // Refresh rate - 200 Hz

uint64_t time_from_last_command = 0;  // ms since last command


struct SteeringResponse {
  bool engaged = false;
  uint8_t steering_value = 127;  // 0-255 - angle for steering servo
};

struct EngineResponse {
  bool engaged = false;
  uint8_t eng_direction = FREE;  // engine direction FREE | FORWARD | BACKWARD | BRAKE
  uint8_t eng_pwm = 0;           // pwm for engine 0-255
};

/*
 * Handle of engine control, set engine direction and PWM.
 */
EngineResponse handle_engine_control(uint16_t acceleration_read) {
  EngineResponse r;

  // forward
  if (acceleration_read > joy_eng.middle) {
    r.eng_direction = FORWARD;
    r.eng_pwm = map(acceleration_read, joy_eng.middle, joy_eng.maximum, PWM_LOW_LIMIT, PWM_LIMIT);
    r.engaged = true;
  }
  // backward
  else if (acceleration_read < joy_eng.middle) {
    r.eng_direction = BACKWARD;
    r.eng_pwm = map(acceleration_read, joy_eng.middle, 0, PWM_LOW_LIMIT, PWM_LIMIT);
    r.engaged = true;
  }

  return r;
};

/*
 * Handle steering control.
 */
SteeringResponse handle_steering_control(uint16_t steering_read) {
  SteeringResponse r;
  
  if (steering_read > joy_steering.middle) {
    r.steering_value = map(steering_read, joy_steering.middle, joy_steering.maximum, 128, 255);
    r.engaged = true;
  } 
  else if (steering_read < joy_steering.middle) {
    r.steering_value = map(steering_read, joy_steering.middle, joy_steering.zero, 126, 0);
    r.engaged = true;
  }

  return r;
};

void setup() {
  Serial.begin(BAUDRATE);

  radio.begin();
  radio.setAutoAck(1);
  radio.setRetries(0, 15);  // delay, count of retry

  radio.setPayloadSize(NRF_PAYLOAD_SIZE);  // package size

  radio.openWritingPipe(address[NRF_ADDR]);  // address
  radio.setChannel(NRF_CHAN);  // chanel

  radio.setPALevel(RF24_PA_MAX);  // RF24_PA_MIN | RF24_PA_LOW | RF24_PA_HIGH | RF24_PA_MAX
  radio.setDataRate(RF24_1MBPS);  // RF24_2MBPS | RF24_1MBPS, | RF24_250KBPS

  radio.powerUp();
  radio.stopListening();  // transmiter mode
  
  pinMode(JOY_ENG_PIN, INPUT);
  pinMode(JOY_STEERING_PIN, INPUT);
  
  pinMode(JOY_FIRST_BUTTON, INPUT_PULLUP);
}

void loop() {
  time_from_last_command = millis();
  
  uint8_t command = CMD_NONE;  // command by default

  // steering_control
  uint16_t steering_read = analogRead(JOY_STEERING_PIN);

  // acceleration control
  uint16_t acc_read = analogRead(JOY_ENG_PIN);

  SteeringResponse sr = handle_steering_control(steering_read);
  EngineResponse er = handle_engine_control(acc_read);

  // temporary brake switch is engine joystic switch
  if (digitalRead(JOY_FIRST_BUTTON) == LOW) {
    command = CMD_BRAKE;  // brake has most priority
  }
  // else if controls were engaged car should move 
  else if (sr.engaged || er.engaged) {
    command = CMD_MOVE;
  }

  // build payload
  uint8_t payload[NRF_PAYLOAD_SIZE];
  payload[0] = command;
  payload[1] = sr.steering_value;
  payload[2] = er.eng_direction;
  payload[3] = er.eng_pwm;
  payload[7] = delay_ms;
  
  radio.write(payload, sizeof(payload));

  // send debug info
  char t[100];
  snprintf(t, 100, "cmd: %i, steer: %i (%i), eng_dir: %i, eng_pwm: %i", 
                    command, sr.steering_value, steering_read, er.eng_direction, er.eng_pwm);
  Serial.println(t);

  // throttle
  while ((millis() - time_from_last_command) < delay_ms) {
    delay(0);  // sleep
  }
}
