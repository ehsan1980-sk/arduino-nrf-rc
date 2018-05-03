#include "nRF24L01.h"
#include "RF24.h"

#define BAUDRATE 9600

#define NRF_CE 9
#define NRF_CS 10
#define NRF_ADDR 0
#define NRF_CHAN 0x60
#define NRF_PAYLOAD_SIZE 32

#define JOY_ENG_PIN 0
#define JOY_STEARING_PIN 1

#define JOY_THRESHOLD 1

#define PWM_LOW_LIMIT 50  // low limit of PWM
#define PWM_LIMIT 255  // limit for PWM of engine (0-255)

struct JOY {
  uint16_t y_middle;
  uint16_t x_middle;
  uint16_t zero;
  uint16_t maximum;
};

JOY joy_eng = {.y_middle = 528, .x_middle=512, .zero = 0, .maximum = 1023};

RF24 radio(NRF_CE, NRF_CS);
uint8_t address[][6] = {"1Node","2Node","3Node","4Node","5Node","6Node"};

const uint8_t delay_ms = 1000 / 200;

uint64_t time_from_last_command = 0;  // ms from last command

void setup() {
  Serial.begin(BAUDRATE); //открываем порт для связи с ПК

  radio.begin();
  radio.setAutoAck(1);
  radio.setRetries(0, 15);  // delay, count of retry

  radio.setPayloadSize(NRF_PAYLOAD_SIZE);  // package size

  radio.openWritingPipe(address[NRF_ADDR]);  // addresss
  radio.setChannel(NRF_CHAN);  // chanel

  radio.setPALevel(RF24_PA_MAX);  // RF24_PA_MIN | RF24_PA_LOW | RF24_PA_HIGH | RF24_PA_MAX
  radio.setDataRate(RF24_1MBPS);  // RF24_2MBPS | RF24_1MBPS, | RF24_250KBPS

  radio.powerUp();
  radio.stopListening();  // transmiter mode
  
  pinMode(JOY_ENG_PIN, INPUT);
  pinMode(JOY_STEARING_PIN, INPUT);
}

void loop() {
  time_from_last_command = millis();
  
  int8_t eng_direction = 0;  // 1 - fwd, 2 - bcwrd, 0 - none
  uint8_t eng_pwm = 0;      // pwm for engine

  uint16_t stearing_read = analogRead(JOY_STEARING_PIN);
  uint8_t stearing_val = map(stearing_read, 0, 1023, 0, 255);

  uint16_t acc_read = analogRead(JOY_ENG_PIN);  // acceleration control

  // forward
  if (acc_read > (joy_eng.y_middle + JOY_THRESHOLD)) {
    eng_direction = 1;
    eng_pwm = map(acc_read, joy_eng.y_middle, joy_eng.maximum, PWM_LOW_LIMIT, PWM_LIMIT);
  }
  // backward
  else if (acc_read < (joy_eng.y_middle - JOY_THRESHOLD)) {
    eng_direction = 2;
    eng_pwm = map(acc_read, joy_eng.y_middle, 0, PWM_LOW_LIMIT, PWM_LIMIT);
  }

  char t[100];
  snprintf(t, 100, "stear: %i, eng_dir: %i, eng_pwm: %i", stearing_val, eng_direction, eng_pwm);
  Serial.println(t);

  // payload
  uint8_t payload[NRF_PAYLOAD_SIZE];
  payload[0] = 1;  // move command
  payload[1] = stearing_val;
  payload[2] = eng_direction;
  payload[3] = eng_pwm;
  payload[7] = delay_ms;
  
  radio.write(payload, sizeof(payload));

  while ((millis() - time_from_last_command) < delay_ms) {
    delay(0);  // sleep 1ms
  }
}
