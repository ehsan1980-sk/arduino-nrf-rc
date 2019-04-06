#include "nRF24L01.h"
#include "RF24.h"


#define BAUDRATE 115200
// Radio Commands
#define CMD_NONE 1
#define CMD_ACTIVE 2


// NRF24 pins
#define NRF_CE 9
#define NRF_CS 10

// NRF24 settings
#define NRF_ADDR 0
#define NRF_CHAN 0x60
#define NRF_PAYLOAD_SIZE 32


// Payload datastructure
struct payload_t {
  uint8_t command;  // current controller command (none or active for now)

  // stick1 data
  uint16_t stick1_x;
  uint16_t stick1_y;
  bool stick1_p;

  // stick2 data
  uint16_t stick2_x;
  uint16_t stick2_y;
  bool stick2_p;

  // typical delay between packages
  uint8_t delay_ms;
};


// Stick datastructure
struct stick_settings_t {
  uint16_t x_middle;  // middle point of x axis
  float x_curve_a;  // coefficient for S curve mapping of x axis

  // same for y axis
  uint16_t y_middle;
  float y_curve_a;

  // pinout definition
  int8_t x_axis_pin;
  int8_t y_axis_pin;
  int8_t btn_pin;
};

// Low-level data about current stick state
struct StickResponse {
  bool engaged;
  bool pressed;
  uint16_t x_value;
  uint16_t y_value;
};


RF24 radio(NRF_CE, NRF_CS);
uint8_t address[][6] = {"1Node","2Node","3Node","4Node","5Node","6Node"};

const uint8_t delay_ms = 1000 / 200;  // Refresh rate - 200 Hz
uint64_t time_from_last_command = 0;  // ms since last command


stick_settings_t stick1 = {
  .x_middle = 512, 
  .x_curve_a = 0,
  
  .y_middle = 488,
  .y_curve_a = .5,

  .x_axis_pin = -1,
  .y_axis_pin = 0,
  .btn_pin = 2
};


stick_settings_t stick2 = {
  .x_middle = 502, 
  .x_curve_a = .65,
  
  .y_middle = 512, 
  .y_curve_a = 0,

  .x_axis_pin = 1,
  .y_axis_pin = -1,
  .btn_pin = -1
};


/*
 * Function to map linear movement of stick to S curve
 * x - input value (0-1023)
 * a - coefficient, lower is less curved
 */
uint16_t s_curve(int16_t x, float a) {
  return a * (1023 * 4 * pow( x/1023.0 - 0.5, 3) + 512) + (1 - a) * x;
};


/*
 * Setup for stick pins if available
 */
void init_stick(stick_settings_t stick) { 
  if (stick.x_axis_pin != -1) pinMode(stick.x_axis_pin, INPUT);
  if (stick.y_axis_pin != -1) pinMode(stick.y_axis_pin, INPUT);
  if (stick.btn_pin != -1) pinMode(stick.btn_pin, INPUT_PULLUP);
}


/*
 * Read stick state, center raw reading to middle axis point (512 ), map to values in S curve
 */
StickResponse read_stick(stick_settings_t stick) {
  StickResponse r;

  // Values by default
  r.x_value = 512;
  r.y_value = 512;
  r.engaged = false;

  uint16_t x_value = stick.x_axis_pin != -1 ? analogRead(stick.x_axis_pin) : 0;
  uint16_t y_value = stick.y_axis_pin != -1 ? analogRead(stick.y_axis_pin) : 0;
  r.pressed = stick.btn_pin != -1 ? digitalRead(stick.btn_pin) == LOW : false;

  if (x_value > (stick.x_middle + 1)) {
    r.x_value = map(x_value, stick.x_middle, 1023, 512, 1023);
    r.engaged = true;
  }
  else if (x_value < (stick.x_middle - 1)) {
    r.x_value = map(x_value, stick.x_middle, 0, 512, 0);
    r.engaged = true;
  }

  if (y_value > (stick.y_middle + 1)) {
    r.y_value = map(y_value, stick.y_middle, 1023, 512, 1023);
    r.engaged = true;
  }
  else if (y_value < (stick.y_middle - 1)) {
    r.y_value = map(y_value, stick.y_middle, 0, 512, 0);
    r.engaged = true;
  }

  r.x_value = s_curve(r.x_value, stick.x_curve_a);
  r.y_value = s_curve(r.y_value, stick.y_curve_a);

  return r;
};


void setup() {
  Serial.begin(BAUDRATE);

  radio.begin();
  radio.setAutoAck(1);
  radio.setRetries(0, 1);  // delay - 250us, retry - 0

  radio.setPayloadSize(NRF_PAYLOAD_SIZE);  // package size

  radio.openWritingPipe(address[NRF_ADDR]);  // address
  radio.setChannel(NRF_CHAN);  // chanel

  radio.setPALevel(RF24_PA_MAX);  // RF24_PA_MIN | RF24_PA_LOW | RF24_PA_HIGH | RF24_PA_MAX
  radio.setDataRate(RF24_1MBPS);  // RF24_2MBPS | RF24_1MBPS, | RF24_250KBPS

  radio.powerUp();
  radio.stopListening();  // transmiter mode

  init_stick(stick1);
  init_stick(stick2);
}


void loop() {
  time_from_last_command = millis();
  
  uint8_t command = CMD_NONE;  // command by default

  StickResponse s1r = read_stick(stick1);
  StickResponse s2r = read_stick(stick2);

  // if controls were engaged car should move 
  if (s1r.engaged || s2r.engaged || s1r.pressed || s2r.pressed) {
    command = CMD_ACTIVE;
  }

  // build payload
  payload_t p = {
    .command = command,
    .stick1_x = s1r.x_value,
    .stick1_y = s1r.y_value,
    .stick1_p = s1r.pressed,
    .stick2_x = s2r.x_value,
    .stick2_y = s2r.y_value,
    .stick2_p = s2r.pressed,
    .delay_ms = delay_ms
  };
  
  radio.write(&p, sizeof(payload_t));

  if (radio.isAckPayloadAvailable()) {
    uint8_t ack_payload[NRF_PAYLOAD_SIZE];
    radio.read(&ack_payload, sizeof(ack_payload));

    char t[100];
    snprintf(t, 100, "Got ack payload, 2b: %i %i", ack_payload[0], ack_payload[1]);
    Serial.println(t);
  }

  // send debug info
  char t[100];
  snprintf(t, 100, "cntrl (%i). s1x: %i, s1y: %i, s1p: %i, s2x: %i, s2y: %i, s2p: %i", 
                    p.command, p.stick1_x, p.stick1_y, p.stick1_p, p.stick2_x, p.stick2_y, p.stick2_p);
  Serial.println(t);

  // throttle
  while ((millis() - time_from_last_command) < delay_ms) {
    delay(0);  // sleep
  }
}
