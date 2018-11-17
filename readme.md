## Arduino + NRF24 RC Car and Controller
### Components

    2 * NRF24L01
    2 * Arduino Nano
    1 * L298 H-bridge (or MX1508)
    1 * SG90 servo for steering
    2 * Dual-axis joystick (KY-023)

### Before use
Put `/libraries` content to the standard Arduino Libriries directory (location depends on platform)

Dependencies to install (download and copy to Arduino library directory):

- https://github.com/nRF24/RF24
- https://github.com/nabontra/ServoTimer2

### Protocol
Every 50ms controller sends commands to the car. payload max size - 32 bytes.

```
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
```

### todo

- [x] Basic car and controller code
- [ ] Response payload to commands
- [ ] LCD display on the controller board to show feedback
- [ ] 3d model for controller compartment
