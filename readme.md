## Arduino + NRF24 RC Car
### Components
2 * NRF24L01
2 * Arduino Nano
1 * L298 H-bridge
1 * SG90 servo for steering
2 * Dual-axis joystick (KY-023)

### NRF + Arduino connection
For both Arduino and NRF used the same connections:

    CE  -> D9
    CSN -> D10
    MOS -> D11
    SCK -> D13
    MSK -> D12

### Transmitter connections
NRF setup + joysticks

    Engine potentiometer to A0
    Wheel control potentiometer to A1

### Receiver connections
Same NRF setup + L298(pwm to `enable <chanel>`, directions to in<n>, in<n+1>)

    Engine PWM -> D6
    Forward direction -> D3
    Backward direction -> D4

### Before use
Put `/libraries` content to the standard Arduino Libriries directory (location depends on platform)

Dependencies to install (download and copy to Arduino library directory):

- https://github.com/nRF24/RF24
- https://github.com/nabontra/ServoTimer2

### Protocol
Every 20 ms controller sends commands to the car. payload size - 32 bytes.
Command format (\[\] - one byte):
    
    [command_type][p1][p2][p3][p4][p5][p6][delay_between_commands][24 reserved bytes]

`move` command format:
    
    [1][<steering angle>][<eng_direction>][<eng_pwm>][3 empty bytes][20 <ms>][24 empty bytes]

### todo
[x] Basic car and controller code
[ ] Response payload to commands
[ ] LCD display on the controller board to show feedback
[ ] 3d model for controller board
