
# PCA9685 Custom Chip 

- [todo - link to image]()
- [chip data sheet](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf)

The PCA9685 Custom Chip simulates the NXP chip of the same name. It provides 16 channels of PWM control that 
allow driving of LEDs or small servos. 

The chip has the following pin groups

| Name         | Description                                            |
| ------------ | ------------------------------------------------------ |
| `LED0-LED15`   | Output pins used to drive the output channels          |
| `A0-A5`        | Address pins. See [addressing](#adressing) below for more info    |
| `VCC, GND`     | The usual power and ground pins                        |
| `SCL, SDA`      | THe I2C pins. See [I2c](#i2c-comms) below for more info           |
| `OE`           | Output enable  pin - unused                       |
| `EXTCLK`       | External Clock pin - unused                            |


### Addressing
There are a maximum of 64 possible programmable addresses using the 6 hardware
address pins. Two of these addresses, Software Reset and LED All Call, cannot be used
because their default power-up state is ON, leaving a maximum of 62 addresses.

The special address values are:
- 0x06 - software reset address value
- 0x30 - led all call

### I2C Comms 
The device uses I2C for comms with the MCU.

A good Arduino library to use on the Arduino side is the Adafruit PWM Server Library (https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)


## Implementation details
The custom chip supports many (but not all) of the I2C commands specified in the NXP datasheet. The section below lists the supported commands links to the appropriate section in the NXP document.

### reset 
Write only. Resets the chip to power on. all channels are set to low and no timers are running. 

This command uses the special `GENERAL_CALL_ADDRESS (0)` on the I2C bus and a specific command word (`0x6`)

Link - Section 7.3.6
### mode 1 (0x0)
- Read returns the value of the mode 1 register
- Write sets some mode settings and executes some operations:
  - `SLEEP` - sets the chip to low power mode and stops all the timers. It also sets the `RESTART` bit (used as part of restart sequence)
  - `RESTART` - restarts all timers using previously set PWM values if any. Resets sleep bit
  - `AI` - sets auto increment mode (this implementation uses this implicitly)
  - `ALLCALL` - sets ALL call behaviour (partially supported)
  - `SUBx` - enables sub addressing  (not supported)  

Link - Section 7.3.1
### mode 2 (0x1)
- Read returns the value of the mode 1 register
- Write not implemented

Link - Section 7.3.2

### Prescale (0xFE)
- Read returns the value of the prescaler
- Write sets the prescaler

Note that the prescaler can only be set if the chip is in SLEEP mode.

### LED PWM Registers (0x6 - 0x45)
These registers are typically used in a sequence of all registers pertaining to a particular output chanel. There are 4 PWM registers per channel so a command sequence will use 5 bytes - identifying the starting register address followed by values for each of the 4 registers.

Reading these registers returns the current value.
Writing to these registers changes them.

The channel's PWM is only modified once all 4 values are written. 

Each channel has an ON and OFF register, each using 2 bytes, LOW and HIGH. The PWM values make use of 12 bits (8 of the low byte and 4 of the high byte.) In addition, 
bit 4 of the HIGH byte indicates whether the channel should be set to either ON or OFF (PWM is not used in that case)


### ALL LED PWM Registers (0xFA - 0xFD)
There is a set of 4 registers, that when written to, programs all channels with the same values for the ON/OFF PWM values.

## Attributes
The chip defines a number of attributes that can aid in debugging operation when used in Wokwi. 

| Name         | Description                                            | Default value             |
| ------------ | ------------------------------------------------------ | ------------------------- |
| debug_timer   | (0 or 1) used to set the internal timer frequency such that a PWM cycle lasts ~1s. This is useful when needing to inspect the visual impact of PWM                   | "0"                 |
| gen_debug | (0 or 1) when set to true, debug logs can be seen in the browser's developer tool window. This can be useful to confirm whether the operation of the program and chip is as expected | "0" |
| i2c_debug     | (0 or 1) enables debug log of the i2c communication exchange. This can be useful when debuggin i2c transactions      | "0"             |

## Simulator examples

- [PCA9685 Custom Chip](https://wokwi.com/projects/348856116302578258)