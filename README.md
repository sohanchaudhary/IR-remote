| Supported Targets | ESP32 | ESP32-S2 | ESP32-C3 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- |
# IR Protocol Example

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This example illustrates how to encode and decode RMT signals with/to common IR protocols (e.g. NEC and RC5).


## How to Use Example

### Hardware Required

* A development board with supported SoC mentioned in the above `Supported Targets` table
* An USB cable for power supply and programming
* A 5mm infrared LED (e.g. IR333C) used to transmit encoded IR signals
* An infrared receiver module (e.g. IRM-3638T), which integrates a demodulator and AGC circuit.

Example connection :

| ESP chip                    | IR333C | IRM-3638T |
| --------------------------- | ------ | --------- |
| CONFIG_EXAMPLE_RMT_TX_GPIO  | Tx     | ×         |
| CONFIG_EXAMPLE_RMT_RX_GPIO  | ×      | Rx        |
| VCC 5V                      | √      | ×         |
| VCC 3.3V                    | ×      | √         |
| GND                         | GND    | GND       |


### Configure the Project

Open the project configuration menu (`idf.py menuconfig`).

In the `Example Configuration` menu:

* Select the infrared protocol used in the example under `Infrared Protocol` option.
* Set the GPIO number used for transmitting the IR signal under `RMT TX GPIO` option.
* Set the GPIO number used for receiving the demodulated IR signal under `RMT RX GPIO` option.
* Set the RMT TX channel number under `RMT TX Channel Number` option.
* Set the RMT RX channel number under `RMT RX Channel Number` option.

### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

### Main Objective
The main objective of this program is to make the device learn the timing signals of the recevied IR Signals. The received IR signals is stored in terms of the signal timing period for each bit high and low signal.