# Surgitouch Code

## Project Structure

- `src` Contains programs to run on the host computer 
- `firmware` Contains the firmware for the arduino
- `tests` this is where any tests will go

## Firmware

The firmware for the ardunio is built using [Platformio](http://platformio.org/).
To build the firmware call `platformio run` from the root directory.
To upload the firmware `platformio run -t upload`.

