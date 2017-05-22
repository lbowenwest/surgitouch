# Surgitouch Code

## Project Structure

- `src` Contains programs to run on the host computer (nothing here yet)
- `firmware` Contains the firmware for the arduino (currently uses make)
- `bin` and `build` this directories won't show up until you run something, and are excluded by gitignore as they don't want to be shared between computers
- `tests` this is where any tests will go

## Procedure

When doing any work on a feature checkout a different branch `git checkout -b <branch name>`. 
For instance if working on the roboclaw communication, checkout a branch called "roboclaw", then when work is finished, push your branch to the main repo, `git push origin <branch name>`.
Ensure that any commit messages are sensible, and informative.

## Firmware 

Currently the firmware is setup using `make`, this requires a bit of setup, so it may be easier to test the firmware code using something like the arduino IDE or [Platformio](http://platformio.org/)

To build the firmware run `make` in the firmware directory. `make clean` cleans the directory.