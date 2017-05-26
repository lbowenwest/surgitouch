# Surgitouch Code

## Project Structure

- `src` Contains programs to run on the host computer (nothing here yet)
- `firmware` Contains the firmware for the arduino
- `lib` Contains any Arduino libraries necessary for the ROS node
- `bin` and `build` this directories won't show up until you run something, and are excluded by gitignore as they don't want to be shared between computers
- `tests` this is where any tests will go

## Procedure

When doing any work on a feature checkout a different branch `git checkout -b <branch name>`. 
For instance if working on the roboclaw communication, checkout a branch called "roboclaw", then when work is finished, push your branch to the main repo, `git push origin <branch name>`.
Ensure that any commit messages are sensible, and informative.
