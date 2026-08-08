#  The Headset

"The Headset" is a prototype control system for a wearable headset built around an Arduino Nano Every. The goal is to coordinate physical hardware behavior on the microcontroller at the tap of buttons that are wired to the board, in order to automate the application of a cold compress on the wearer's eyes for the goal of treating eye itching and pain caused by allergies.

I am working on this 'automated allergy icer' because each spring, I repeatedly find myself holding a cold washcloth to my eyes to fight off the pain that comes with my allergies. So my goal was to “automate” the washcloth: I wanted a hands-free way to apply and maintain a cold compress on one or both eyes. The prototype has achieved this key functionality, but is in need of refinement.
- There are currently three buttons. The first two toggle the left and right "arms" up and down. At the end of each arm is a small cold pack attached to it. The third button is an Emergency Stop button, cutting output to the motors.

## Navigation
- This is a PlatformIO project, allowing for easier development in VSCode as opposed to the Arduino IDE.
- The `src` folder contains the code in C++ for the allergy icer. The `include` and `lib` folders may contain other code files in the future as I change the code structure.
- The `CAD` folder contains all cad related things. Parts will contain stl files for 3d printed parts and drawings will contain part and assembly drawings.
- Documentation will contain the initial BOM and an updated one for future iterations, as well as other notes I have made about this project.

## Hardware Control In The Sketch
The C++ source code is structured as a small embedded control loop rather than a collection of one-off callbacks. In `sketch/main.cpp`, the firmware keeps the live hardware state in a few shared variables (arm state, emergency stop enabled) and updates them through handlers called by `checkButtons()`. Those handlers are intentionally lightweight, while the `updateSetpoints()` function, also called by the `periodic()` loop, is responsible for quickly applying that state to the hardware.

The supporting C++ files reinforce the split between main logic and hardware-managing code: 
- Pin mapping and board-specific constants live in `src/Constants.hpp`
- Button handling is wrapped in `DigitalInput` for debouncing and edge detection
- Temperature sensing is isolated in `Thermometer` so the main sketch can treat sensing as a service rather than embedding hardware details inline. 

This way, the firmware can keep safety-sensitive behavior local to the microcontroller and avoid relying on web requests for timing.

Motor control is still in progress, but the architecture already has the intended structure: `main.cpp` computes desired setpoints from the current arm state, and the lower-level DCMotor.hpp consumes those setpoints once it is fully wired up.

## How to Use
- Use the resources in this repository to obtain the materials necessary to build the headset. Either clone this repository or specifically download the STL files to print the custom parts.
 - Note: As the current CAD version is still under refinement, it is not uploaded to the CAD folder yet.
- Download the Arduino IDE and copy the code into a `.ino` file. If you didn't clone the repository, make sure that `TheHeadset.ino` is inside a folder calledd "TheHeadset" to use in the Arduino IDE. 
- Or use the PlatformIO extension in your IDE (like VSCode), clone the repo, and open this Arduino project there.

## Current and Future Plans (in this project's Github Issues)
I intend to make improvements for this prototype and turn it into a refined product. I still intend to stay open source. Future iterations will:
- Use a more compact microcontroller - Arduino Nano Every - and design a custom PCB to lighten the design in order to improve user comfortability.
- Large modifications to the frame to improve appearance and user comfortability.
- Improve consistency of arm subsystems by switching to a different DC motor with a custom-designed gearbox.
- Create a better way of informing the user that a cold pack has warmed: I will likely use LEDs or a buzzer for this purpose.
- Introduce a self-refrigeration mechanism to increase time between cold pack replacing. This enhancement may feature ice cubes or refrigerant.
 - This is a long-term improvement, something to attempt once the others are successfully completed.

## License
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.