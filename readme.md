#  The Headset
"The Headset" is currently a prototype to automate the application of a cold compress on the wearer's eyes. The intended use scenario is to treat eye itching and pain caused by allergies.
I am working on this 'automated allergy icer' because each spring, I repeatedly find myself holding a cold washcloth to my eyes to fight off the pain that comes with my allergies. So my goal was to “automate” the washcloth: I wanted a hands-free way to apply and maintain a cold compress on one or both eyes. The prototype has achieved this key functionality, but is in need of refinement.
- There are currently three buttons. The first two toggle the left and right "arms" up and down. At the end of each arm is a small cold pack attached to it. The third button is sort-of a zeroing button, reseting the position of the encoders.

## Navigation
- The `TheHeadset` folder contains the code in Arduino/C++ for the allergy icer.
- The CAD folder contains all cad related things. Parts will contain stl files for 3d printed parts and drawings will contain part and assembly drawings.
- Documentation will contain the initial BOM and an updated one for future iterations, as well as other notes I have made about this project.

## How to Use
- Use the resources in this repository to obtain the materials necessary to build the headset. Either clone this repository or specifically download the STL files to print the custom parts.
- Download the Arduino IDE and copy the code into a `.ino` file. If you didn't clone the repository, make sure that `TheHeadset.ino` is inside a folder calledd "TheHeadset" to use in the Arduino IDE.

## Future Plans
I intend to make improvements for this prototype and turn it into a refined product. I still intend to stay open source. Future iterations will:
- Use a more compact microcontroller and design a custom PCB to lighten the design in order to improve user comfortability.
- Small modifications to the frame to improve appearance and user comfortability.
- Improve consistency of arm subsystems. I may switch from DC hobby motors with encoders to small steppers. (I used the hobby motors because that is what I had at the time)
- Create a better way of informing the user that a cold pack has warmed: I will likely use LEDs or a buzzer for this purpose.
- Introduce a self-refrigeration mechanism to increase time between cold pack replacing. This enhancement may feature ice cubes or refrigerant.