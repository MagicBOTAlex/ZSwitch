# This is ZSwitch!
![image](https://github.com/user-attachments/assets/dbee23c3-8310-46dd-8ac8-5d697cf7f815)
An open source 2 filament MMU for my P1S, focused on simplicity. I never planned on open-sourcing this, but here are the project files.
The code is a complete mess, since I focused on speed of development, and not quality.
A major reason for this project is I wanted the community to know that this was possible.
Another reason is because I don't want to support Bambu anymore. (Hate when companies remove features)

#### Pros
- Cheap
- Simple
- Apparently supports TPU
- Designed for P1S
- I still use Bambu Studio
- USB-C PD 👍
- Doesn't support Bambu
- Is open-source

#### Cons
- Bad software (Too early of a project)
- Not connected to the actual slicer (It's just a button that controls when to switch the filament. May change in the future, since I'm using an ESP32-C3)
- No community (Yet? idk)

##### Reason for only 2 filaments
I only designed this for 2 filaments, because I could not see myself use more than 2.
I wanted dissalvable supports and maybe PLA + TPU designs.

##### Inspiration/design
The mechanism was inspired by [TetraFlow](https://github.com/apoorv1in/TetraFlow_MMU)
But it felt like it had too many components.

### Demo video [Youtube](https://youtu.be/LOB4sRO39Bo)
[![Demo video](http://img.youtube.com/vi/LOB4sRO39Bo/0.jpg)](https://youtu.be/LOB4sRO39Bo)
### Dual TPU demo [Youtube](https://youtu.be/BcuDLzC1Btk)
[![Demo video](http://img.youtube.com/vi/BcuDLzC1Btk/0.jpg)](https://youtu.be/BcuDLzC1Btk)
##### Internals GIF
![ezgif-4bcd5e0f03a0ef](https://github.com/user-attachments/assets/d12f7e44-84cf-4363-b937-c65ca188389a)
##### Printed results
![2025-02-18 02 24 16](https://github.com/user-attachments/assets/ec4de492-c135-4f0e-83e8-bb43668c63a2)
![20250218_171123](https://github.com/user-attachments/assets/39550d14-3765-4754-8e4f-1bf2e6e71a00)
![20250224_081407](https://github.com/user-attachments/assets/a799c66d-8ffb-44d7-8174-590495202833)
had some bleeding/stringing because of wet TPU

### MMU
![20250218_010620](https://github.com/user-attachments/assets/1d19f10f-dc9e-4824-b82c-c2926f4becf0)
![20250218_010822](https://github.com/user-attachments/assets/5cd85980-0cf1-4fd2-b591-0bb1e85c5c09)

##### MMU circuits
![20250218_010315](https://github.com/user-attachments/assets/afc427bf-3e34-4655-a244-776b1d5f0d38)
![20250218_010326](https://github.com/user-attachments/assets/9d4fd046-55ac-4572-876a-0742dc3d2c9d)
###### Good luck making this circuit lol

##### Mock-up schematics
![2025-02-18 01 18 58](https://github.com/user-attachments/assets/903994d8-51bd-4910-9cfe-e601e07107da)

### Reason for name
Every design I make is Z + the project.
So this is ZSwitch.
(Yeah, I'm not good at names!!!)

### Material list
This is the list of components I used.
A better design would require less.
(Excluding 3D printed parts, and small parts)
- 2: TMC2209
- 2: Nema steppers
- 1: Esp32-C3 (Yes, I wanted to add MQTT support. But lazy)
- 1: USB-C PD Trigger board
- 1: Step-Down converter
- 2: Capasitors (For stepper drivers)
- 2: Arcade switches
- 3: Skateboard bearings
- 1: Filament grabbing gear (I got mine from an old sherpa mini extruder)

## DISCLAIMER
As per usual, if anything of my design causes damage to you or your machine, it is your own fault.
I cannot be held liable for the decisions you make.

### Contributting
I honestly don't expect anyone to continue this. But if anyone wants to, then I'll be here.
Here's a small list of TODO. All of these will be solve if the software is rewritten from scratch. But if we want to keep patching up this version, then go ahead:
- [ ]: Create functions/methods to reuse code.
- [ ]: Homing inconisitant. (Doesn't work at certain start rotations)
- [ ]: Save the calibration data to flash.
- [ ]: Seperate ready task from calibration task. (Everything is currently just in Calibration.h
- [ ]: StallGoard to stop pushing after hitting extruder.

Should be good enough for now.


Cya ♥ 
-Alex
