# ConHid

ConHid is a program intending to bring support to Linux for the Switch 2 generation of Nintendo controllers. This includes:

- Pro Controller 2
- Joy Con 2
- GameCube Controller for Switch 2 

## Current Limitations
The limitations right now come from the driver being in userspace. Being in userspace makes it easier to produce a workable result at the cost of it being a seperate piece of software that has to be installed, and a background service. The two methods we have at performing in userspace is uinput and uhid.

uinput is the most usable for us, as it gives us the most ability to have a complete implementation.

## Intended Implementation
I intend to bring this to support the entire Switch 2 generation of controllers, and these subsets of features:

- Buttons, Joysticks (basic)
- IMU (Gyroscope, Magnetometer, Accelerometer)
- Mouse Input (Joy Con 2)
- Haptics / Rumble

Future ideas I have not gone through yet:

- NFC
- Audio (Pro Controller 2)

## Resources
A lot of resources used were from these two github repositories.

https://github.com/german77/JoyconDriver

https://github.com/ndeadly/switch2_controller_research