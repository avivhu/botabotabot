# BoTaBoTaBoT
A robot with three omni-directional wheels.

I built this robot to learn about kinematics and for fun.

<img src="docs/anim.gif" style="width:400px;"/>


## Source code and inspiration
This project takes source code and inspiration from the following sources.

### Linorobot
https://github.com/linorobot/linorobot2

A library for robots with various kinematics by [Juan Miguel Jimeno](https://github.com/grassjelly).

### ESP32Encoder
https://github.com/madhephaestus/ESP32Encoder


## Other Resources

* https://en.wikipedia.org/wiki/Kiwi_drive
* https://github.com/manav20/3-wheel-omni
* https://github.com/YugAjmera/omni3ros_pkg
* [YouTube: Kinematics of Mobile Robots with Omni Directional Wheels](https://youtu.be/-wzl8XJopgg). Lecture by Dr. Mehran Andalibi.

## Images

### Platform kit
<img src="docs/platform_kit_parts.jpg" style="width:300px;"/>

### Breadboard
<img src="docs/breadboard.jpg" style="width:300px;"/>

### Prototype board
After testing on a breadboard, I soldered the components to a prototype board.
I tried to make it small, because I worried the platorm won't fit all the components.
It was laborious to fit all the components in to this tiny area.
In retrospect the platform has enough room, so I could have used more area.

<img src="docs/board_top.jpg" style="width:300px;"/>
<img src="docs/solder_back.jpg" style="width:300px;"/>

## Current view

<img src="docs/view_1.jpg" style="width:300px;"/>

### Odometry demonstration
This video demonstrates that the encoders are measuring wheel rotation speeds correctly.

[Motion odometry video](docs/measured_motion.mp4)

### Xbox controller input
The robot can be controlled with an Xbox Controller over Bluetooth.

<img src="docs/anim.gif" style="width:400px;"/>

[Video](docs/xbox_controller.mp4)

