uarm ros package
=================

ROS tools for UArm by UFactory.


Dependency
--------------
- rosserial
- UF_uArm (arduino library)

Install
===============
0. Install arduino sdk from Arduino.cc[http://arduino.cc/en/Main/Software].

1. Install rosserial

```bash
    $ sudo apt-get install ros-hydro-rosserial
```

3. checkout uarm(this!) and build

```bash
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/OTL/uarm.git
    $ cd ~/catkin_ws
    $ catkin_make
```

4. generate ros_lib (ros message file for arduino)

```bash
    $ rosrun rosserial_arduino make_libraries.py ~/sketchbook/libraries
```


2. install UF_uArm arduino library

```bash
    $ cd  ~/sketchbook/libraries
    $ git clone https://github.com/UFactory/UF_uArm.git
```

5. make a link to the sketch directory 

```bash
    $ ln -s ~/catkin_ws/src/uarm/uArmROS ~/sketchbook/
```

6. Start Arduino SDK, and install sketch, uArmROS

  File -> Sketchbook -> uArmROS

Run
================

```bash
    $ rosrun rosserial_python serial_node.py
    $ rostopic list
```

Pub/Sub
=================

Publish
-----------
* /uarm/button/d4 (std_msgs/Bool): button state (D4)
* /uarm/button/d7 (std_msgs/Bool): button state (D7)
* /uarm/joint_states (sensor_msgs/JointState): TODO: raw joint AD value

Subscribe
-----------
* /uarm/joint_commands (uarm/Joints): move joints (unit looks [deg] and [mm])
* /uarm/gripper (std_msgs/Bool): true -> hold, false -> release (only tested for pump)

