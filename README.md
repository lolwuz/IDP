![alt text](https://i.imgur.com/RtWOTL1.png)

# IDP 2018 - Slimme Robot Groep 9

## Team
* Jan Julius de Lang
* Wesley Hofma
* Moustafa Elhagaly
* Marten Hoekstra
* Niek van Dijk
* Nies Douwe Yntema
* Wytse de Vries
* Jornt Reitsma
* Teake Cramer
* Koen Lukkien

## Install:
* Rosbots ISO (https://docs.google.com/forms/d/e/1FAIpQLSeq5Sq8QyBLBJ5ncH46fNSh4MTQbexsWpMBKhSUpJjba1MIZQ/viewform)
* I2C (sudo apt-get install i2c-tools)
* GPIO_Adafruit (https://github.com/adafruit/Adafruit_Python_GPIO)

### Build packages:
```
cd ros
catkin_make
```
### Controller run:
```
cd ros
source devel/setup.bash
roslaunch robot_joystick_bringup controller.launch
```

### Robot run:
```
cd ros
source devel/setup.bash
roslaunch robot_joystick_bringup mnangagwa.launch
```
