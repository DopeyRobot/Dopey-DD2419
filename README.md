# Dopey Handbook

## Rosbag
### Record rosbag

1. `cd ~/<workspace>/bagfiles` - IMPORTANT!

-  `rosbag record -a ` - record everything


-  `rosbag record -O subset <Topic-1> <Topic-2> ... ` - record only specific topics

### Play rosbag

- `roslaunch launch mainSIMULATION.launch`
  
- `rosbag play --clock --pause <bag-name>.bag` - will start in paused mode, press SPACE to play

## Packages
### Joystick Package
`sudo apt-get install ros-noetic-joy`

### Cam Packages

