# Dopey Handbook

## Get access to Dopey remotely

0. CONNECT TO INTERNET
1. Plug screen, keyboard and mouse into Dopey
2. `hostname -I` - get current IP address
3. On client `ssh robot@<IP-address-from-above>` 
4. If first time, accept everything
5. Input password `Dopey5000`
6. Congrats! You now have access to Dopey's mind (files)!


## Control dopey with keyboard or joystick
0. Be inside Dopey
1. `cd ~/dd2419_ws`
2. Create at least 2 terminal tabs/windows
3. In one: `roscore`
4. In the other `roslaunch launch main.launch`
5. Use the joystick (hold A+move left stick [don't forget USB-stick]) or the keyboard (ikjl) to control Dopey
6. DOPE! 

## See Dopey's view live in Rviz
0. [Make sure Dopey's alive](#control-dopey-with-keyboard-or-joystick)
1. `roscore`
2. `roslaunch launch rviz.launch`

## Play a recorded rosbag and visualize in Rviz

1. `roscore`
2. `roslaunch launch mainSIMULATION.launch`
3. `cd ~/dd2419_ws/bagfiles`
4. `rosbag play --clock --pause <bag-name>.bag` - as defined in [Play rosbag](#play-rosbag)

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

## Packages
### Joystick Package
`sudo apt-get install ros-noetic-joy`

### Cam Packages


## Milestone 1 run through

0. Plug the battery into Dopey
1. Use mission control computer
2. [Invade Dopey's privacy](#get-access-to-dopey-remotely)
3. [Take control over Dopey's life](#control-dopey-with-keyboard-or-joystick)
4. `cd ~/<workspace>/bagfiles` - IMPORTANT!
5. `bash record.sh`
6. Drive around and collect data
7. `ctrl-c` in terminal of recording
8. Plug Dopey back into a screen, keyboard and mouse
9. Inside Dopey [relive the best moments of Dopey's life](#play-a-recorded-rosbag-and-visualize-in-rviz) (Use the rosbag tagged with today's date)
