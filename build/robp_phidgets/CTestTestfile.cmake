# CMake generated Testfile for 
# Source directory: /home/robot/dd2419_ws/src/robp_robot/robp_phidgets
# Build directory: /home/robot/dd2419_ws/build/robp_phidgets
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_robp_phidgets_roslaunch-check_launch "/home/robot/dd2419_ws/build/robp_phidgets/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/robot/dd2419_ws/build/robp_phidgets/test_results/robp_phidgets/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/robot/dd2419_ws/build/robp_phidgets/test_results/robp_phidgets" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/robot/dd2419_ws/build/robp_phidgets/test_results/robp_phidgets/roslaunch-check_launch.xml\" \"/home/robot/dd2419_ws/src/robp_robot/robp_phidgets/launch\" ")
set_tests_properties(_ctest_robp_phidgets_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/robot/dd2419_ws/src/robp_robot/robp_phidgets/CMakeLists.txt;112;roslaunch_add_file_check;/home/robot/dd2419_ws/src/robp_robot/robp_phidgets/CMakeLists.txt;0;")
subdirs("gtest")
