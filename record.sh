# EXCLUDE="/camera/color/image_raw/compressed/(.*)|/camera/color/image_raw/compressedDepth/(.*)|/camera/depth/image_rect_raw/compressed/(.*)'|/camera/depth/image_rect_raw/compressedDepth/(.*)"
EXCLUDE="/camera/(.*)"
INCLUDE="/camera/color/image_raw /camera/depth/color/points"
cd /home/robot/dd2419_ws/bagfiles
rosbag record -a -x $EXCLUDE -O subset $INCLUDE