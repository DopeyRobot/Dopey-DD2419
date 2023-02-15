# EXCLUDE="/camera/color/image_raw/compressed/(.*)|/camera/color/image_raw/compressedDepth/(.*)|/camera/depth/image_rect_raw/compressed/(.*)'|/camera/depth/image_rect_raw/compressedDepth/(.*)"
EXCLUDE="/camera/(.*)|/usb_cam/(.*)"
INCLUDE="/camera/color/image_raw /camera/depth/color/points /usb_cam/image_raw"
cd /home/robot/dd2419_ws/bagfiles
rosbag record -a -x $EXCLUDE $INCLUDE