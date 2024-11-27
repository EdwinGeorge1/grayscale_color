# grayscale_color
ros2_ws.zip >> contain all workspace 
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
image_conversion Node:
The image_conversion node should Subscribe to the image topic (colored image) published by usb_cam package.
The node should host (server) a ros2 service (bool), to change its mode - 
Mode 1: Greyscale
Mode 2: Color
The Ros2 node should let the user change the mode of the node, through the above service.
Based on the mode, Convert the RGB/BGR input image to grayscale (mode 1) or publish it without conversion (mode 2)
Publish the image on a ros2 topic.


usb_cam  https://index.ros.org/p/usb_cam/

how to run this file

ros2 launch cpp_package start.launch.py  >> this code launches all code  including camera and c++codes 


ros2 service call /change_mode std_srvs/srv/SetBool "{data: true}"  >> sent through terminal to change mode 1,2

ros2 run image_view image_view --ros-args -r image:=/camera1/image_out >>> two view captured image
