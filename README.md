libfreenect2pclgrabber
======================

A grabber for the kinect2 which returns images and Point Clouds. 
There is asupport for multiple kinects and a ROS interface.
The ROS interface was tested on the same 4 core machine against the standard iai_kinect2:

	ai_kinect2 published color, depth and cloud at 30fps with 340% cpu
	this grabber published color, depth and cloud at 30fps using 120% cpu

both grabbers have been tested using the same parameters.

For any questions don't hesitate to contact me at giacomo.dabisias@gmail.com.

If you have issues with multiple kinects execute "echo 64 > /sys/module/usbcore/parameters/usbfs_memory_mb"
