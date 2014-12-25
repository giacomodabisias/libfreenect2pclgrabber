libfreenect2pclgrabber
======================

A grabber for the kinect2 which returns a registered pcl point cloud.

For now the size of the cloud is fixed (512x424) even if there is the initial support for a dynamic size. The idea is to get a point cloud up to 1920x1080 as the rgb image by interpolating the depth points. It is possible to obtain a full cloud (also containing Nans) or to obtain just a point cloud with good points. The first method is faster while the second one is a bit slower. Everything runs in parallel using openmp, but I will release a Cuda implementation soon. 

Depth is registered onto rgb, meaning that there are more good depth points, but there are some shadows of the rgb on the background. I am finishing the implementation of rgb onto depth registration in order to get less points, but no shadows.

There is also the possibility to serialize all the different frames by just enabling serialization. I will add soon the method to load the serialized "clouds".

For any questions don't hesitate to contact me at giacomo.dabisias@gmail.com
