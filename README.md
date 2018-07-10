eye-to-hand calibration 
=========================
* Build a workpiece coordinate system and get 4 points like (0,0,0),(10,0,0),(0,10,0),(0,0,10).<br> 
* Get coordinates in camera coordinate system (Xc,Yc,Zc) of the 4 points by using 'rosrun kinect2_viewer click_rgb'.<br> 
* Then calculate the camera to workpiece transformation matrix A with the 4 points informations.<br> 
* Detect the target and get the pixs (x1,y1),then use click_rgb.cpp get the camera coordunate (X1c,Y1c,Z1c).<br> 

### click_rgb.cpp
Get (Xc,Yc,Zc) by pixs(x,y) of color image and depth image .we can get the (Xc,Yc,Zc) from a cloud image with known pixs(x,y).<br> 
*Ps*:Kinect2.0 SDK 'MapColorFrameToCameraSpace()' can get the (Xc,Yc,Zc) by color frame and depthPoint frame at one pix .<br> 

## Requirements

Ubuntu16.04
ROS
Python2.7
Kinect V2 => (OpenNi2 OpenCL OpenCV)

## Usage

1.Create ROS package as eye_to_hand <br> 
2.Clone files to eye_to_hand/src directory<br> 
3.Install iai_kinect2 from .[code-iai/iai_kinect2].(https://github.com/code-iai/iai_kinect2)<br> 
4.Move the file '/eye_to_hand/src/calibration/2Dimg_to_3Dcamer/click_rgb.cpp' to  '/iai_kinect2/kinect2_viewer/src' <br> 
5.Add follow lines in<br> "/iai_kinect2/kinect2_viewer/CMakeLists.txt":<br> 
	'add_executable(click_rgb src/click_rgb.cpp)<br> 
	target_link_libraries(click_rgb<br> 
	  ${catkin_LIBRARIES}<br> 
	  ${OpenCV_LIBRARIES}<br> 
	  ${PCL_LIBRARIES}<br> 
	  ${kinect2_bridge_LIBRARIES}<br> 
	)<br> 

	install(TARGETS click_rgbs
	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)'<br>
 
	then make Package iai_kinect2<br> 
6.		'rosrun kinect2_viewer click_rgb'<br>
7.More information .[ROS下Kinect2的运用].(https://blog.csdn.net/sunbibei/article/details/51594824)
