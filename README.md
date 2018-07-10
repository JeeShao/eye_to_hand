# eye-to-hand calibration 
Build a workpiece coordinate system and get 4 points like (0,0,0),(10,0,0),(0,10,0),(0,0,10).
Get coordinates in camera coordinate system (Xc,Yc,Zc) of the 4 points by using 'rosrun kinect2_viewer click_rgb'.
Then calculate the camera to workpiece transformation matrix A with the 4 points informations.
Detect the target and get the pixs (x1,y1),then use click_rgb.cpp get the camera coordunate (X1c,Y1c,Z1c).

###click_rgb.cpp
Get (Xc,Yc,Zc) by pixs(x,y) of color image and depth image .we can get the (Xc,Yc,Zc) from a cloud image with known pixs(x,y).
Ps:Kinect2.0 SDK 'MapColorFrameToCameraSpace()' can get the (Xc,Yc,Zc) by color frame and depthPoint frame at one pix .

##Requirements

Ubuntu16.04
ROS
Python2.7
Kinect V2 => (OpenNi2 OpenCL OpenCV)

##Usage

1.Create ROS package as eye_to_hand 
2.Clone files to eye_to_hand/src directory
3.Install iai_kinect2 from .[code-iai/iai_kinect2].(https://github.com/code-iai/iai_kinect2)
4.Move the file '/eye_to_hand/src/calibration/2Dimg_to_3Dcamer/click_rgb.cpp' to  '/iai_kinect2/kinect2_viewer/src' 
5.Add follow lines in '/iai_kinect2/kinect2_viewer/CMakeLists.txt':
	'add_executable(click_rgb src/click_rgb.cpp)
	target_link_libraries(click_rgb
	  ${catkin_LIBRARIES}
	  ${OpenCV_LIBRARIES}
	  ${PCL_LIBRARIES}
	  ${kinect2_bridge_LIBRARIES}
	)

	install(TARGETS click_rgb
	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)'
	then 'make' Package iai_kinect2
6.'rosrun kinect2_viewer click_rgb'
7.More information .[ROS下Kinect2的运用].(https://blog.csdn.net/sunbibei/article/details/51594824)
