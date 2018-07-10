eye-to-hand calibration 
=========================
* Build a workpiece coordinate system and get 4 points like *(0,0,0)*, *(10,0,0)*, *(0,10,0)*, *(0,0,10)*.<br> 
* Get coordinates in camera coordinate system *(Xc,Yc,Zc)* of the 4 points by using `rosrun kinect2_viewer click_rgb`.<br> 
* Then calculate the camera to workpiece transformation matrix A with the 4 points informations(__Pay attention to the unit scale__).<br> 
* Detect the target and get the pixs *(x1,y1)*,then use click_rgb.cpp get the camera coordunate *(X1c,Y1c,Z1c)*.<br> 

### click_rgb.cpp
Get *(Xc,Yc,Zc)* by pixs *(x,y)* of color image and depth image .we can get the *(Xc,Yc,Zc)* from a cloud image with known pixs(x,y).<br> <br> 
**Ps**:Kinect2.0 SDK `MapColorFrameToCameraSpace()` can get the *(Xc,Yc,Zc)* by color frame and depthPoint frame at one pix .<br> 

## Requirements

Ubuntu16.04<br> 
ROS<br> 
Python2.7<br> 
Kinect V2 => (OpenNi2 OpenCL OpenCV)<br> 

## Usage

* Create ROS package as eye_to_hand <br> 
* Clone files to eye_to_hand/src directory<br> 
* Install iai_kinect2 [iai_kinect2] (from https://github.com/code-iai/iai_kinect2) <br> 
* Move the file `/eye_to_hand/src/calibration/2Dimg_to_3Dcamer/click_rgb.cpp` to  `/iai_kinect2/kinect2_viewer/src` <br> 
* Add follow lines in<br> `/iai_kinect2/kinect2_viewer/CMakeLists.txt`:<br> 
```Bash
	add_executable(click_rgb src/click_rgb.cpp) 
	target_link_libraries(click_rgb
	  ${catkin_LIBRARIES}
	  ${OpenCV_LIBRARIES} 
	  ${PCL_LIBRARIES}
	  ${kinect2_bridge_LIBRARIES}
	)

	install(TARGETS click_rgbs
	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
```
 
	then **make** Package iai_kinect2<br> 
* `rosrun kinect2_viewer click_rgb`<br>
* More information [ROS下Kinect2的运用]	(https://blog.csdn.net/sunbibei/article/details/51594824)

### files in eye_to_hand/src/
`kinect2_color.py` : get color image from Kinect2<br>
`kinect2_depth.py` : get depth image from Kinect2<br>
`kinect2_points.py`: get cloudPoints image from Kinect2<br>

#### files in eye_to_hand/src/calibration/ 
`camera_calibration.py` : calibrate Kinect2 And get camera matrix(Internal and External)<br>
`get_depth_imgs.py` : subscriber ROS node get depth image of Kinect2<br>
`get_imgs.py` : take some photos by Kinect2<br> press **Space** to take one<br>

