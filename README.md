yolo preprocessor module
===

How to build
---
Clone this repository in the catkin workspace (any workspace is ok)
~~~
cd ~/catkin_ws 
git clone https://github.com/moyamung/yolo_preprocess.git
~~~
To build, just use 
~~~
catkin_make
~~~

### Handle with errors
 - When error occurs while `catkin_make`, with error message talks about duplicate code or msg <-- something like this
   - This problem occurs when this package builds with detected_image_saver or darknet ros
   - delete the duplicated msg source code folder in this folder


How to use
---
~~~
rosrun yolo_preprocess yolo_preprocess
~~~

Information
---
Subscribing topics:
 - /camera/color/image_raw
 - /darknet_ros/bounding_boxes   

Publishing topic:
 - /target_detection
