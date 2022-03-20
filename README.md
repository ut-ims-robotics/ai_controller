# ai_controller
A catkin package for controlling a ROS robot using AI (e.g. Darknet &amp; YOLOv3)
#### How to run ai_controller on ROBOTONT to recognize objects and publish Twist messages with angular position of object "person":
 - Connect to the ROBOTONT robot (with display access)
 - Clone and build ai_controller package in your catkin workspace
 - Inside the package navigate to the branch `noetic-devel`:</br>
 
        git checkout noetic-devel
 - In /ai_controller/launch/ai_controller.launch uncomment line 5: `<remap from="image_raw" to="/camera/color/image_raw" />`and save changes
 - Open terminal window:</br>

        roslaunch ai_controller ai_controller.launch
 - To check the Twist messages with angular position of object "person" published:
 
        rostopic echo cmd_vel
