
# My Robot test environment
---
##M Launch a test world with my robot.  
$roslaunch test.launch  

##M Move my robot.    
$./key_teleop.py  

##M Test my kelman filter. 
Launch a world.  
$./kelman.py  
Play it.
(Adjust the Standard Deviation Parameter and noise)

##M Test ORB_SLAM with odometry.  
Launch a world.  
Launch ORB_SLAM with odometry.  
$roslaunch RunWithTf_MyRobot.launch  
(https://github.com/scomup/ORB_SLAM_with_odometry)  
$./tfreader.py  

