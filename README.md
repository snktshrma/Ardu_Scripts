# Ardu_Scripts
1. Three scripts for basic maneuver of ArduCopter, i.e. moving in a square, circle and hexagon.
2. Launch file for mavros to connect ROS with Ardupilot SITL.
3. Updated iris model and added a LiDAR on top of it which publishes data over topic /scan.
4. Added an obstacle avoidance script, using LiDAR data, which will simply increase the altitude whenever there is an obstacle in the vicinity of the iris drone.
5. Added files related to hector slam. I have made necessary changes to the hector slam files related to reference and coordinate frames for tf.

## Commands for Obstacle avoidance:
T1: `roslaunch gzbo.launch`

T2: `../Tools/autotest/sim_vehicle.py -f gazebo-iris`

T3: `roslaunch apm.launch`

T4: `python obstacle_avoidance.py`

## Commands for Hector SLAM:
T1: `roslaunch gzbo.launch`

T2: `../Tools/autotest/sim_vehicle.py -f gazebo-iris`

T3: `roslaunch apm.launch`

T4: `roslaunch hector_slam_launch tutorial.launch`

