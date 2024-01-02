# Where-AM-I
OpenCV Robot Relocalization: This project utilizes OpenCV and IMU data to address the 'kidnapped robot' problem. It features two key components: 
1. Kidnapping Detection, which uses IMU data to identify unexpected movements suggesting the robot has been moved
2. Relocalization, employing OpenCV for visual SLAM to reorient the robot in its new environment. The approach combines threshold-based 
- IMU analysis with feature detection and mapping, offering a robust solution for autonomous reorientation after displacement.
# How to use it
1. cd catkin_ws/src #wherever your main is

2. git clone https://github.com/AlpMercan/Where-AM-I.git
3. cd ~catkin_ws
4. rosdep install --from-paths src --ignore-src -r
5 catkin_make
# some configurations
- Use the correct imu topic.
- Change your map.yaml's origin to [0,0,0].
- The global costmap and local cost map should be in the same directory
- First enter the The_kidnapper_Algorithm.py and change image path and template path to your respective position
- From map class change the global cost map topic with its name and after work the system 2 times. 
- After you have initial costmaps change back to local costmaps.
- You can add the kidnapper.launch to your main launch file for it to work continously.
- As long as it does not detect kidnapping, the main program will not work so CPU usage is respectively low

<img width="410" alt="Real_world_position" src="https://github.com/AlpMercan/Where-AM-I/assets/112685013/9dbdffa2-be5c-41f6-a404-d8b6b5ef8e12">

The real world position

<img width="419" alt="Initial_AMCL_Position" src="https://github.com/AlpMercan/Where-AM-I/assets/112685013/39691656-8e26-4f2d-ba38-d329e0c07635">

Rviz initial condition

![Position_detection](https://github.com/AlpMercan/Where-AM-I/assets/112685013/dc6689fb-72ef-4835-ab7a-91f015fff501)

The localition finder

<img width="380" alt="Relocalization" src="https://github.com/AlpMercan/Where-AM-I/assets/112685013/b59f3118-2257-4ce2-bf6a-9cdede504c8e">

relocalized position
