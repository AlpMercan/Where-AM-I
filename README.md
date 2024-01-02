# Where-AM-I
Open_CV based solution to kidnapped robot problem. It will detect kidnapping from IMU and the relocalize itself.
# How to use it
cd catkin_ws/src #wherever your main is
git clone https://github.com/AlpMercan/Where-AM-I.git
cd ~catkin_ws
rosdep install --from-paths src --ignore-src -r
catkin_make
#some configurations
#Use the correct imu topic
#change your map.yaml's origin to [0,0,0]
#the global costmap and local cost map should be in the same directory
#first enter the The_kidnapper_Algorithm.py and change image path and template path to your respective position
#from map class change the global cost map topic with its name and after work the system 2 times. 
#After you have initial costmaps change back to local costmaps.
#You can add the kidnapper.launch to your main launch file for it to work continously.
#As long as it does not detect kidnapping, the main program will not work so CPU usage is respectively low
<img width="410" alt="Real_world_position" src="https://github.com/AlpMercan/Where-AM-I/assets/112685013/9dbdffa2-be5c-41f6-a404-d8b6b5ef8e12">
The real world position
<img width="419" alt="Initial_AMCL_Position" src="https://github.com/AlpMercan/Where-AM-I/assets/112685013/39691656-8e26-4f2d-ba38-d329e0c07635">
Rviz initial condition
![Position_detection](https://github.com/AlpMercan/Where-AM-I/assets/112685013/dc6689fb-72ef-4835-ab7a-91f015fff501)
The localition finder
<img width="380" alt="Relocalization" src="https://github.com/AlpMercan/Where-AM-I/assets/112685013/b59f3118-2257-4ce2-bf6a-9cdede504c8e">
relocalized position
