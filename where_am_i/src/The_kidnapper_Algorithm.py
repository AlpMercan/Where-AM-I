#!/usr/bin/env python3

import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
import numpy as np
import cv2
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
import threading

class Matching:
    @staticmethod
    def publish_transform(center_x, center_y):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        # Convert pixel coordinates to map coordinates here
        map_x, map_y = Matching.pixel_to_map_conversion(center_x, center_y)

        # Set the position
        t.transform.translation.x = map_x
        t.transform.translation.y = map_y
        t.transform.translation.z = 0.0

        # Set the orientation
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)

    @staticmethod
    def pixel_to_map_conversion(pixel_x, pixel_y):
        return -pixel_x * 0.05, -pixel_y * 0.05  # pixel to real life measurement (resolution)
    
    @staticmethod
    def publish_initial_pose(x, y, theta):
        initial_pose_pub = rospy.Publisher('initialpose', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)
        rospy.sleep(1)  # Give time for the publisher to set up

        initial_pose = geometry_msgs.msg.PoseWithCovarianceStamped()
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id = "map"

        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.position.z = 0

        quaternion = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
        initial_pose.pose.pose.orientation.x = quaternion[0]
        initial_pose.pose.pose.orientation.y = quaternion[1]
        initial_pose.pose.pose.orientation.z = quaternion[2]
        initial_pose.pose.pose.orientation.w = quaternion[3]

        initial_pose_pub.publish(initial_pose)

    @staticmethod
    def match_template(image_path, template_path):
        bg = cv2.imread(image_path)
        bg = cv2.cvtColor(bg, cv2.COLOR_BGR2RGB)
        face = cv2.imread(template_path)
        face = cv2.cvtColor(face, cv2.COLOR_BGR2RGB)
        
        height, width, channels = face.shape
        methods = {"cv2.TM_CCOEFF": cv2.TM_CCOEFF}

        for method_name, method in methods.items():
            bg_copy = bg.copy()
            result = cv2.matchTemplate(bg_copy, face, method)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

            if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
                top_left = min_loc
            else:
                top_left = max_loc

            center_x1 = top_left[0] + width // 2
            center_y1 = top_left[1] + height // 2
            center_x = abs((-center_x1))*0.05
            center_y = abs((-center_y1))*0.05
            bottom_right = (top_left[0] + width, top_left[1] + height)

            # Publish the transform and initial pose
            #publish_transform(center_x, center_y)
            Matching.publish_initial_pose(center_x, center_y, 0)  
            cv2.rectangle(bg_copy, top_left, bottom_right, 255, 10)

        # Plot the Images for debugging reason, you can delete it if you want
            plt.subplot(121)
            plt.imshow(result)
            plt.title("Result of Template Matching or photo matching")

            plt.subplot(122)
            plt.imshow(bg_copy)
            plt.title("Match Point")
            plt.show()


class Map:
    @staticmethod
    def map_value_to_color(value):
        if value == -1:
        # Unknown or unexplored
            return (128, 128, 128)  # Gray
        elif value <= 30:
        # Free space (0-30)
            return (255, 255, 255)  # White
        elif value <= 70:
            # Unknown space (30-70)
            return (0, 0, 255)  # Red
        else:
        # Occupied space (70-100)
            return (0, 255, 0)  # Green
    @staticmethod
    def callback(data):
    
        width = height = int(np.sqrt(len(data.data)))
        data_array = np.array(data.data, dtype=np.int8).reshape((height, width))

    
        color_image = np.array([Map.map_value_to_color(val) for val in data_array.flat], dtype=np.uint8).reshape((height, width, 3))

    
        cv2.imwrite("/home/alp/catkin_ws/src/nerdeyim/turtlebot3/where_am_i/src/local_colored.png", color_image)######### change it to your folder
        
        #rospy.sleep(5)

    @staticmethod
    def listener():
        #rospy.init_node('costmap_to_png_node', anonymous=True)
        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, Map.callback) #ensure that you have that cost map
        rospy.spin()

# Main loop
if __name__ == '__main__':
    rospy.init_node('Kidnapping_Preventation_Node', anonymous=True)
    listener_thread = threading.Thread(target=Map.listener)
    listener_thread.start()
    #to ensure that local costmap is written
    rospy.sleep(1)
    image_path = "/home/alp/catkin_ws/src/nerdeyim/turtlebot3/where_am_i/src/GLOBAL.png" ######### change it to your folder
    template_path = "/home/alp/catkin_ws/src/nerdeyim/turtlebot3/where_am_i/src/local_colored.png" ######### change it to your folder
    Localization = Matching()
    #Map.listener()

    try:
        Localization.match_template(image_path, template_path)
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.signal_shutdown('Task completed')  

