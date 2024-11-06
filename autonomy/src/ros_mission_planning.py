#  The idea of this code is to perform a llop that calculates the sstuuuffssss

import rospy
import yaml
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math
import numpy as np
from dataclasses import dataclass
from types import point_3d


def pose_reached( first_coordinate: Point3D, second_coordinate: Point3D, threshold: int):
    position_diff = math.sqrt(
            (first_coordinate[0] - second_coordinate[0]) ** 2 +
            (first_coordinate[1] - second_coordinate[1]) ** 2 +
            (first_coordinate[2] - second_coordinate[2]) ** 2)
    return position_diff <= threshold 



     
    

def get_ros_data(topics_file: str):
    detections = []


class Mission:
    def __init__(self):
        self.checkpoints = []


class PreQualificationMission:
    def __init__(self):
        self.checkpoints = []
        self.gate_position = None
        self.buoy_position = None

def search_objects():
    for i in range
    

def main():
    mission_completed = False
    timeout_duration = 5
    threshold = 5
    start_time = 0
    initial_data = get_ros_data()
    while not rospy.is_shutdown() and rospy.Time.now().secs - start_time < timeout_duration:
        if pose_reached(shared_data.zed_data["pose"], target_point, threshold):
            rospy.loginfo("Destination reached. Verifying stabilization.")
            stabilization_start = rospy.Time.now()
            while rospy.Time.now() - stabilization_start < rospy.Duration(0.5):
                if not pose_reached(shared_data.zed_data["pose"], target_point, threshold):
                    break
                rospy.sleep(0.1)
            else: 
                rospy.loginfo("Destination stabilized.")
                return 'success'
        rospy.sleep(0.1)
        print("Monitoring loop")

        




if __name__ == "__main__":
    pass