import rospy
from autonomy.src.cv_algorithms import color_filter, yolo_object_detection
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tf.transformations as tft
import yaml
import os

bridge = CvBridge()
rgb_image = None
depth_image = None
camera_info = None
imu_pose = None



def depth_image_callback(msg):
    global depth_image
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, "32FC1")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def rgb_image_callback(msg):
    global rgb_image
    try:
        rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def camera_info_callback(msg):
    global camera_info
    camera_info = msg

def imu_pose_callback(msg):
    global imu_pose
    imu_pose = msg


def publish_vision_detections():
    rospy.init_node('computer_vision_detections')
    detection_pub = rospy.Publisher('/detector/box_detection', Detections, queue_size=10)
    annotated_pub = rospy.Publisher('annotated_image', Image, queue_size=10)
    pipelines = [color_filter,yolo_object_detection]
    for pipeline in pipelines:
        detections = pipelines()


def init_controllers_services_publishers():
    rospy.init_node('controller', anonymous=True)
    thrusters_publishers = []
    thrusters_publishers.append(rospy.Publisher('/hydrus/thrusters/' + str(i+1), Vector3, queue_size=10))
    navigate_service = rospy.Service('navigate_to_waypoint', NavigateToWaypoint, handle_navigate_request)
    set_parameters_service = rospy.Service('set_parameters', SetParameters, handle_set_parameters)




def read_yaml_file(file_path):
    with open(file_path, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            rospy.logerr(exc)
            return None
def initialize_subscribers(topics_file):
    topics_info = read_yaml_file(topics_file)
    if topics_info is None:
        rospy.logerr("Failed to read YAML file or file is empty.")
        return
    rospy.loginfo("YAML file read successfully.")
    rospy.Subscriber(topics_info['zed_camera']['image'], Image, rgb_image_callback)
    rospy.Subscriber(topics_info['zed_camera']['depth_image'], Image, depth_image_callback)
    rospy.Subscriber(topics_info['zed_camera']['camera_info'], CameraInfo, camera_info_callback)
    rospy.Subscriber(topics_info['zed_camera']['imu_pose'], PoseStamped, imu_pose_callback)
    rospy.sleep(5)

if __name__ == "__main__":

    initialize_subscribers("")
    publish_vision_detections()
    publish


    rospy.spin()
