
# Python Dependencies
import cv2
import numpy as np
from typing import List, Dict, Tuple
from ultralytics import YOLO
import autonomy.src.types  as types
from dataclasses import dataclass

# ROS Dependencies
import rospy
from sensor_msgs.msg import Image, CameraInfo, RegionOfInterest
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Header
from autonomy.msg import Detection, Detections  
from cv_bridge import CvBridge, CvBridgeError


@dataclass
class ColorFilterConfig:
    tolerance: float
    min_confidence: float
    min_area: float
    rgb_range: Tuple[int,int,int]

#//////////////////////////////////////////// 
#//////////DETECTION FUNCTIONS///////////////
#////////////////////////////////////////////

def color_filter(image: np.ndarray, config: ColorFilterConfig = ColorFilterConfig(
                     tolerance=0.4, min_confidence=0.3, min_area=0.2, rgb_range= (255,0,0)
                 )) -> List[types.Detection]:
    assert config.min_confidence < 1.0 and config.min_confidence > 0, "Min confidence must be between 0 and 1"

    result = []
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

    # Convert the RGB color to HSV for filtering
    target_hsv = cv2.cvtColor(np.uint8([[config.rgb_range]]), cv2.COLOR_RGB2HSV)[0][0]
    lower_bound = np.array([max(0, target_hsv[0] - config.tolerance * 180), 100, 100], dtype=np.uint8)
    upper_bound = np.array([min(180, target_hsv[0] + config.tolerance * 180), 255, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > config.min_area:
            x, y, w, h = cv2.boundingRect(contour)
            contour_mask = np.zeros(mask.shape, np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)

            # Calculate the confidence based on color presence within the contour
            red_pixels_in_contour = cv2.bitwise_and(mask, mask, mask=contour_mask)
            red_pixel_count = np.sum(red_pixels_in_contour == 255)
            total_pixels_in_contour = w * h

            if total_pixels_in_contour > 0:
                confidence = red_pixel_count / total_pixels_in_contour
                if confidence > config.min_confidence:
                    result.append(types.Detection(x, y, x + w, y + h, 0, confidence))

    return result


def yolo_object_detection(image: np.ndarray)-> List[types.Detection]:
    result_list = []
    model = YOLO("yolo11n.pt")  
    results = model(image) 
    for box in results.boxes:
        x1, y1, x2, y2 = box.xyxy.cpu().numpy()[0]
        conf = box.conf
        cls = box.cls
        result_list.append(types.Detection(x1,y1,x2,y2,str(cls), conf))

    return  result_list






#//////////////////////////////////////////// 
#/// IN PLACE MODIFICATION OF DETECTION//////
# ///////////////////////////////////////////

def calculate_distance(image:np.ndarray, bbox: List[types.Detection], object_heights:Dict[str: float], focal_length:float = 0):
    for box in bbox:    
        for key in object_heights.keys():
            distance_feet = (object_heights[key] * focal_length) / (box.y2 - box.y1)
            distance_feet = round(distance_feet, 2)  
            box.depth = distance_feet


def calculate_point_3d(detections: List[types.Detection], depth_image: np.ndarray, camera_intrinsic: tuple):
    # Calculate the average depth within the bounding box
    for detection in detections:
        x_min, y_min, x_max, y_max = detection.x1, detection.y1, detection.x2, detection.y2
        if depth_image is not None:
            x_min_int = int(x_min)
            x_max_int = int(x_max)
            y_min_int = int(y_min)
            y_max_int = int(y_max)

            bbox_depth = depth_image[y_min_int:y_max_int, x_min_int:x_max_int]
            if bbox_depth.size > 0:
                mean_depth = np.nanmean(bbox_depth)
                if not np.isnan(mean_depth):
                    # Convert depth to 3D point using camera intrinsic parameters
                    fx = camera_intrinsic[0]
                    fy = camera_intrinsic[1]
                    cx = camera_intrinsic[2]
                    cy = camera_intrinsic[3]

                    z = mean_depth
                    x_center = (x_min + x_max) / 2
                    y_center = (y_min + y_max) / 2
                    x = (x_center - cx) * z / fx
                    y = (y_center - cy) * z / fy

                    point_camera = Point(x=x, y=y, z=z)
                    

def transform_to_global(detections: List[types.Detection],imu_point: types.Point3D, imu_rotation:types.Point3D):
    translation = [imu_point.x, imu_point.y, imu_point.z]
    rotation = [imu_rotation.a, imu_rotation.b,imu_rotation.c, imu_rotation.d]

    def quaternion_to_matrix(rotation: types.Rotation3D):
        w, x, y, z = rotation.a, rotation.b, rotation.c, rotation.d
        # Define the rotation matrix from quaternion
        rotation_matrix = np.array([
            [1 - 2*y**2 - 2*z**2,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
        ])
        # Create a 4x4 transformation matrix
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        return transform_matrix

    transform_matrix = quaternion_to_matrix(rotation)
    transform_matrix[0:3, 3] = translation
    
    for detection in detections:    
        point_homogeneous = np.array([detection.point.x, detection.point.y, detection.point.z, 1.0])
        point_global = np.dot(transform_matrix, point_homogeneous)
        detection.point = types.Point3D(point_global[0], point_global[1], point_global[2])



#//////////////////////////////////////////// 
#///////// ROS CODE PUBLISHERS///////////////
# ///////////////////////////////////////////

bridge = CvBridge()
rgb_image: np.ndarray = None
depth_image: np.ndarray = None
imu_point: types.Point3D = None
imu_rotation: types.Rotation3D = None
camera_intrinsics: tuple = None
camera_info = None
color_filter_config: ColorFilterConfig = ColorFilterConfig(tolerance=0.4, min_confidence=0.3, min_area=0.2, rgb_range= (255,0,0))

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
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def camera_info_callback(msg):
    global camera_intrinsics
    fx = msg.K[0] 
    fy = msg.K[4] 
    cx = msg.K[2] 
    cy = msg.K[5] 
    camera_intrinsics = (fx,fy,cx,cy)

def imu_pose_callback(msg):
    global imu_point, imu_rotation
    imu_point = types.Point3D(x=msg.position.x, y=msg.position.y, z=msg.position.z)
    imu_rotation = types.Rotation3D(x = msg.orientation.x, y = msg.orientation.y, z=msg.orientation.z, w=msg.orientation.w)



def create_detector_message(detected_object: List[types.Detection])-> List[Detection]:
    detections_conversiton: List[Detection] = []
    for detection in detected_object:
        detector_msg = Detection()
        bbox = RegionOfInterest(
        x_offset=detection.x1,
        y_offset=detection.y1,
        height=detection.y2 - detection.y1,
        width=detection.x2 - detection.x1)
        point = Point(x=detection.point.x ,y=detection.point.y, z=detection.point.z)

        detector_msg.cls = detected_object.cls
        detector_msg.confidence = detected_object.conf
        detector_msg.point = point
        detector_msg.bounding_box = bbox
        detections_conversiton.append(detector_msg)
    return detections_conversiton



def run_detection_pipelines()->List[Tuple[str,List[Detection]]]:
    global rgb_image, depth_image, camera_intrinsics,imu_point, imu_rotation, color_filter_config
    detectors_output = []
    pipelines = [color_filter, yolo_object_detection]
    detectors_names = ['color_detector', 'yolo_detector']
    for name in range(detectors_names):
        if name == 'color_detector':
            detection_results = pipelines[i](rgb_image, color_filter_config)  
        else:
            detection_results = pipelines[i](rgb_image, color_filter_config)  
        calculate_point_3d(detections=detection_results, depth_image=depth_image, camera_intrinsic= camera_intrinsics)
        transform_to_global(detections=detection_results, imu_point=imu_point,imu_rotation=imu_rotation)
        detector_msg_list = create_detector_message(create_detector_message(detection_results))
        detectors_output.append((detectors_names[i], detector_msg_list))

    return detectors_output

def publish_vision_detections():
    rospy.init_node('computer_vision_detections')
    detection_pub = rospy.Publisher('/detector/box_detection', Detections, queue_size=10)
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        pipelines_results = run_detection_pipelines()
        for detector_name, detections in pipelines_results:
            detection_msg = Detections()
            detection_msg.detections = detections
            detection_msg.detector_name = detector_name  
            detection_pub.publish(detection_msg)

        rate.sleep()



def handle_set_color_filter(req):
    global color_filter_config
    color_filter_config = ColorFilterConfig(
        tolerance=req.tolerance,
        min_confidence=req.min_confidence,
        min_area=req.min_area,
        rgb_range=tuple(req.rgb_range)
    )
    return SetColorFilterResponse(success=True, message="Color filter config updated.")







def initialize_subscribers():
    rospy.loginfo("Initializing subscribers...")
    rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color", Image, rgb_image_callback)
    rospy.Subscriber("/zed2i/zed_node/depth/depth_registered", Image, depth_image_callback)
    rospy.Subscriber("/zed2i/zed_node/rgb/camera_info", CameraInfo, camera_info_callback)
    rospy.Subscriber("/zed2i/zed_node/pose", PoseStamped, imu_pose_callback)
    rospy.sleep(5)

if __name__ == "__main__":
    initialize_subscribers()
    publish_vision_detections()
    rospy.spin()
