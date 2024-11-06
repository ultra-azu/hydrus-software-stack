#!/usr/bin/env python

import cv2
import numpy as np
import ultralytics
from dataclasses import dataclass
from typing import List, Dict
from ultralytics import YOLO
from types import OutputBBox, point_3d,rotation_3d

def color_filter(image: np.ndarray, tolerance:float = 0.4, min_confidence:float = 0.3, min_area = 0.2, rgb_range: tuple = (255,0,0))-> List[OutputBBox]:
    assert min_confidence < 1.0 and min_confidence > 0 , "Min confidence is a percentage it must be between 0-1"

    result = []
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
    # Masks for color red
    lower_rgb1 = np.array([rgb_range[0], rgb_range[1], rgb_range[2]]) - tolerance
    upper_rgb1 = np.array([10 + tolerance, 255, 255])
    lower_rgb2 = np.array([170 - tolerance, 100, 100])
    upper_rgb2 = np.array([180, 255, 255])


    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask_red1 | mask_red2
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    object_count = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_area:
            object_count += 1
            x, y, w, h = cv2.boundingRect(contour)
            contour_mask = np.zeros(mask.shape, np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)

            red_pixels_in_contour = cv2.bitwise_and(mask, mask, mask=contour_mask)
            red_pixel_count = np.sum(red_pixels_in_contour == 255)
            total_pixels_in_contour = w * h

            if total_pixels_in_contour > 0:
                confidence = (red_pixel_count / total_pixels_in_contour) 
            if confidence > min_confidence:
                result.append(OutputBBox(x,y,x+w, y+h, 'null', confidence))

    return result            


def yolo_object_detection(image: np.ndarray)-> List[OutputBBox]:
    result_list = []
    model = YOLO("yolo11n.pt")  
    results = model(image) 
    for box in results.boxes:
        x1, y1, x2, y2 = box.xyxy.cpu().numpy()[0]
        conf = box.conf
        cls = box.cls
        result_list.append(OutputBBox(x1,y1,x2,y2,str(cls), conf))

    return  result_list

# In place modifification of the OutputBBox to add the depth.
def calculate_distance(image:np.ndarray, bbox: List[OutputBBox], object_heights:Dict[str: float], focal_length:float = 0):
    for box in bbox:    
        for key in object_heights.keys():
            distance_feet = (object_heights[key] * focal_length) / (box.y2 - box.y1)
            distance_feet = round(distance_feet, 2)  
            box.depth = distance_feet

    
#AI-GENERATED
def quaternion_to_matrix(rotation: rotation_3d):
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

def transform_to_global(camera_point: point_3d,imu_point: point_3d, imu_rotation:rotation_3d)-> point_3d:
    translation = [imu_point.x, imu_point.y, imu_point.z]
    rotation = [imu_rotation.a, imu_rotation.b,imu_rotation.c, imu_rotation.d]
    transform_matrix = quaternion_to_matrix(rotation)
    transform_matrix[0:3, 3] = translation

    point_homogeneous = np.array([camera_point.x, camera_point.y, camera_point.z, 1.0])
    point_global = np.dot(transform_matrix, point_homogeneous)
    return point_3d(point_global[0, point_global[1], point_global[2]])
