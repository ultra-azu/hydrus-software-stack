#!/usr/bin/env python3

import math
import yaml
from controller.srv import NavigateToWaypoint, NavigateToWaypointResponse, SetParameters, SetParametersResponse
import os
from types import point_3d, rotation_3d



class SubController:
    def __init__(self):
        self.DEPTH_SPEED = 2
        self.ROTATION_SPEED = 1
        self.LINEAR_SPEED = 2
        self.FRONT_TORPEDO_SPEED = 5
        self.BACK_TORPEDO_SPEED = 4
        self.DELTA = 0.01

        self.speed_translation = {
            1: 1550,
            2: 1600,
            3: 1650,
            4: 1700,
            5: 1300,
            6: 1400,
            7: 1350
        }

        self.DEPTH_MOTORS_ID = [5, 6]  # front left, front right, back left, back right
        self.FRONT_MOTORS_ID = [3, 4]  # left, right
        self.BACK_MOTORS_ID = [1, 2]  # left, right
        self.TORPEDO_MOTORS_ID = [7, 8]

        self.target_point = None
        self.current_pose = 
        self.current_pose.position = None
        self.thrusters_publishers = []
        self.thruster_values = [Vector3() for _ in range(8)]
        self.moving = [False, False, False]  # [depth, rotation, linear]

        for i in range(8):
            self.thrusters_publishers.append(rospy.Publisher('/hydrus/thrusters/' + str(i+1), Vector3, queue_size=10))

            if self.target_point is not None:
                self.move_submarine(self.current_pose.position, self.target_point)


    @staticmethod
    def move_submarine(self, current_pose, target_point):
        if current_pose is None or target_point is None:
            return

        if self.moving[0]:  # Go up or down
            if abs(current_pose.position.z - target_point.z) > self.DELTA:
                if (current_pose.position.z - target_point.z) > self.DELTA:
                    for motor_id in self.DEPTH_MOTORS_ID:
                        if motor_id == 5:
                          self.thruster_values[motor_id] = Vector3(x=1400, y=0, z=0)
                        else:
                          self.thruster_values[motor_id] = Vector3(x=self.speed_translation[self.DEPTH_SPEED], y=0, z=0)
                else:
                    for motor_id in self.DEPTH_MOTORS_ID:
                        if motor_id == 5:
                          self.thruster_values[motor_id] = Vector3(x=-1400, y=0, z=0)
                        else:
                          self.thruster_values[motor_id] = Vector3(x=-self.speed_translation[self.DEPTH_SPEED], y=0, z=0)
            else:
                self.moving = [False, True, False]
        elif self.moving[1]:  # Rotate to face the target point
            target_yaw = self.calculate_yaw_to_target(current_pose.position, target_point)
            current_yaw = self.calculate_current_yaw(current_pose.position.orientation)
            angle_diff = self.normalize_angle(target_yaw - current_yaw)

            if abs(angle_diff) > self.DELTA:
                if angle_diff > 0:
                    self.thruster_values[self.FRONT_MOTORS_ID[0]] = Vector3(x=-self.speed_translation[7], y=0, z=0)
                    self.thruster_values[self.FRONT_MOTORS_ID[1]] = Vector3(x=self.speed_translation[self.ROTATION_SPEED], y=0, z=0)
                    self.thruster_values[self.BACK_MOTORS_ID[0]] = Vector3(x=-self.speed_translation[self.ROTATION_SPEED], y=0, z=0)
                    self.thruster_values[self.BACK_MOTORS_ID[1]] = Vector3(x=self.speed_translation[self.ROTATION_SPEED], y=0, z=0)
                else:
                    self.thruster_values[self.FRONT_MOTORS_ID[0]] = Vector3(x=self.speed_translation[self.ROTATION_SPEED], y=0, z=0)
                    self.thruster_values[self.FRONT_MOTORS_ID[1]] = Vector3(x=-self.speed_translation[self.ROTATION_SPEED], y=0, z=0)
                    self.thruster_values[self.BACK_MOTORS_ID[0]] = Vector3(x=self.speed_translation[self.ROTATION_SPEED], y=0, z=0)
                    self.thruster_values[self.BACK_MOTORS_ID[1]] = Vector3(x=-self.speed_translation[self.ROTATION_SPEED], y=0, z=0)
            else:
                self.moving = [False, False, True]
        elif self.moving[2]:  # Move forward
            if abs(current_pose.position.x - target_point.x) > self.DELTA or abs(current_pose.position.y - target_point.y) > self.DELTA:
                for motor_id in self.FRONT_MOTORS_ID:
                    if motor_id == 3:
                      self.thruster_values[motor_id] = Vector3(x=self.speed_translation[6], y=0, z=0)
                    else:
                      self.thruster_values[motor_id] = Vector3(x=self.speed_translation[self.LINEAR_SPEED], y=0, z=0)
                for motor_id in self.BACK_MOTORS_ID:
                    self.thruster_values[motor_id] = Vector3(x=self.speed_translation[self.LINEAR_SPEED], y=0, z=0)
            else:
                self.moving = [True, False, False]
        else:
            self.moving = [True, False, False]

        for i in range(len(self.thruster_values)):
            self.thrusters_publishers[i].publish(self.thruster_values[i])
        self.rate.sleep()
    @staticmethod
    def calculate_yaw_to_target(self, current_position, target_point):
        dx = target_point.x - current_position.x
        dy = target_point.y - current_position.y
        return math.atan2(dy, dx)
    @staticmethod
    def calculate_current_yaw(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)
    @staticmethod
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle




if __name__ == '__main__':
    try:
        controller = SubController()
    except rospy.ROSInterruptException:
        pass


