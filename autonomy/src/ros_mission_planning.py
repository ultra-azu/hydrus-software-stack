import math
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Callable




# ROS Dependencies
import rospy
import actionlib
from geometry_msgs.msg import Point, PoseStamped
from autonomy.msg import Detection, Detections  
from autonomy.msg import NavigateToWaypointAction, NavigateToWaypointGoal, NavigateToWaypointFeedback, NavigateToWaypointResult



@dataclass
class MissionObject:
    name: str
    position: int

class TaskType(Enum):
    MOVETOCENTER = 1
    MOVEAROUND = 2

class Status(Enum):
    COMPLETED = 1
    ONPROGRESS = 2
    NONCALLED = 3

@dataclass
class MissionInstructions:
    task: TaskType
    object_name: str
    conditions: Callable
    status: Status = Status.NONCALLED

class PreQualificationMission:
    def __init__(self):
        self.cls_names = {1: 'Gate', 2: 'Bouy'}
        self.detected_objects = []
        self.current_position = None
        self.instructions = deque()
        self.current_instruction: Optional[MissionInstructions] = None
        self.client = actionlib.SimpleActionClient('navigate_to_waypoint', NavigateToWaypointAction)
        self.imu_sub = rospy.Subscriber("/zed2i/zed_node/pose", PoseStamped, imu_pose_callback)
        self.detections = rospy.Subscriber("/detector/box_detection", Detections, imu_pose_callback)
        
        rospy.loginfo("Waiting for action server to start...")
        self.client.wait_for_server()
        rospy.loginfo("Action server started.")
        
        # Define mission instructions
        self.instructions.extend([
            MissionInstructions(task=TaskType.MOVETOCENTER, object_name='Gate', conditions=lambda: True),
            MissionInstructions(task=TaskType.MOVEAROUND, object_name='Buoy', conditions=lambda: True),
            MissionInstructions(task=TaskType.MOVETOCENTER, object_name='Gate', conditions=lambda: True)
        ])

    def search_objects(self, data: MissionData):
        for detection in data.detections:
            if detection.cls == 'Gate' and not any(obj.name == 'Gate' for obj in self.detected_objects):
                self.detected_objects.append(MissionObject('Gate', position=detection.position))
            if detection.cls == 'Buoy' and not any(obj.name == 'Buoy' for obj in self.detected_objects):
                self.detected_objects.append(MissionObject('Buoy', position=detection.position))

    def execute_task(self, task: MissionInstructions):
        target = next((obj.position for obj in self.detected_objects if obj.name == task.object_name), None)
        if target is None:
            rospy.logwarn(f"Object '{task.object_name}' not found in detected objects.")
            return False

        # Send goal to action server
        goal = NavigateToWaypointGoal(x=target[0], y=target[1], z=target[2])
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)
        
        # Wait for the result
        self.client.wait_for_result()
        result = self.client.get_result()

        if result.success:
            rospy.loginfo(f"Successfully completed task: {task.task} for object: {task.object_name}")
            return True
        else:
            rospy.logwarn(f"Failed to complete task: {task.task} for object: {task.object_name}")
            return False

    def feedback_callback(self, feedback: NavigateToWaypointFeedback):
        rospy.loginfo(f"Distance to target: {feedback.distance_to_target}")


    def run(self):
        if not self.current_instruction and self.instructions:
            self.current_instruction = self.instructions.popleft()

        if self.current_instruction:
            if self.current_instruction.status == Status.NONCALLED:
                self.current_instruction.status = Status.ONPROGRESS
                rospy.loginfo(f"Starting task: {self.current_instruction.task} for object: {self.current_instruction.object_name}")

            # Execute the current instruction's task
            if self.current_instruction.status == Status.ONPROGRESS:
                success = self.execute_task(self.current_instruction)
                if success:
                    self.current_instruction.status = Status.COMPLETED
                    rospy.loginfo(f"Completed task: {self.current_instruction.task} for object: {self.current_instruction.object_name}")
                    self.current_instruction = None  # Move to the next instruction

def main():
    rospy.init_node('prequalification_mission_node')
    mission = PreQualificationMission()
    rate = rospy.Rate(1) 
    while not rospy.is_shutdown():
        mission.run()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
