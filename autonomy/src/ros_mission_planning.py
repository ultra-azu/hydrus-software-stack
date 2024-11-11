import math
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Callable, Set

# ROS Dependencies
import rospy
import actionlib
from geometry_msgs.msg import Point, PoseStamped
from autonomy.msg import Detection, Detections  
from autonomy.msg import NavigateToWaypointAction, NavigateToWaypointGoal, NavigateToWaypointFeedback, NavigateToWaypointResult

@dataclass
class MissionObject:
    position:Point
    object_name: str
    distance: float

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
    object_cls: str
    conditions: Callable
    status: Status = Status.NONCALLED






class PreQualificationMission:
    def __init__(self):

        #///////////////////////////////
        # ///////CONFIGURATION//////////
        # //////////////////////////////
        self.cls_names = {1: 'Gate', 2: 'Bouy'}
        self.instructions = deque()
        # Define mission instructions
        self.instructions.extend([
            MissionInstructions(task=TaskType.MOVETOCENTER, object_cls="Gate", conditions=lambda: True),
            MissionInstructions(task=TaskType.MOVEAROUND, object_name="Buoy", conditions=lambda: True),
            MissionInstructions(task=TaskType.MOVETOCENTER, object_name="Gate", conditions=lambda: True)
        ])



        # //////////////////////////////
        # ////////MUTABLE DATA//////////
        # //////////////////////////////
        self.current_instruction: Optional[MissionInstructions] = None
        self.mission_objects: Set[MissionObject] = set()
        self.detected_objects: Set[str] = set()
        self.submarine_pose : PoseStamped = None
        self.current_detections: Detections = None


        # ///////////////////////////////
        # //////INIT ROS DATA ///////////
        # ///////////////////////////////
        def detection_callback(msg):
            self.current_detections = msg
        rospy.Subscriber("/detector/box_detection", Detections, detection_callback)
        def pose_callback(msg):
            self.submarine_pose =  msg
        rospy.Subscriber("/zed2i/zed_node/pose", PoseStamped, pose_callback)
        self.client = actionlib.SimpleActionClient('navigate_to_waypoint', NavigateToWaypointAction)
        
        rospy.loginfo("Waiting for action server to start...")
        self.client.wait_for_server()
        rospy.loginfo("Action server started.")


        self.calculate_distance = lambda pos1, pos2: math.sqrt((pos1.x - pos2.x)**2 +(pos1.y - pos2.y)**2+ (pos1.z - pos2.x)**2 )

    def search_mission_objet(self, object_name: str):
        for item in self.mission_objects:
            if item.object_name == object_name:
                return item
        return None


    def execute_task(self, task: MissionInstructions):

        if task.object_cls in self.detected_objects:
            mission_object = self.search_mission_objet(task.object_cls)



        # Send goal to action server
        goal = NavigateToWaypointGoal(mission_object.position)
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
        detector_name = self.current_detections.detector_name
        detections = self.current_detections
        for detection in detections:
            if self.cls_names[detection.cls] == "Gate" and self.cls_names[detection.cls] not in self.detected_objects:
                self.detected_objects.add("Gate")
                distance = self.calculate_distance(detection.point, self.submarine_pose.position)
                self.mission_objects.append(MissionObject(detection.point,"Gate", distance=distance)) 
            if self.cls_names[detection.cls] == "Buoy" and self.cls_names[detection.cls] not in self.detected_objects:
                self.detected_objects.add("Buoy") 
                distance = self.calculate_distance(detection.point, self.submarine_pose.position)
                self.mission_objects.append(MissionObject(detection))            

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
