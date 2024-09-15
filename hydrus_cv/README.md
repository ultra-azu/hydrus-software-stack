
# Computer Vision  Package

## About

In this package, the Computer Vision Team is tasked with developing the Computer Vision Nodes for our autonomous submarine project.

## Why ROS?

In this module, we will utilize the Robot Operating System (ROS). ROS provides several advantages over building a custom backend API, such as real-time communication between modules and continuous data publishing. This eliminates the need for multiple HTTP requests between connected modules, making it more efficient. Additionally, ROS offers standardized message types designed for robotics, simplifying integration with other technologies in the system.

## Preliminary Ideas

###  Key Algorithms

- **Color Filters**: 
  filters objects based on color. This should account for color tolerance and return number of objects filtered, bounding boxes, and confidence levels.

- **Image Matching**:
  A node dedicated to image matching or template matching. Given an input image (template), this node should locate the object within a larger image or frame. It should return a bounding box around the matched object and a confidence score.

- **Optical Flow**:
  This node will track the movement of specific points across frames in a video. The output should include the tracked point’s position in each frame.

- **Video Object Segmentation**:
  This node will predict and track the segmentation mask of an object across a video. Outputs include the segmentation mask, bounding box, object ID, and potential reidentification of the object as it moves through different frames.

- **Object Detection**:
  A node for detecting objects in real time, classifying them into predefined categories. This will likely integrate popular object detection models like YOLO or SSD.

- **Orientation of Objects**:
  This node will return the orientation or rotation of a detected object.

- **Plane Detection**:
  A node that identifies planes (e.g., walls or floors) in the image frames.





### Installation Process


To install this package you will need to install ROS2, the distribution of ROS we are using (Please install ROS2 not ROS). The Distribution we are going to use is Jazzy.



Link to Install Jazzy.
