import pyzed.sl as sl
import rclpy
from rclpy.node import Node

class ZedCameraNode(Node):

    def __init__(self):
        super().__init__('zed_camera_node')

        # Declare and get parameters from ROS 2
        self.declare_parameter('camera_resolution', 'AUTO')
        self.declare_parameter('camera_fps', 30)

        camera_resolution_param = self.get_parameter('camera_resolution').get_parameter_value().string_value
        camera_fps_param = self.get_parameter('camera_fps').get_parameter_value().integer_value

        # Create a Camera object
        zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()

        # Set camera resolution
        if camera_resolution_param == 'HD720':
            init_params.camera_resolution = sl.RESOLUTION.HD720
        elif camera_resolution_param == 'HD1080':
            init_params.camera_resolution = sl.RESOLUTION.HD1080
        else:
            init_params.camera_resolution = sl.RESOLUTION.AUTO

        # Set FPS
        init_params.camera_fps = camera_fps_param

        # Open the camera
        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Camera Open Error: {err}. Exiting.")
            exit()

        # Capture 50 frames and stop
        i = 0
        image = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        while i < 50:
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                # A new image is available if grab() returns SUCCESS
                zed.retrieve_image(image, sl.VIEW.LEFT)
                timestamp = zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)  # Get the timestamp at the time the image was captured
                self.get_logger().info(f"Image resolution: {image.get_width()} x {image.get_height()} || Image timestamp: {timestamp.get_milliseconds()}")
                i += 1

        # Close the camera
        zed.close()


def main(args=None):
    rclpy.init(args=args)
    zed_camera_node = ZedCameraNode()

    try:
        rclpy.spin(zed_camera_node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    zed_camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
