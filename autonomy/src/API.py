import uvicorn
import asyncio
import rospy
import cv2
import numpy as np
from fastapi import FastAPI
from fastapi.responses import StreamingResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from fastapi.middleware.cors import CORSMiddleware
from typing import List, Dict, Any
from cv_publishers import run_detection_pipelines, initialize_subscribers

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

bridge = CvBridge()
rgb_image = None  

def rgb_image_callback(msg):
    global rgb_image
    try:
        rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")

def generate_video_stream():
    while True:
        if rgb_image is not None:
            _, jpeg = cv2.imencode('.jpg', rgb_image)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        else:
            rospy.logwarn("No image received yet")
        asyncio.sleep(0.1)

@app.get("/video_feed")
def video_feed():
    return StreamingResponse(generate_video_stream(), media_type="multipart/x-mixed-replace; boundary=frame")

def convert_detections_to_dict(pipeline_results) -> List[Dict[str, Any]]:
    output = []
    for detector_name, detection_list in pipeline_results:
        converted_detections = []
        for det_msg in detection_list:
            converted_detections.append({
                "cls": det_msg.cls,
                "confidence": det_msg.confidence,
                "point": {
                    "x": det_msg.point.x,
                    "y": det_msg.point.y,
                    "z": det_msg.point.z
                },
                "bounding_box": {
                    "x_offset": det_msg.bounding_box.x_offset,
                    "y_offset": det_msg.bounding_box.y_offset,
                    "width":   det_msg.bounding_box.width,
                    "height":  det_msg.bounding_box.height
                }
            })
        output.append({
            "detector_name": detector_name,
            "detections": converted_detections
        })
    return output

async def sse_generator():
    while True:
        pipeline_results = run_detection_pipelines()
        data = convert_detections_to_dict(pipeline_results)
        import json
        json_str = json.dumps(data)
        yield f"data: {json_str}\n\n"
        await asyncio.sleep(0.1)

@app.get("/detections/stream")
def get_detections_stream():
    return StreamingResponse(sse_generator(), media_type="text/event-stream")

if __name__ == "__main__":
    rospy.init_node('cv_publisher', anonymous=True)
    rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color", Image, rgb_image_callback)
    initialize_subscribers()
    uvicorn.run(app, host="0.0.0.0", port=8000)