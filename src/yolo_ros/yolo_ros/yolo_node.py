import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    BoundingBox2D,
    ObjectHypothesisWithPose,
)
from geometry_msgs.msg import PoseWithCovariance
from cv_bridge import CvBridge       # <— add this line
from ultralytics import YOLO
from collections import deque
from threading import Thread, Lock
from time import sleep

class YoloRosNode(Node):
    def __init__(self):
        super().__init__('yolo_ros_node')
        self.bridge = CvBridge()

        # THREAD-SAFE QUEUE FOR FRAMES
        self.frame_queue = deque(maxlen=2)   # keep only up to 2 frames; drop older
        self.queue_lock = Lock()

        # Subscribers (just push into queue, don’t run inference here)
        self.image_sub = self.create_subscription(
            Image, '/camera/rgbd/image_raw', self.image_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/rgbd/camera_info', self.caminfo_callback, 10
        )

        # Publishers (we will publish from the worker thread)
        self.image_pub = self.create_publisher(Image, '/yolo/image', 10)
        self.info_pub  = self.create_publisher(CameraInfo, '/yolo/camera_info', 10)
        self.det_pub   = self.create_publisher(Detection2DArray, '/yolo/detections', 10)

        # LOAD YOLO MODEL ON GPU WITH FP16
        # Note: `device='cuda:0'` forces GPU; `model.model.half()` enables FP16.
        self.model = YOLO('/home/jay/Documents/yolo11-tutorial/runs/detect/train17/weights/best.pt')
        # 2) move weights onto GPU #0
        self.model.to('cuda:0')         # or self.model.cuda()

        # 3) convert to FP16
        self.model.model.half()

        # START A DEDICATED INFERENCE THREAD
        self.running = True
        self.infer_thread = Thread(target=self.inference_worker, daemon=True)
        self.infer_thread.start()

        self.get_logger().info("YOLO node up and running on CUDA with FP16!")

    def caminfo_callback(self, info_msg: CameraInfo):
        # Immediately forward camera info
        self.info_pub.publish(info_msg)

    def image_callback(self, img_msg: Image):
        # 1) Convert ROS‐Image to CV2
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # 2) Push the raw CV image + header into our queue
        with self.queue_lock:
            # Store a tuple (image, header); maxlen=2 means if queue is full, oldest is discarded
            self.frame_queue.append((cv_img, img_msg.header))

    def inference_worker(self):
        """
        Continuously pull the most recent frame from the queue,
        run YOLO inference (FP16) on it, then publish annotated image + detections.
        """
        from time import sleep

        while rclpy.ok() and self.running:
            frame_item = None

            # Pop the newest frame (if any)
            with self.queue_lock:
                if self.frame_queue:
                    frame_item = self.frame_queue.pop()
                    # Clear the queue if there was more than one, since we only want most recent
                    self.frame_queue.clear()

            if frame_item is None:
                # No frame available—sleep for a few milliseconds and retry
                sleep(0.005)
                continue

            cv_img, header = frame_item

            # 3) Run inference (FP16 on GPU). This is synchronous, but it's in our worker thread.
            #    Because we set model to FP16 and device='cuda:0', this call will use more GPU resources.
            results = self.model(cv_img, imgsz=640, half=True)  # imgsz=640 is common; adjust as needed

            # 4) Annotate & publish image
            annotated = results[0].plot()  # still returns uint8 BGR on CPU, but inference was fp16 on GPU
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = header
            self.image_pub.publish(out_msg)

            # 5) Build & publish Detection2DArray
            det_arr = Detection2DArray()
            det_arr.header = header
            for box in results[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                w = x2 - x1
                h = y2 - y1

                det = Detection2D()
                det.header = header

                # BBox center & size
                bb = BoundingBox2D()
                bb.center.position.x = float((x1 + x2) / 2.0)
                bb.center.position.y = float((y1 + y2) / 2.0)
                bb.size_x = float(w)
                bb.size_y = float(h)
                det.bbox = bb

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(int(box.cls[0].item()))
                hyp.hypothesis.score = float(box.conf[0].item())
                hyp.pose = PoseWithCovariance()

                det.results = [hyp]
                det_arr.detections.append(det)

            self.det_pub.publish(det_arr)

        self.get_logger().info("Inference worker shutting down.")

    def destroy_node(self):
        # Stop the worker thread cleanly before shutting down
        self.running = False
        self.infer_thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloRosNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()