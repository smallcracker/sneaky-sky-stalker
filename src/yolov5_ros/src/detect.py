#!/usr/bin/env python3
import platform
import pathlib
plt = platform.system()
if plt != "Windows":
    pathlib.WindowsPath = pathlib.PosixPath
import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
from rostopic import get_topic_type

from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from detection_msgs.msg import BoundingBox, BoundingBoxes

# --- yolov5 子模块路径修正开始 ---
FILE = Path(__file__).resolve()
# detect.py 在 <workspace>/src/yolov5_ros/src/detect.py，yolov5 子模块在同级 src/yolov5
ROOT = FILE.parents[0] / "yolov5"
if not ROOT.exists():
    rospy.logerr(f"yolov5 path not found: {ROOT}")
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
# --- yolov5 子模块路径修正结束 ---

from models.common import DetectMultiBackend
from utils.general import (
    check_img_size,
    check_requirements,
    non_max_suppression,
    scale_coords
)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox

@torch.no_grad()
class Yolov5Detector:
    def __init__(self):
        # Detection parameters
        self.conf_thres = rospy.get_param("~confidence_threshold")
        self.iou_thres = rospy.get_param("~iou_threshold")
        self.agnostic_nms = rospy.get_param("~agnostic_nms")
        self.max_det = rospy.get_param("~maximum_detections")
        self.classes = rospy.get_param("~classes", None)
        self.line_thickness = rospy.get_param("~line_thickness")
        self.view_image = rospy.get_param("~view_image")

        # Model initialization
        weights = rospy.get_param("~weights")
        self.device = select_device(str(rospy.get_param("~device", "")))
        self.model = DetectMultiBackend(
            weights,
            device=self.device,
            dnn=rospy.get_param("~dnn"),
            data=rospy.get_param("~data")
        )
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
            self.model.stride,
            self.model.names,
            self.model.pt,
            self.model.jit,
            self.model.onnx,
            self.model.engine,
        )

        # Inference size and precision
        self.img_size = [rospy.get_param("~inference_size_w", 640), rospy.get_param("~inference_size_h", 480)]
        self.img_size = check_img_size(self.img_size, s=self.stride)
        self.half = rospy.get_param("~half", False) and (self.pt or self.jit or self.onnx or self.engine) and self.device.type != "cpu"
        if self.pt or self.jit:
            self.model.model.half() if self.half else self.model.model.float()
        cudnn.benchmark = True
        self.model.warmup()

        # Subscribers: color image
        input_type, input_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking=True)
        self.compressed_input = input_type == "sensor_msgs/CompressedImage"
        if self.compressed_input:
            rospy.Subscriber(input_topic, CompressedImage, self.callback, queue_size=1)
        else:
            rospy.Subscriber(input_topic, Image, self.callback, queue_size=1)

        # Publishers
        self.pred_pub = rospy.Publisher(rospy.get_param("~output_topic"), BoundingBoxes, queue_size=10)
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(rospy.get_param("~output_image_topic"), Image, queue_size=5)

        # Depth subscribers and intrinsics
        self.bridge = CvBridge()
        self.latest_depth = None
        self.fx = self.fy = self.cx = self.cy = None

        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_cb, queue_size=1)
        rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.info_cb, queue_size=1)

    def info_cb(self, msg: CameraInfo):
        if self.fx is None:
            K = np.array(msg.K).reshape(3, 3)
            self.fx, self.fy = K[0, 0], K[1, 1]
            self.cx, self.cy = K[0, 2], K[1, 2]
            rospy.loginfo(f"[Depth intrinsics] fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def depth_cb(self, msg: Image):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) * 0.001
        self.latest_depth = depth

    def callback(self, data):
        # Read color image
        if self.compressed_input:
            im = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        else:
            im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        im, im0 = self.preprocess(im)

        # Inference
        img = torch.from_numpy(im).to(self.device)
        img = img.half() if self.half else img.float()
        img /= 255
        if img.ndim == 3:
            img = img[None]

        pred = self.model(img, augment=False, visualize=False)
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)
        det = pred[0].cpu().numpy()

        # Prepare output message
        boxes_msg = BoundingBoxes()
        boxes_msg.header = data.header
        boxes_msg.image_header = data.header
        annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))

        if det.size:
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
            for *xyxy, conf, cls in reversed(det):
                bbox = BoundingBox()
                c = int(cls)
                bbox.Class = self.names[c]
                bbox.probability = float(conf)
                bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax = map(int, xyxy)

                # Compute depth at box center
                u = int((xyxy[0] + xyxy[2]) / 2)
                v = int((xyxy[1] + xyxy[3]) / 2)
                if self.latest_depth is not None and self.fx is not None:
                    Z = float(self.latest_depth[v, u])
                    if Z > 0 and not np.isnan(Z):
                        X = (u - self.cx) * Z / self.fx
                        Y = (v - self.cy) * Z / self.fy
                        bbox.x, bbox.y, bbox.z = X, Y, Z
                    else:
                        bbox.x = bbox.y = bbox.z = float('nan')

                boxes_msg.bounding_boxes.append(bbox)

                if self.publish_image or self.view_image:
                    label = f"{self.names[c]} {conf:.2f}"
                    annotator.box_label(xyxy, label, color=colors(c, True))

            im0 = annotator.result()

        # Publish results
        self.pred_pub.publish(boxes_msg)
        if self.view_image:
            cv2.imshow("det", im0)
            cv2.waitKey(1)
        if self.publish_image:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im0, "bgr8"))

    def preprocess(self, img):
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        img = img[..., ::-1].transpose((0, 3, 1, 2))
        return np.ascontiguousarray(img), img0


if __name__ == "__main__":
    check_requirements(exclude=("tensorboard", "thop"))
    rospy.init_node("yolov5", anonymous=True)
    detector = Yolov5Detector()
    rospy.spin()
