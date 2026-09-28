#!/usr/bin/env python3
"""인식 노드 - 노트북에서 실행 (로봇 roscore에 원격 접속).

로봇 카메라 영상을 구독해 YOLO로 추론하고 결과만 publish 한다.
cmd_vel은 만들지 않는다 - 그건 로봇의 controller_node 몫.

  roslaunch sign_light_driving detect.launch
"""
import os

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO

from sign_light_driving.msg import Detection, DetectionArray

CLASSES = ["left_turn", "right_turn", "red_light", "green_light"]


def main():
    rospy.init_node("detector")
    model_path = os.path.expanduser(
        rospy.get_param("~model", "~/tb_auto/runs/detect/runs/train/weights/best.pt"))
    model = YOLO(model_path)
    imgsz = rospy.get_param("~imgsz", 320)
    conf = rospy.get_param("~conf", 0.25)   # 최종 판단은 controller가 한다. 여기선 넉넉히.

    pub = rospy.Publisher("detections", DetectionArray, queue_size=1)
    bridge = CvBridge()

    # 첫 추론은 워밍업 때문에 수백 ms 걸린다. 미리 한 번 돌려서
    # 주행 시작 직후 controller 워치독이 헛발동하는 것을 막는다.
    model.predict(np.zeros((240, 320, 3), np.uint8), imgsz=imgsz, verbose=False)

    def on_image(msg):
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        h = frame.shape[0]
        r = model.predict(frame, imgsz=imgsz, conf=conf, verbose=False)[0]
        out = DetectionArray()
        out.header = msg.header
        for b in r.boxes:
            out.detections.append(Detection(
                label=CLASSES[int(b.cls)],
                confidence=float(b.conf),
                height_ratio=float(b.xywh[0][3]) / h))
        pub.publish(out)

    rospy.Subscriber(rospy.get_param("~topic", "/camera/image"), Image, on_image, queue_size=1)
    rospy.loginfo("detector ready (%s)", model_path)
    rospy.spin()


if __name__ == "__main__":
    main()
