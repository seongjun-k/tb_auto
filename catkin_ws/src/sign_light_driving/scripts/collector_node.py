#!/usr/bin/env python3
"""데이터 수집 노드 - 노트북에서 실행.

영상은 로봇 카메라(aicon_camera.launch가 내보내는 /camera/image)를 구독한다.
화면 표시와 이미지 저장은 노트북에서 한다 - 찍는 장면을 보면서 모을 수 있고
이미지가 노트북에 바로 쌓여 scp가 필요 없다.

  roslaunch sign_light_driving collect.launch

키: 1~4 클래스 선택 / space 저장 / q 종료
"""
import os
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

CLASSES = ["left_turn", "right_turn", "red_light", "green_light"]
DEFAULT_SAVE_DIR = "~/tb_auto/dataset_raw"


def main():
    rospy.init_node("collector")
    bridge = CvBridge()
    topic = rospy.get_param("~topic", "/camera/image")
    save_dir = os.path.expanduser(rospy.get_param("~save_dir", DEFAULT_SAVE_DIR))

    state = {"frame": None}
    rospy.Subscriber(topic, Image, lambda m: state.update(frame=bridge.imgmsg_to_cv2(m, "bgr8")))

    for c in CLASSES:
        os.makedirs(os.path.join(save_dir, c), exist_ok=True)
    # labelImg가 읽는 클래스 목록. 순서가 곧 YOLO 클래스 인덱스라 절대 바꾸지 말 것.
    with open(os.path.join(save_dir, "classes.txt"), "w") as f:
        f.write("\n".join(CLASSES) + "\n")
    counts = {c: len(os.listdir(os.path.join(save_dir, c))) for c in CLASSES}

    sel = 0
    rospy.loginfo("waiting for %s ... (save_dir=%s)", topic, save_dir)
    while not rospy.is_shutdown():
        frame = state["frame"]
        if frame is None:
            rospy.sleep(0.05)
            continue

        view = cv2.resize(frame, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
        cv2.putText(view, "[%d] %s  saved=%d" % (sel + 1, CLASSES[sel], counts[CLASSES[sel]]),
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("collect (1-4 class / space save / q quit)", view)

        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break
        if ord("1") <= key <= ord("4"):
            sel = key - ord("1")
        elif key == ord(" "):
            name = CLASSES[sel]
            stamp = time.strftime("%Y%m%d_%H%M%S") + "_%03d" % (time.time() % 1 * 1000)
            path = os.path.join(save_dir, name, "%s_%s.jpg" % (name, stamp))
            cv2.imwrite(path, frame)   # 리사이즈 전 원본 320x240 저장
            counts[name] += 1
            rospy.loginfo("saved %s (%d)", path, counts[name])

    cv2.destroyAllWindows()
    print({c: counts[c] for c in CLASSES})


if __name__ == "__main__":
    main()
