#!/usr/bin/env python3
"""로봇 카메라 실시간 보기 (+YOLO 검출 오버레이). 로봇은 안 움직인다. q로 종료."""
import numpy as np, cv2, rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO

CLASSES = ["left_turn", "right_turn", "red_light", "green_light"]
m = YOLO("runs/detect/runs/train/weights/best.pt")
m.predict(np.zeros((240, 320, 3), np.uint8), imgsz=320, verbose=False)

rospy.init_node("view", anonymous=True, disable_signals=True)
br = CvBridge()
state = {"f": None, "cmd": (0.0, 0.0)}
rospy.Subscriber("/camera/image", Image, lambda msg: state.update(f=br.imgmsg_to_cv2(msg, "bgr8")), queue_size=1)
# 로봇이 실제로 받는 명령을 같이 띄운다. 검출과 동작이 맞는지 한 화면에서 보려고.
rospy.Subscriber("/cmd_vel", Twist, lambda m: state.update(cmd=(m.linear.x, m.angular.z)), queue_size=1)

print("창이 뜹니다. q 누르면 종료.")
while not rospy.is_shutdown():
    f = state["f"]
    if f is None:
        rospy.sleep(0.05); continue
    h = f.shape[0]
    r = m.predict(f, imgsz=320, conf=0.25, verbose=False)[0]
    vis = cv2.resize(f, None, fx=2.5, fy=2.5, interpolation=cv2.INTER_NEAREST)
    for b in r.boxes:
        x1, y1, x2, y2 = [int(v * 2.5) for v in b.xyxy[0]]
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cv2.putText(vis, "%s %.2f r=%.2f" % (CLASSES[int(b.cls)], float(b.conf), float(b.xywh[0][3]) / h),
                    (x1, max(14, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    lin, ang = state["cmd"]
    act = "STOP" if (abs(lin) < 1e-3 and abs(ang) < 1e-3) else (
        ("TURN LEFT" if ang > 0 else "TURN RIGHT") if abs(ang) > 1e-3 else "FORWARD")
    color = (0, 0, 255) if act == "STOP" else ((0, 200, 255) if "TURN" in act else (0, 220, 0))
    cv2.rectangle(vis, (0, 0), (vis.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(vis, "%s   lin=%.2f  ang=%.2f" % (act, lin, ang), (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.imshow("camera + detection + cmd_vel (q=quit)", vis)
    if cv2.waitKey(20) & 0xFF == ord("q"):
        break
cv2.destroyAllWindows()
