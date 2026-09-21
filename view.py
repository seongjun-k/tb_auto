#!/usr/bin/env python3
"""로봇 카메라 실시간 보기 (+YOLO 검출 오버레이). 로봇은 안 움직인다. q로 종료."""
import numpy as np, cv2, rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

CLASSES = ["left_turn", "right_turn", "red_light", "green_light"]
m = YOLO("runs/detect/runs/train/weights/best.pt")
m.predict(np.zeros((240, 320, 3), np.uint8), imgsz=320, verbose=False)

rospy.init_node("view", anonymous=True, disable_signals=True)
br = CvBridge()
state = {"f": None}
rospy.Subscriber("/camera/image", Image, lambda msg: state.update(f=br.imgmsg_to_cv2(msg, "bgr8")), queue_size=1)

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
    cv2.imshow("camera + detection (q=quit)", vis)
    if cv2.waitKey(20) & 0xFF == ord("q"):
        break
cv2.destroyAllWindows()
