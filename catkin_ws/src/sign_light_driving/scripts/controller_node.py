#!/usr/bin/env python3
"""제어 노드 - 터틀봇에서 실행.

노트북의 detector_node가 보내는 인식 결과를 구독해 cmd_vel을 만든다.
torch/ultralytics가 필요 없는 순수 로직이라 RPi4에서도 가볍게 돈다.

  roslaunch sign_light_driving control.launch

로직만 검증:  rosrun sign_light_driving controller_node.py --selftest
"""
import math
import sys

CLASSES = ["left_turn", "right_turn", "red_light", "green_light"]


class Controller:
    """검출 결과 -> (linear.x, angular.z). ROS 의존 없음 (그래서 테스트 가능).

    거리 센서 대신 bbox 높이 / 프레임 높이 비율을 근접도로 쓴다.
    파라미터 기본값은 실제 표지판 크기/조명에 맞춰 현장에서 튜닝할 것.
    """

    def __init__(self, conf=0.6, light_ratio=0.15, turn_ratio=0.30, debounce=3,
                 speed=0.1, turn_z=0.5, turn_angle=math.pi / 2, sign_cooldown=2.0):
        self.conf = conf
        self.ratio = {"red_light": light_ratio, "green_light": light_ratio,
                      "left_turn": turn_ratio, "right_turn": turn_ratio}
        self.debounce = debounce
        self.speed = speed
        self.turn_z = turn_z
        # ponytail: 오도메트리/IMU 피드백 없이 시간 적분으로 각도를 맞추는 열린루프 방식.
        # 바퀴 슬립 등으로 오차가 누적되면 실제 yaw 피드백(오도메트리) 기반 회전으로 교체.
        self.turn_time = turn_angle / turn_z
        self.sign_cooldown = sign_cooldown

        self.stopped = False
        self.turn_until = None
        self.turn_dir = 0
        self.ignore_signs_until = 0.0
        self._streak_cls = None
        self._streak_n = 0

    def _confirmed(self, dets):
        """conf/근접도 통과 검출 중 최고 conf 클래스가 연속 debounce 프레임이면 반환."""
        ok = [d for d in dets if d[1] >= self.conf and d[2] >= self.ratio[d[0]]]
        cls = max(ok, key=lambda d: d[1])[0] if ok else None
        if cls == self._streak_cls:
            self._streak_n += 1
        else:
            self._streak_cls, self._streak_n = cls, 1
        return cls if cls is not None and self._streak_n >= self.debounce else None

    def step(self, dets, now):
        # 회전 중에는 새 검출 무시 (상태머신 락)
        if self.turn_until is not None:
            if now < self.turn_until:
                return (0.0, self.turn_dir * self.turn_z)
            self.turn_until = None
            # 돌고 난 뒤에도 같은 표지판이 시야에 남아 또 도는 것 방지
            self.ignore_signs_until = now + self.sign_cooldown

        cls = self._confirmed(dets)
        if cls == "red_light":
            self.stopped = True
        elif cls == "green_light":
            self.stopped = False
        elif cls in ("left_turn", "right_turn") and now >= self.ignore_signs_until:
            self.turn_until = now + self.turn_time
            self.turn_dir = 1 if cls == "left_turn" else -1
            self.stopped = False
            return (0.0, self.turn_dir * self.turn_z)

        return (0.0, 0.0) if self.stopped else (self.speed, 0.0)


def selftest():
    def feed(c, dets, n, t0=0.0, dt=0.1):
        out = None
        for i in range(n):
            out = c.step(dets, t0 + i * dt)
        return out

    red = [("red_light", 0.9, 0.2)]
    green = [("green_light", 0.9, 0.2)]
    left = [("left_turn", 0.9, 0.4)]

    # 아무것도 없으면 기본 직진
    c = Controller()
    assert c.step([], 0.0) == (0.1, 0.0)

    # 디바운스: 2프레임까지는 발동 안 함, 3프레임째 정지
    c = Controller()
    assert feed(c, red, 2) == (0.1, 0.0)
    assert c.step(red, 0.3) == (0.0, 0.0)

    # 낮은 conf / 먼 거리(작은 bbox)는 무시
    c = Controller()
    assert feed(c, [("red_light", 0.5, 0.2)], 5) == (0.1, 0.0)
    assert feed(c, [("red_light", 0.9, 0.1)], 5) == (0.1, 0.0)

    # 연속이 끊기면 스트릭 리셋
    c = Controller()
    feed(c, red, 2)
    c.step([], 0.3)
    assert c.step(red, 0.4) == (0.1, 0.0)

    # 정지 상태는 검출이 사라져도 유지, green 3프레임이면 재출발
    c = Controller()
    feed(c, red, 3)
    assert c.step([], 0.4) == (0.0, 0.0)
    assert feed(c, green, 3, t0=0.5) == (0.1, 0.0)

    # 좌회전: 즉시 회전 시작, turn_time 동안 유지, 이후 직진 복귀
    c = Controller()
    assert feed(c, left, 3) == (0.0, 0.5)
    assert c.step([], 1.0) == (0.0, 0.5)        # 회전 중 무시
    assert c.step(red, 2.0) == (0.0, 0.5)       # 빨간불도 무시 (락)
    assert c.step([], 5.5) == (0.1, 0.0)        # 0.2+3.1=3.3 경과

    # 우회전은 반대 방향
    c = Controller()
    assert feed(c, [("right_turn", 0.9, 0.4)], 3) == (0.0, -0.5)

    # 회전 직후 쿨다운: 같은 표지판이 남아 있어도 다시 돌지 않음
    c = Controller()
    feed(c, left, 3)
    c.step([], 5.0)                              # 회전 종료 -> 쿨다운 7.0까지
    assert feed(c, left, 3, t0=5.1) == (0.1, 0.0)
    assert feed(c, left, 3, t0=7.1) == (0.0, 0.5)

    # 좌회전: 즉시 회전 시작, 90도(turn_angle/turn_z)만큼 유지, 이후 직진 복귀
    c = Controller()
    assert feed(c, left, 3) == (0.0, 0.5)
    assert c.step([], 1.0) == (0.0, 0.5)        # 회전 중 무시
    assert c.step(red, 2.0) == (0.0, 0.5)       # 빨간불도 무시 (락)
    assert c.step([], 5.5) == (0.1, 0.0)        # 0.2+pi≈3.34 경과, 이후 신호 없으면 직진

    # 우회전은 반대 방향
    c = Controller()
    assert feed(c, [("right_turn", 0.9, 0.4)], 3) == (0.0, -0.5)

    # 회전 직후 쿨다운: 같은 표지판이 남아 있어도 다시 돌지 않음
    c = Controller()
    feed(c, left, 3)
    c.step([], 5.0)                              # 회전 종료 -> 쿨다운 5.34초 후까지
    assert feed(c, left, 3, t0=5.1) == (0.1, 0.0)
    assert feed(c, left, 3, t0=7.1) == (0.0, 0.5)

    # 회전 종료 후에는 신호등 신호에 따라 동작(정지)
    c = Controller()
    feed(c, left, 3)
    assert feed(c, red, 3, t0=3.5) == (0.0, 0.0)

    print("selftest ok")


def main():
    import rospy
    from geometry_msgs.msg import Twist
    from sign_light_driving.msg import DetectionArray

    rospy.init_node("controller")
    watchdog = rospy.get_param("~watchdog", 0.5)
    ctrl = Controller(
        conf=rospy.get_param("~conf", 0.6),
        light_ratio=rospy.get_param("~light_ratio", 0.15),
        turn_ratio=rospy.get_param("~turn_ratio", 0.30),
        debounce=rospy.get_param("~debounce", 3),
        speed=rospy.get_param("~speed", 0.1),
        turn_z=rospy.get_param("~turn_z", 0.5),
        turn_angle=rospy.get_param("~turn_angle", math.pi / 2),
    )

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    last_msg = [0.0]

    def send(lin, ang):
        t = Twist()
        t.linear.x, t.angular.z = lin, ang
        pub.publish(t)

    def on_detections(msg):
        now = rospy.get_time()
        last_msg[0] = now
        dets = [(d.label, d.confidence, d.height_ratio) for d in msg.detections]
        send(*ctrl.step(dets, now))

    def on_watchdog(_):
        # 무선이 끊기거나 노트북 추론이 멈추면 로봇이 마지막 명령으로 계속 달린다.
        # 정지 판단을 로봇 자신이 하므로 링크가 죽어도 확실히 선다.
        if rospy.get_time() - last_msg[0] > watchdog:
            send(0.0, 0.0)

    rospy.Subscriber("detections", DetectionArray, on_detections, queue_size=1)
    rospy.Timer(rospy.Duration(0.1), on_watchdog)
    rospy.on_shutdown(lambda: send(0.0, 0.0))
    rospy.loginfo("controller ready (watchdog %.2fs)", watchdog)
    rospy.spin()


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        selftest()
    else:
        main()
