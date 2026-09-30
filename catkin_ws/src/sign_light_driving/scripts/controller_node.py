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


def _wrap_angle(a):
    """각도를 (-pi, pi]로 접어서 yaw가 pi/-pi 경계를 넘어도 진행량이 튀지 않게 한다."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def _yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Controller:
    """검출 결과 -> (linear.x, angular.z). ROS 의존 없음 (그래서 테스트 가능).

    거리 센서 대신 bbox 높이 / 프레임 높이 비율을 근접도로 쓴다.
    파라미터 기본값은 실제 표지판 크기/조명에 맞춰 현장에서 튜닝할 것.
    """

    def __init__(self, conf=0.6, light_ratio=0.15, turn_ratio=0.30, debounce=3,
                 speed=0.1, turn_z=0.5, turn_angle=math.pi / 2, sign_cooldown=2.0,
                 turn_scale=1.0, turn_brake=0.0):
        self.conf = conf
        self.ratio = {"red_light": light_ratio, "green_light": light_ratio,
                      "left_turn": turn_ratio, "right_turn": turn_ratio}
        self.debounce = debounce
        self.speed = speed
        self.turn_z = turn_z
        # ponytail: 오도메트리/IMU 피드백 없이 시간 적분으로 각도를 맞추는 열린루프 방식.
        # 바퀴 슬립 등으로 오차가 누적되면 실제 yaw 피드백(오도메트리) 기반 회전으로 교체.
        # turn_scale: 실제 회전이 90도에 못 미치거나 넘치는 만큼 보정하는 현장 튜닝 값
        # (덜 돌면 키우고, 더 돌면 줄인다).
        self.turn_time = turn_angle / turn_z * turn_scale
        self.sign_cooldown = sign_cooldown

        self.turn_angle = turn_angle
        # turn_brake: 각속도 명령을 끊어도 관성으로 더 돌아가는 만큼 목표각에서 미리 빼는 값(라디안).
        # 실측(2026-09-28, turn_z=0.5): 좌회전 +5.2도, 우회전 +5.7도 관성 오버슈트 확인.
        self.turn_target = max(0.0, turn_angle - turn_brake)
        self.turn_safety_factor = 3.0  # IMU가 안 잡히거나 튈 때의 회전 시간 상한 배수

        self.stopped = True  # 초록불을 봐야 출발. 빨간불/미검출 상태로 시작
        self.turn_until = None
        self.turn_deadline = None
        self.turn_start_yaw = None
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

    def step(self, dets, now, yaw=None):
        # 회전 중에는 새 검출 무시 (상태머신 락)
        if self.turn_until is not None:
            # yaw(IMU 헤딩)가 있으면 실제 회전각으로 정지 시점을 판단(폐루프),
            # 없으면 기존 시간 적분(turn_time)으로 판단(열린루프, 기존 동작 그대로).
            if yaw is not None and self.turn_start_yaw is not None:
                turned = _wrap_angle(yaw - self.turn_start_yaw) * self.turn_dir
                done = turned >= self.turn_target or now >= self.turn_deadline
            else:
                done = now >= self.turn_until
            if not done:
                return (0.0, self.turn_dir * self.turn_z)
            self.turn_until = None
            self.turn_start_yaw = None
            # 돌고 난 뒤에도 같은 표지판이 시야에 남아 또 도는 것 방지
            self.ignore_signs_until = now + self.sign_cooldown

        cls = self._confirmed(dets)
        if cls == "red_light":
            self.stopped = True
        elif cls == "green_light":
            self.stopped = False
        elif cls in ("left_turn", "right_turn") and now >= self.ignore_signs_until:
            self.turn_until = now + self.turn_time
            self.turn_deadline = now + self.turn_time * self.turn_safety_factor
            self.turn_start_yaw = yaw
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

    # 아무것도 없으면 정지 상태로 시작 (초록불을 봐야 출발)
    c = Controller()
    assert c.step([], 0.0) == (0.0, 0.0)
    assert feed(c, green, 3, t0=0.1) == (0.1, 0.0)

    # 디바운스: 2프레임까지는 발동 안 함, 3프레임째 정지
    c = Controller()
    feed(c, green, 3)
    assert feed(c, red, 2, t0=0.4) == (0.1, 0.0)
    assert c.step(red, 0.7) == (0.0, 0.0)

    # 낮은 conf / 먼 거리(작은 bbox)는 무시
    c = Controller()
    feed(c, green, 3)
    assert feed(c, [("red_light", 0.5, 0.2)], 5, t0=0.4) == (0.1, 0.0)
    assert feed(c, [("red_light", 0.9, 0.1)], 5, t0=1.0) == (0.1, 0.0)

    # 연속이 끊기면 스트릭 리셋
    c = Controller()
    feed(c, green, 3)
    feed(c, red, 2, t0=0.4)
    c.step([], 0.7)
    assert c.step(red, 0.8) == (0.1, 0.0)

    # 정지 상태는 검출이 사라져도 유지, green 3프레임이면 재출발
    c = Controller()
    feed(c, green, 3)
    feed(c, red, 3, t0=0.4)
    assert c.step([], 0.8) == (0.0, 0.0)
    assert feed(c, green, 3, t0=0.9) == (0.1, 0.0)

    # 좌회전: 출발 신호(green) 없이도 즉시 회전 시작, 90도(turn_angle/turn_z)만큼 유지, 이후 직진 복귀
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

    # IMU(yaw)가 있으면 시간 대신 실제 회전각으로 정지: 슬립으로 덜 돌았으면 계속 돔
    c = Controller()
    c.step(left, 0.0)
    c.step(left, 0.1)
    assert c.step(left, 0.2, yaw=0.0) == (0.0, 0.5)         # 회전 시작, turn_start_yaw=0.0
    assert c.step([], 1.0, yaw=0.5) == (0.0, 0.5)           # 슬립으로 pi/2 못 미침 -> 계속 회전
    assert c.step([], 10.0, yaw=math.pi / 2) == (0.1, 0.0)  # 목표 각도 도달 -> 직진 복귀

    # turn_brake: 관성 오버슈트만큼 목표각보다 일찍 각속도를 끊는다
    c = Controller(turn_brake=0.1)
    c.step(left, 0.0)
    c.step(left, 0.1)
    assert c.step(left, 0.2, yaw=0.0) == (0.0, 0.5)
    assert c.step([], 1.0, yaw=math.pi / 2 - 0.2) == (0.0, 0.5)   # 목표(pi/2-0.1)보다 못 미침
    assert c.step([], 2.0, yaw=math.pi / 2 - 0.1) == (0.1, 0.0)   # pi/2-brake 도달 -> 조기 정지

    # yaw가 안 변해도(센서 불량/미장착) turn_deadline 지나면 강제 종료 (무한 회전 방지)
    c = Controller()
    c.step(left, 0.0)
    c.step(left, 0.1)
    c.step(left, 0.2, yaw=0.0)
    assert c.step([], 100.0, yaw=0.0) == (0.1, 0.0)

    print("selftest ok")


def main():
    import rospy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Imu
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
        turn_scale=rospy.get_param("~turn_scale", 1.0),
        turn_brake=rospy.get_param("~turn_brake", 0.09),
    )

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    last_msg = [0.0]
    yaw = [None]  # 내장 IMU에서 읽은 현재 헤딩. 폐루프 회전에 씀

    def send(lin, ang):
        t = Twist()
        t.linear.x, t.angular.z = lin, ang
        pub.publish(t)

    def on_imu(msg):
        q = msg.orientation
        yaw[0] = _yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def on_detections(msg):
        now = rospy.get_time()
        last_msg[0] = now
        dets = [(d.label, d.confidence, d.height_ratio) for d in msg.detections]
        send(*ctrl.step(dets, now, yaw=yaw[0]))

    def on_watchdog(_):
        now = rospy.get_time()
        if now - last_msg[0] > watchdog:
            # 무선이 끊기거나 노트북 추론이 멈추면 로봇이 마지막 명령으로 계속 달린다.
            # 정지 판단을 로봇 자신이 하므로 링크가 죽어도 확실히 선다.
            send(0.0, 0.0)
            return
        if ctrl.turn_until is not None:
            # 회전은 IMU로 각도를 확인해야 정확히 멈추므로, 검출 메시지 도착 주기와
            # 무관하게 이 타이머에서도 계속 확인한다. 주기가 굵으면 그 주기 동안
            # turn_z만큼 더 돌고 나서야 멈추는 오버슈트가 생긴다(10Hz 실측 시 약 5도) —
            # 50Hz로 좁혀서 그 오버슈트를 1/5 수준으로 줄인다.
            send(*ctrl.step([], now, yaw=yaw[0]))

    rospy.Subscriber("detections", DetectionArray, on_detections, queue_size=1)
    rospy.Subscriber("/imu", Imu, on_imu, queue_size=1)
    rospy.Timer(rospy.Duration(0.02), on_watchdog)
    rospy.on_shutdown(lambda: send(0.0, 0.0))
    rospy.loginfo("controller ready (watchdog %.2fs)", watchdog)
    rospy.spin()


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        selftest()
    else:
        main()
