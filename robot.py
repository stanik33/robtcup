# -*- coding: utf-8 -*-
import time
import pypot.dynamixel
import serial
import logging
import camera
import cv2
import numpy as np
import RPi.GPIO as GPIO

logger = logging.getLogger(__name__)


class Robot(object):
    _dxl_io: pypot.dynamixel.DxlIO = None
    _ard_io: serial.Serial = None
    _camera: camera.CameraReader = None

    to_storage_move_speed = 500
    to_storage_move_speed1 = 100
    to_storage_move_speed2 =300
    to_storage_move_speed3 = 200
    
    to_storage_move_fwd_time = 2
    to_storage_move_fwd_time1 = 1
    to_storage_move_left_time = 1.35
    to_storage_move_left_time1 = 2
    to_storage_move_fwd_time2 = 3
    to_storage_move_right_time = 2.5


    color_mask = {
        'Y': [[(0, 10, 90), (40, 200, 150)]],
        'G': [[(50, 95, 95), (80, 245, 255)]],
        'R': [[(150, 70, 80), (185, 255, 255)], [(0, 125, 80), (10, 255, 255)]],
        'B': [[(90, 80, 0), (125, 255, 255)]],
        'W': [[(61, 28, 153), (88, 87, 213)]],
    }

    def __init__(self):
        self.motors = {
            'j1': [1, 'joint'],
            'j2': [2, 'joint'],
            'front': [8, 'wheel'],
            'rear': [10, 'wheel'],
            'right': [7, 'wheel'],
            'left': [9, 'wheel'],
        }
        self._camera = camera.CameraReader(0, 320, 240)

    def connect(self, dxl_port, ard_port=None):
        print('Connecting to DXL port ', dxl_port)
        self._dxl_io = pypot.dynamixel.DxlIO(dxl_port)
        print('DXL Connected!')

        for m in self.motors:
            self.setup_motor(self.motors[m])
            print("Motor " + m + " configured")

        if not (ard_port is None):
            print('Connecting to Arduino port ', ard_port)
            self._ard_io = serial.Serial(ard_port, 115200)

        print('Starting camera')
        self._camera.start()

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(17, GPIO.OUT)
        GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def setup_motor(self, motor):
        motor_id = motor[0]
        if self._dxl_io.ping(motor_id):
            if motor[1] == 'wheel':
                self._dxl_io.set_wheel_mode([motor_id])
            else:
                self._dxl_io.set_joint_mode([motor_id])
            self._dxl_io.enable_torque([motor_id])
        else:
            raise IOError('Motor ' + motor_id + ' not responded')

    def close(self):
        if not (self._dxl_io is None):
            self._dxl_io.close()
            self._dxl_io = None
            print('DXL Connection closed')

        if not (self._ard_io is None):
            self._ard_io.close()
            self._ard_io = None
            print('Arduino Connection closed')

        self._camera.stop()
        print('Camera closed')

    def wait_btn(self):
        while GPIO.input(27) == GPIO.HIGH:
            time.sleep(0.1)

    def vacuum_on(self):
        GPIO.output(17, GPIO.HIGH)

    def vacuum_off(self):
        GPIO.output(17, GPIO.LOW)

    def set_speed(self, motors):
        s = {}
        for m in motors:
            s[self.motors[m][0]] = motors[m]
        self._dxl_io.set_moving_speed(s)

    def set_pos(self, motors, wait=False):
        s = {}
        for m in motors:
            s[self.motors[m][0]] = motors[m]

        self._dxl_io.enable_torque(s.keys())
        self._dxl_io.set_goal_position_speed_load(s)
        if wait:
            self.wait(s.keys())

    def wait(self, motors):
        moving = True
        while moving:
            try:
                mov = self._dxl_io.is_moving(motors)
                moving = False
                for m in mov:
                    if m:
                        moving = m
                        break
            except:
                self._dxl_io.enable_torque(motors)

    #############################################################################

    def init(self):
        self.set_pos({
            'j1': [-130, 100, 100],
            'j2': [90, 100, 100],
        }, True)

    def stop(self):
        self.set_speed({"front": 0, "rear": 0, "left": 0, "right": 0})

    def read_code(self):
        self.set_pos({
            'j1': [120, 100, 100],
            'j2': [85, 100, 100],
        }, True)

        return [
            self.detect_rect_color(122, 50, 155, 50),
            self.detect_rect_color(162, 50, 97, 50),
            self.detect_rect_color(203, 50, 36, 50),
        ]
        # frame = self._camera.read()
        # cv2.imwrite("frame.jpg", frame)

    def detect_rect_color(self, x, w, y, h, save=False, pref="", trace=False):
        frame = self._camera.read()
        cropped = frame[y:y + h, x:x + w]

        if save:
            s_frame = frame.copy()
            cv2.rectangle(s_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.imwrite('frame_f_' + pref + '.jpg', s_frame)
            cv2.imwrite('frame_c_' + pref + '.jpg', cropped)

        res = self.detect_frame_color(cropped, trace=trace)
        if res is None:
            inv = cv2.bitwise_not(cropped)
            r = self.detect_frame_color(inv)
            if r == 'W':
                res = 'N'
        return res

    def detect_frame_color(self, frame, trace=False):
        blured = cv2.blur(frame, (2, 2))
        hsv = cv2.cvtColor(blured, cv2.COLOR_BGR2HSV)
        res = None
        max = 0

        for c in self.color_mask:
            mask = cv2.inRange(hsv, self.color_mask[c][0][0], self.color_mask[c][0][1])
            if len(self.color_mask[c]) > 1:
                mask = mask + cv2.inRange(hsv, self.color_mask[c][1][0], self.color_mask[c][1][1])
            cnt = cv2.countNonZero(mask)
            if trace:
                print(c, cnt)
            if cnt > 500 and cnt > max:
                max = cnt
                res = c
        return res

    def correction_horz(self):
        while True:
            a = self.calc_angle()
            if a is None:
                break
            else:
                a = a if a < 90 else a - 180
                a = a - 4
                print(a)
                if -2 < a < 2:
                    break
                else:
                    a = 20 * a
                    self.set_speed({'left': a, 'right': a, 'front': a, 'rear': a})
                time.sleep(0.01)
        self.stop()

    def to_storage(self):
        self.set_pos({
            'j1': [90, 100, 100],
            'j2': [90, 100, 100],
        }, True)
        # Едем вперед
        t = time.time()
        while time.time() - t < self.to_storage_move_fwd_time:
            self.set_speed({'left': self.to_storage_move_speed, 'right': -self.to_storage_move_speed, 'front': 0, 'rear': 0})
            time.sleep(0.01)
        # Едем влево
        t = time.time()
        while time.time() - t < self.to_storage_move_left_time:
            self.set_speed({'left': 0, 'right': 0, 'front': -self.to_storage_move_speed, 'rear': self.to_storage_move_speed})
            time.sleep(0.01)
        # Едем вперед
        t = time.time()
        while time.time() - t < self.to_storage_move_fwd_time:
            a = self.calc_angle()
            if a is None:
                self.set_speed({'left': self.to_storage_move_speed, 'right': -self.to_storage_move_speed, 'front': 0, 'rear': 0})
                time.sleep(0.01)
            else:
                time.sleep(0.05)
                break
        self.stop()
        # Выравниваемся
        self.correction_horz()
        time.sleep(0.01)
        # Поворачиваем руку вправо (не ждем)
        self.set_pos({'j1': [0, 100, 100], 'j2': [90, 100, 100]}, False)
        # Едем вперед
        t = time.time()
        while time.time() - t < self.to_storage_move_fwd_time2:
            self.set_speed(
                {'left': self.to_storage_move_speed, 'right': -self.to_storage_move_speed, 'front': 0, 'rear': 0})
            time.sleep(0.01)
        self.stop()
        # Поворачиваем руку вправо (ждем)
        self.set_pos({
            'j1': [0, 100, 100],
            'j2': [90, 100, 100],
        }, True)
        # Едем вправо до линии
        t = time.time()
        while time.time() - t < self.to_storage_move_right_time:
            a = self.calc_angle()
            if a is None:
                self.set_speed({'left': 0, 'right': 0, 'front': self.to_storage_move_speed, 'rear': -self.to_storage_move_speed})
                time.sleep(0.01)
            else:
                time.sleep(0.1)
                break
        self.stop()
        self.correction_horz()
        # Поворачиваем руку вперед
        self.set_pos({
            'j1': [90, 100, 100],
            'j2': [90, 100, 100],
        }, True)

    def calc_angle(self):
        frame = self._camera.read()
        frame = cv2.GaussianBlur(frame, (5, 5), 2)
        cv2.imwrite("frame.jpg", frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY)
        cv2.imwrite("thresh.jpg", thresh)
        thresh = cv2.bitwise_not(thresh)
        contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
        max_cnt = 0
        max_contour = None
        for c in contours:
            mask = np.zeros(thresh.shape[:2], dtype="uint8")
            cv2.drawContours(mask, [c], -1, color=255, thickness=cv2.FILLED)
            masked = cv2.bitwise_and(thresh, thresh, mask=mask)
            area = cv2.contourArea(c)
            approx = cv2.approxPolyDP(c, 0.03 * cv2.arcLength(c, True), True)
            if len(approx) == 4 and cv2.countNonZero(masked) >= 0.9 * area and area > max_cnt:
                max_cnt = area
                max_contour = approx
        if max_cnt > 500:
            image_copy = frame.copy()
            cv2.drawContours(image=image_copy, contours=[max_contour], contourIdx=-1, color=(0, 255, 0), thickness=2,
                             lineType=cv2.LINE_AA)
            cv2.imwrite("cont.jpg", image_copy)
            max_d = 0
            max_p1 = None
            max_p2 = None
            r = max_contour.ravel()
            for i in range(4):
                p1 = [r[2 * i], r[(2 * i) + 1]]
                if i < 3:
                    p2 = [r[2 * (i + 1)], r[(2 * (i + 1)) + 1]]
                else:
                    p2 = [r[0], r[1]]
                dx = p1[0] - p2[0]
                dy = p1[1] - p2[1]
                d = np.sqrt(dx * dx + dy * dy)
                if d > max_d:
                    max_d = d
                    max_p1 = p1
                    max_p2 = p2

            a = np.rad2deg(np.arctan2(max_p2[1] - max_p1[1], max_p2[0] - max_p1[0]))
            a = a if a > 0 else 180 + a
            return a
        else:
            return None

    def find_circle(self):
        a = 0
        self.set_pos({
            'j1': [90, 100, 100],
            'j2': [90, 100, 100],
        }, True)
        t = time.time()
        while time.time() - t < 4.3:
            frame = self._camera.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            circ = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 50, param1=50, param2=50, minRadius=0, maxRadius=0)
            if circ is None:
                self.set_speed(
                    {'left': self.to_storage_move_speed1, 'right': -self.to_storage_move_speed1, 'front': 0, 'rear': 0})
                time.sleep(0.01)
            else:
                print(circ)
                time.sleep(0.1)
                break
        self.stop()

        t = time.time()
        while time.time() - t < 10:
            frame = self._camera.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            circ = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 50, param1=50, param2=50, minRadius=0, maxRadius=0)
            center = None
            if not (circ is None):
                circ = np.uint16(np.around(circ))[0, :]
                for j in circ:
                    #print(j)
                    mask = np.zeros(gray.shape[:2], dtype="uint8")
                    cv2.circle(mask, (j[0], j[1]), j[2], 255, -1)
                    th, binary = cv2.threshold(gray, 128, 192, cv2.THRESH_BINARY)
                    masked = cv2.bitwise_and(binary, binary, mask=mask)
                    if cv2.countNonZero(masked) > 5000:
                        radius = cv2.circle(gray, (j[0], j[1]), j[2], (0, 100, 100), 2)
                        center = cv2.circle(gray, (j[0], j[1]), 2, (0, 100, 100), 3)
                        print(center)
                        break

            if center is None:
                self.set_speed({'left': 0, 'right': 0, 'front': self.to_storage_move_speed1, 'rear': -self.to_storage_move_speed1})
                time.sleep(0.001)
            else:
                break
        self.stop()
    def take_ball(self):
        self.set_pos({'j2': [135, 80, 100]})
        time.sleep(3)
        GPIO.output(17, GPIO.HIGH)
        time.sleep(0.5)
        self.set_pos({'j2': [90, 80, 100]})
        time.sleep(1)
        self.stop
    def rotate_180(self):
        self.set_speed({'left': -200, 'right': -200, 'front': -200, 'rear': -200})
        time.sleep(3.60)
        self.stop
    def spin_me_right_round(self):
        self.set_speed({'left': self.to_storage_move_speed2, 'right': -self.to_storage_move_speed2, 'front': 0, 'rear': 0})
        time.sleep(2)
        self.set_speed({'left': -200, 'right': -200, 'front': -200, 'rear': -200})
        time.sleep(60)
        self.set_pos({'j2': [90, 100, 100]})
        self.set_pos({'j2': [135, 100, 100]})
        self.set_pos({'j2': [90, 100, 100]})
        self.set_pos({'j2': [135, 100, 100]})
        self.stop
    def buhta(self):
        t = time.time()
        while time.time() - t < self.to_storage_move_fwd_time2:
            self.set_speed(
                {'left': self.to_storage_move_speed2, 'right': -self.to_storage_move_speed2, 'front': 0, 'rear': 0})
            time.sleep(0.01)
        self.set_pos({'j2': [135, 100, 100]})
        time.sleep(1.5)
        GPIO.output(17, GPIO.LOW)
    def center(self):
        t = time.time()
        while time.time() - t < 30:
            frame = self._camera.read()
            image_output = frame.copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            circ = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 50, param1=50, param2=50, minRadius=0, maxRadius=0)
            center = None
            if not (circ is None):
                circ = np.round(circ[0, :]).astype("int")
                for (x, y, r) in circ:
                    print(f"Center: {x},{y}   Radius: {r}")
                    # обведём найденный круг
                    cv2.circle(image_output, (x, y), r, (0, 255, 0), 1)
                    # и ткнём точку в центр
                    cv2.circle(image_output, (x, y), 2, (255, 0, 255), -1)
                for j in circ:
                    # print(j)
                    mask = np.zeros(gray.shape[:2], dtype="uint8")
                    cv2.circle(mask, (j[0], j[1]), j[2], 255, -1)
                    th, binary = cv2.threshold(gray, 128, 192, cv2.THRESH_BINARY)
                    masked = cv2.bitwise_and(binary, binary, mask=mask)
                    if cv2.countNonZero(masked) > 5000:
                        radius = cv2.circle(gray, (j[0], j[1]), j[2], (0, 100, 100), 2)
                        center = cv2.circle(gray, (j[0], j[1]), 2, (0, 100, 100), 3)
                        #print(center)
        cv2.imwrite("rk_battle/center1.jpg", frame)
    def huy_v_zhopu(self):
        t = time.time()
        while time.time() - t < 10:
            image = self._camera.read()
            image_output = image.copy()
            # конвертация в grayscale
            image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # детекция кругов
            circles = cv2.HoughCircles(image_gray, cv2.HOUGH_GRADIENT, 1, 50, param1=50, param2=50, minRadius=0, maxRadius=0)

            if circles is not None:
                # конвертация координат центра и радиуса в int
                circles = np.round(circles[0, :]).astype("int")

                for (x, y, r) in circles:
                    print(f"Center: {x},{y}   Radius: {r}")
                    # обведём найденный круг
                    cv2.circle(image_output, (x, y), r, (0, 255, 0), 1)
                    # и ткнём точку в центр
                    cv2.circle(image_output, (x, y), 2, (255, 0, 255), -1)
            cv2.imwrite("huy.jpg", image_output)
    def camera_set(self):
        self.set_pos({
            'j1': [90, 100, 100],
            'j2': [90, 100, 100],
        })
        time.sleep(1)
        self.stop
                        
        
