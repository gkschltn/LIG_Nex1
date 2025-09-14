#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2, rospy, time, os, math, collections
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

# ============================================================
# 변수 초기화
# ============================================================

# 1. 일반 변수
image = np.empty(shape=[0])
ranges = None
motor = None
motor_msg = XycarMotor()
Fix_Speed = 30
bridge = CvBridge()
angle_history = collections.deque(maxlen=5)
theta=np.pi/180
threshold=90


# 2. 이전 프레임의 회귀 결과를 보관해 오류 복구(NaN 방지)용 전역 변수
# 프레임 간에 라인 검출 실패 시(계산 결과 NaN 발생) 이전 정상 값을 사용해 픽스하기 위함
p_cross_x=-1
p_cross_y=-1
p_left_calculated_bias=-1
p_right_calculated_bias=-1
p_l_target=-1
p_r_target=-1

# 3. 라이다 시각화 설정
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-120, 120)
ax.set_ylim(-120, 120)
ax.set_aspect('equal')
lidar_points, = ax.plot([], [], 'bo')

#===================================================================================
# 토픽 구독 및 발행
#===================================================================================

def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def lidar_callback(data):
    global ranges    
    ranges = data.ranges[0:360]

def drive(angle, speed):
    motor_msg.angle = float(angle)
    motor_msg.speed = float(speed)
    motor.publish(motor_msg)

#===================================================================================
# 신호등
#===================================================================================
def detect_green_light(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_green = np.array([45, 100, 100])
    upper_green = np.array([75, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    green_area = cv2.countNonZero(mask)
    # cv2.imshow("Green Mask", mask)
    return green_area > 500

#===================================================================================
# 차선 인식 알고리즘
#===================================================================================
# 1. 차선 감지 및 그에 따른 조향
def process_lane(frame):
    height, width = frame.shape[:2]
    roi = frame[height * 5 // 8:, :]  # 상단 1/8 추가로 자름

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # 왼쪽 흰색 차선 (이미지 왼쪽 절반만 마스크)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])
    mask_white = cv2.inRange(hsv, lower_white, upper_white)
    mask_white[:, width//2:] = 0  # 오른쪽 흰색 차선 제거

    # 오른쪽 흰색 차선 (이미지 오른쪽 절반만 마스크)
    mask_right_white = cv2.inRange(hsv, lower_white, upper_white)
    mask_right_white[:, :width//2] = 0

    # 오른쪽 노란색 중앙선 (오른쪽 절반)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([40, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_yellow[:, :width//2] = 0  # 왼쪽 노란색 제거

    # 선 추출 함수
    def find_lane_centers(mask, side_name=""):
        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=30, minLineLength=30, maxLineGap=20)
        x_points = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(roi, (x1,y1), (x2,y2), (0,255,0), 2)
                mid_x = (x1 + x2) // 2
                x_points.append(mid_x)
        if len(x_points) > 0:
            center_x = np.mean(x_points)
            return center_x
        else:
            return None

    left_white_x = find_lane_centers(mask_white, "left_white")
    right_yellow_x = find_lane_centers(mask_yellow, "right_yellow")
    right_white_x = find_lane_centers(mask_right_white, "right_white")

    center_x_img = width // 2
    angle = None

    # 왼쪽 흰색 & 오른쪽 노란색 둘 다 있을 때
    if left_white_x is not None and right_yellow_x is not None:
        lane_center = (left_white_x + right_yellow_x) / 2
        offset = lane_center - center_x_img
        angle = -math.degrees(math.atan2(offset, 100))
    # 왼쪽 흰색만 있을 때
    elif left_white_x is not None:
        offset = left_white_x - center_x_img + 50  # 오른쪽 노란선 보정용 약간 오른쪽으로 오프셋
        angle = -math.degrees(math.atan2(offset, 100))
    # 오른쪽 노란색만 있을 때
    elif right_yellow_x is not None:
        offset = right_yellow_x - center_x_img - 50  # 왼쪽 흰선 보정용 약간 왼쪽으로 오프셋
        angle = -math.degrees(math.atan2(offset, 100))
    # 둘 다 없으면 오른쪽 흰색 차선으로 계산
    elif right_white_x is not None:
        offset = right_white_x - center_x_img
        angle = -math.degrees(math.atan2(offset, 50))
    else:
        angle = 0  # 차선 못 찾으면 직진

    # 각도 평균내기
    if not np.isfinite(angle):
        angle = 0
    angle_history.append(angle if np.isfinite(angle) else 0)
    smooth_angle = np.mean(angle_history)

    # 조향 시각화
    offset_px = int(smooth_angle * 5)
    steering_x = center_x_img + offset_px
    cv2.line(roi, (center_x_img, roi.shape[0]), (steering_x, roi.shape[0] - 50), (0, 0, 255), 3)

    return smooth_angle, roi

# 2. 각도에 따른 속도 조정
def compute_speed(angle, max_speed=30, min_speed=12):
    if -5 <= angle <= 5:
        return Fix_Speed
    norm_angle = abs(angle)

    if norm_angle >= 50:
        return 8  # 최저 속도 강제
    
    k = (max_speed - min_speed) / (45**2)
    speed = max(min_speed, max_speed - k * (norm_angle**2))
    return speed

#===================================================================================
# 라바콘 회피 주행
#===================================================================================

# 1. 라바콘 인식
def detect_cones_by_color(img, width_threshold):
    height = img.shape[0]
    roi_start = int(height * 0.4)   # 상단 40% 잘라냄
    roi_img = img[roi_start:, :]   # 하단 60%만 남김

    hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)

    lower_orange = np.array([5, 100, 100])
    upper_orange = np.array([20, 255, 255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    r_mid, l_mid, m_mid = [], [], []
    for pts in contours:
        if cv2.contourArea(pts) < 100:
            continue
        x, y, w, h = cv2.boundingRect(pts)
        mid = [x + w // 2, y + h // 2]

        if mid[0] > width_threshold:
            r_mid.append(mid)
        else:
            l_mid.append(mid)
        m_mid.append(mid)
        
    return r_mid, l_mid, m_mid, mask

# 2. 중심점 리스트 기반 선형 회귀
def weighted_linear_reg(points):
    """중심점 리스트 기반 선형 회귀"""
    if len(points) < 2:
        return None, None
    xs = np.array([p[0] for p in points])
    ys = np.array([p[1] for p in points])
    mean_x, mean_y = np.mean(xs), np.mean(ys)
    denom = ((xs - mean_x) ** 2).sum()
    if denom == 0:
        return None, None
    slope = ((xs - mean_x) * (ys - mean_y)).sum() / denom
    bias = mean_y - slope * mean_x
    return slope, bias

# 3. 각도 보정
def compute_angle(offset, base_distance=60):
    k = 2 if abs(offset) > 60 else 1.0  # offset이 클수록 더 민감하게
    return -math.degrees(math.atan2(k * offset, base_distance))

# 4. 라바콘 회피 주행
def lavacone_refined(image, m_mid, lateral_offset = 100):
    # m_mid = filter_nearby_cones(m_mid)
    slope, bias = weighted_linear_reg(m_mid)

    if slope is None or bias is None:
        return 0  # fallback: 직진

    y_target = image.shape[0] - 50
    x_target = (y_target - bias) / slope

    x_target += lateral_offset
    center_x_img = image.shape[1] // 2
    offset = x_target - center_x_img

    # 급커브 보정 조향
    angle = compute_angle(offset, base_distance=60)
    angle = np.clip(angle, -80, 120)
    return angle


#==========================================================================================================
# 장애물 감지
#======================================================================================================
# LaserScan.ranges 값은 미터(m) 단위 (range = 1.2 : 1.2m)
# threshold = 20m (감지 거리) fov = 15 (좌우 15도씩 총 30도의 시야각)

# 1. 전방 거리 계산 함수
def front_distance(ranges):
    if ranges is None:
        return None
    # 전방 +-30도 영역 거리 중 유효 거리만 필터링
    front_angles = list(range(-30, 30))
    front_indices = [(angle + 360) % 360 for angle in front_angles]
    front_distances = [ranges[i] for i in front_indices if 0.01 < ranges[i]]
    # 유효 값 없으면 None 반환
    if len(front_distances) == 0:
        return None
    return min(front_distances)

# 2. 장애물 회피 및 차선 유지
def drive_by_white_lane(image, angle_clip_left=(-5, 10), angle_clip_right=(-10, 5), base_speed=60):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])
    mask_white = cv2.inRange(hsv, lower_white, upper_white)

    height, width = mask_white.shape
    center_x_img = width // 2

    # 오른쪽 흰색 차선
    right_mask = mask_white[:, width//2:]
    edges_right = cv2.Canny(right_mask, 50, 150)
    lines_right = cv2.HoughLinesP(edges_right, 1, np.pi/180, threshold=30, minLineLength=30, maxLineGap=20)

    # 왼쪽 흰색 차선
    left_mask = mask_white[:, :width//2]
    edges_left = cv2.Canny(left_mask, 50, 150)
    lines_left = cv2.HoughLinesP(edges_left, 1, np.pi/180, threshold=30, minLineLength=30, maxLineGap=20)

    right_x, left_x = [], []

    if lines_right is not None:
        for line in lines_right:
            x1, _, x2, _ = line[0]
            mid_x = (x1 + x2) // 2
            right_x.append(mid_x)

    if lines_left is not None:
        for line in lines_left:
            x1, _, x2, _ = line[0]
            mid_x = (x1 + x2) // 2
            left_x.append(mid_x)

    valid_right = len(right_x) > 3
    valid_left = len(left_x) > 3

    if valid_right and valid_left:
        right_center = np.mean(right_x) + width // 2
        left_center = np.mean(left_x)

        # 중심 기준에서 더 가까운 차선 선택
        if abs(right_center - center_x_img) < abs(left_center - center_x_img):
            offset = right_center - center_x_img
            angle = -math.degrees(math.atan2(offset, 100))
            angle = np.clip(angle, *angle_clip_right)
            print(f"➡️ 오른쪽 흰색 차선 기준 주행 (우선), angle={angle:.2f}")
        else:
            offset = left_center - center_x_img
            angle = -math.degrees(math.atan2(offset, 100))
            angle = np.clip(angle, *angle_clip_left)
            print(f"⬅️ 왼쪽 흰색 차선 기준 주행 (우선), angle={angle:.2f}")
    
    elif valid_right:
        lane_center = np.mean(right_x) + width // 2
        offset = lane_center - center_x_img
        angle = -math.degrees(math.atan2(offset, 100))
        angle = np.clip(angle, *angle_clip_right)
        print(f"➡️ 오른쪽 흰색 차선 기준 주행 (단독), angle={angle:.2f}")
    
    elif valid_left:
        lane_center = np.mean(left_x)
        offset = lane_center - center_x_img
        angle = -math.degrees(math.atan2(offset, 100))
        angle = np.clip(angle, *angle_clip_left)
        print(f"⬅️ 왼쪽 흰색 차선 기준 주행 (단독), angle={angle:.2f}")
    else:
        angle = 10
        print("❓ 흰색 차선 미검출 → 직진 유지 or 우회전")

    drive(angle=angle, speed=base_speed)

#================================================================================================
# main 함수
#================================================================================================
def start():
    global motor, image, ranges, HEIGHT, WIDTH, Fix_Speed
    print("Start program --------------")
    rospy.init_node('Track_Driver')
    rospy.Subscriber("/usb_cam/image_raw/", Image, usbcam_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    motor = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)

    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    print("Camera Ready --------------")
    rospy.wait_for_message("/scan", LaserScan)
    print("Lidar Ready ----------")

    plt.ion()
    plt.show()
    print("Lidar Visualizer Ready ----------")

    print("======================================")
    print(" S T A R T    D R I V I N G ...")
    print("======================================")

    driving = False
    HEIGHT, WIDTH = image.shape[:2]
    
    # 라바콘 상태 관리 변수
    lava_mode = True
    lava_detected_once = False
    lava_end_time = None
    lava_exited_time = None

    # 차량 회피 모드 상태 관리 변수
    in_obstacle_mode = False
    clear_start_time = None

    while not rospy.is_shutdown():
        if image.size == 0:
            continue
        cv2.imshow("original", image)

        if ranges is None:
            continue
        
        # 1. 신호등 인식
        if detect_green_light(image):
            if not driving:
                print("초록불 감지: 출발")
                driving = True

        # 2. 주행 시작
        if driving:
            
            # 라바콘 회피 주행
            if lava_mode:
                r_mid, l_mid, m_mid, cone_mask = detect_cones_by_color(image, WIDTH/2)
                
                if len(m_mid) > 0:
                    lava_detected_once = True  #  한 번이라도 라바콘 감지되었음을 표시
                    lava_end_time = None
                    angle = lavacone_refined(image, m_mid, lateral_offset=100)
                    drive(angle, speed=6)

                    # angle = lavacone(image, m_mid, r_mid, l_mid)
                    # drive(angle, speed = 5)
                    # cv2.imshow("라바콘 HSV 마스크", cone_mask)

                else:
                    if lava_detected_once:
                        if lava_end_time is None:
                            lava_end_time = time.time()
                        elif time.time() - lava_end_time > 1.0:
                            print(" 라바콘 지역 종료 → 일반 주행 모드 전환")
                            lava_mode = False
                            lava_end_time = None
                            lava_exited_time = time.time()
                    else:
                        angle, roi = process_lane(image)
                        speed = compute_speed(angle)
                        print(f"angle: {angle:.2f}, speed: {speed:.2f}")
                        drive(angle=-angle, speed=speed)  # 조향 반전 적용
                        # cv2.imshow("ROI with lane", roi)  
            
            if not lava_mode:
                r_mid, l_mid, m_mid, cone_mask = detect_cones_by_color(image, WIDTH/2)
                if len(m_mid) > 0:
                    lava_mode = True
                else:
                    now = time.time()
                    cooldown_elapsed = lava_exited_time is None or (now - lava_exited_time > 5.0)
                    d = front_distance(ranges)
                    if d is not None:
                        print(f"전방 평균 거리: {d:.2f}m")
                    else:
                        print("전방 평균 거리: None")

                    # 장애물 X m 이내 접근 시 회피모드 진입
                    if d is not None and not in_obstacle_mode and d < 8 and cooldown_elapsed:
                        print("🚨 장애물 감지 → 회피 모드 진입")
                        in_obstacle_mode = True
                        clear_start_time = None  # 초기화
                    
                    # 회피 모드 on
                    if in_obstacle_mode:   
                        
                        # 일정 시간 이상 장애물 감지 X
                        if d >= 100 : 
                            if clear_start_time is None:
                                clear_start_time = time.time() # 진입 시간 측정 시작
                                
                                drive_by_white_lane(image)
                                # =========================================================   

                            # 장애물 인식 3초 이상 지속 -> 회피모드 종료     
                            elif time.time() - clear_start_time > 0.1:
                                print("✅ 장애물 통과 → 회피 모드 종료 ")
                                in_obstacle_mode = False
                                clear_start_time = None  # 리셋
                        else:
                            # 회피 모드 진입
                            clear_start_time = None  # d < 100으로 떨어지면 타이머 초기화
                            drive_by_white_lane(image)
                            
                    else:
                        angle, roi= process_lane(image)
                        speed = compute_speed(angle)
                        print(f"angle: {angle:.2f}, speed: {speed:.2f}")
                        drive(angle=-angle, speed=speed)  # 조향 반전 적용
        else:
            drive(angle=0.0, speed=0.0)

        time.sleep(0.1)
        cv2.waitKey(1)

if __name__ == '__main__':
    start()
