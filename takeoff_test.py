#!/usr/bin/env python3

########IMORTS#########

import rospy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import sys
import time
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from array import array

#######VARIABLES########

# vehicle = connect('tcp:127.0.0.1:5760',wait_ready=True)
vehicle = connect('127.0.0.1:14550')
velocity = -0.5  # m/s
takeoff_height = 5  # m
########################
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)
P_gain = 1.0
I_gain = 1.0
D_gain = 1.0
id_to_find = 72  # arucoID
marker_size = 20  # CM

horizontal_res = 640
vertical_res = 480

horizontal_fov = 62.2 * (math.pi / 180)  # 62.2 for picam V2, 53.5 for V1
vertical_fov = 48.8 * (math.pi / 180)  # 48.8 for V2, 41.41 for V1

found_count = 0
notfound_count = 0

#############CAMERA INTRINSICS#######


#####
time_last = 0
time_to_wait = 0.1  # 100 ms
################FUNCTIONS###############


def arm_and_takeoff(targetHeight):
    while vehicle.is_armable != True:
        print('Waiting for vehicle to become armable')
        time.sleep(1)
    print('Vehicle is now armable')

    vehicle.mode = VehicleMode('GUIDED')

    while vehicle.mode != 'GUIDED':
        print('Waiting for drone to enter GUIDED flight mode')
        time.sleep(1)
    print('Vehicle now in GUIDED mode. Have Fun!')

    vehicle.armed = True
    while vehicle.armed == False:
        print('Waiting for vehicle to become armed.')
        time.sleep(1)
    print('Look out! Virtual props are spinning!')

    vehicle.simple_takeoff(targetHeight)

    while True:
        print('Current Altitude: %d' % vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= .95 * targetHeight:
            break
        time.sleep(1)
    print('Target altitude reached!')

    return None


# def send_land_message(x, y):
#     msg = vehicle.message_factory.landing_target_encode(
#         # int(time.time() * 1e6),  # time_usec (microseconds)
#         0,
#         0,  # target_num
#         # mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
#         16,
#         x,  # angle_x (radians)
#         y,  # angle_y (radians)
#         0,  # distance (meters)
#         0,  # size_x (normalized, e.g. 0.3 for 30%)
#         0,  # size_y (normalized, e.g. 0.3 for 30%)
#         # mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,  # target_type
#         # True  # position_valid
#     )
#     vehicle.send_mavlink(msg)
#     vehicle.flush()
#     print(f"Landing target message sent: x={x}, y={y}")   not operate

def send_land_message(x, y):
    
        
    msg = vehicle.message_factory.landing_target_encode(
        int(time.time() * 1e6),  # time_usec (microseconds)
        0,  # target_num
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        x,  # angle_x (radians)
        y,  # angle_y (radians)
        0,  # distance (meters)
        0,  # size_x (normalized, e.g. 0.3 for 30%)
        0,  # size_y (normalized, e.g. 0.3 for 30%)
        mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,  # target_type
        0  # position_valid
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    print(f"Landing target message sent: x={x}, y={y}")
        
        # 명령 ACK 수신 대기
        
def go_move():
    msg = vehicle.message_factory.mission_item_int_encode(
        0, 0, 0, 0, 16, 2, 0, 0, 0, 0, 0, -353621474, 1491651746, 10
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    # while True:
    #     print('Not There Yet')
    #     if (vehicle.location.global_relative_frame.lat == -35.3621474 and vehicle.location.global_relative_frame.lon == -149.1651746):
    #         break
          

# def land():
#     msg2 = vehicle.message_factory.mission_item_int_encode(
#         0, 0, 0, 0, 21, 2, 0, 0, 0, 0, 0, -353621474, 1491651746, 10
#     )
#     vehicle.send_mavlink(msg2)########

#     vehicle.flush()







if __name__ == '__main__':
    try:
        arm_and_takeoff(takeoff_height)
        time.sleep(10)
        go_move()
        # time.sleep(30)
        # land()
        # time.sleep(1)
        # subscriber()
        # send_land_message(-35.362566, 149.165031)
    except rospy.ROSInterruptException:
        pass