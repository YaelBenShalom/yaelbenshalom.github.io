## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import colorsys
import math
import time
import rospy
import threading
import modern_robotics as mr

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from interbotix_sdk import angle_manipulation as ang
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from interbotix_sdk.msg import JointCommands
from interbotix_sdk.msg import SingleCommand
from interbotix_sdk.srv import RobotInfo
from interbotix_sdk.srv import OperatingModes
from interbotix_sdk.srv import OperatingModesRequest
from interbotix_sdk.srv import RegisterValues
from interbotix_sdk.srv import RegisterValuesRequest
from interbotix_descriptions import interbotix_mr_descriptions as mrd
from interbotix_sdk.robot_manipulation import InterbotixRobot


# Create a pipeline
pipeline = rs.pipeline()

#Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_record_to_file("camera_video2")
# config.enable_device_from_file("camera_video2")

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)


looking_for_pen = True

#TODO
def convert_to_cyl(position):
    x = -position[0] - 0.30  # distance of base from camera - x axis
    y = -position[1]
    z = position[2] - 0.17 # distance of base from camera - z axis
    r = math.sqrt(x**2 + z**2)
    theta = math.atan2(x, -z) #- math.pi/2
    return (r, theta, y)


#TODO
def move_to_position(position):
    arm = InterbotixRobot(robot_name = "px100", mrd = mrd)
    arm.go_to_home_pose()
    arm_position = (0.0,0.0)
    arm.open_gripper(2.0)
    arm.set_ee_pose_components(x = position[0],z = position[2], blocking = True)
    arm.set_ee_cartesian_trajectory(x = -0.05)
    arm.set_single_joint_position("waist", position[1], blocking = True)
    arm.set_ee_cartesian_trajectory(x = +0.05)

    arm.close_gripper(2.0)
    arm.set_ee_cartesian_trajectory(x = -0.05)
    arm.go_to_home_pose()
    arm.open_gripper(2.0)
    arm.go_to_sleep_pose()


# Streaming loop
try:
    while looking_for_pen:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        #TODO
        cfg = profile.get_stream(rs.stream.color)
        intrinsics = cfg.as_video_stream_profile().get_intrinsics()


        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        aligned_color_frame = aligned_frames.get_color_frame()

        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics

        # Validate that both frames are valid
        if not aligned_depth_frame or not aligned_color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(aligned_color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        # Convert between color spaces
        convert_to_hsv = cv2.cvtColor(bg_removed, cv2.COLOR_BGR2HSV)
        
        # define range of purple color in HSV
        lower_purple = np.array([110, 50, 50])
        upper_purple = np.array([130, 255, 255])
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 40])

        # Threshold the HSV image to get only purple colors
        mask_image = cv2.inRange(convert_to_hsv, lower_purple, upper_purple)
        mask_image_blur = cv2.GaussianBlur(mask_image,(5, 5), 0)
        # mask_image_bitwise = cv2.bitwise_and(mask_image_blur, mask_image_blur, mask = mask_image_blur) #TODO
        contours, hierarchy = cv2.findContours(mask_image_blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_b, hierarchy = cv2.findContours(mask_image_blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #TODO
        index = -1
        curr_area = -1
        for a in range(len(contours)):
            area = cv2.contourArea(contours[a])
            if (area > curr_area and area > 1000):
                curr_area = area
                index = a

        if len(contours) > 0 and curr_area > 1000:
            cnt = contours[index]
            cv2.drawContours(bg_removed, contours, index, (0, 255, 0), 3)

            M = cv2.moments(cnt)
            px = int(M['m10'] / M['m00'])
            py = int(M['m01'] / M['m00'])
            depth_pixel = [px, py]
            position = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth_image[py][px] * depth_scale)
            print("position", position)
            move_to_position(convert_to_cyl(position))
            # print("depth pixel", depth_pixel)
            # print("X: ", position[0],  "Y: ", position[1],"Z: ", position[2])
            looking_for_pen = False

        # depth = bg_removed[320,240].astype(float)
        # distance = depth * depth_scale
        # print("Distance (m): ", distance )
        # print("depth_image_3d (m): ", depth_image_3d )

        # Render images
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))
        cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Align Example', images)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()





# def pen_recognision(img):
#     # ima_proc = img`
#     # cv2.circle(img,(447,63), 63, (0,0,255), -1)
#     cv2.rectangle(img,(384,0),(510,128),(0,255,0),3)
#     hsvImage = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#     lower_green = np.array([0, 0, 150])
#     upper_green = np.array([255, 255, 255])

#     mask = cv2.inRange(hsvImage, lower_green, upper_green)
#     res = cv2.bitwise_and(img, img, mask=mask)

#     return res

