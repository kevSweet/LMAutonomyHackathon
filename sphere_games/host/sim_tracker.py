import time

import numpy as np
import rospy
import cv2
import argparse

from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point

#Constants
NUM_GAMES = 1
GAME_LENGTH = 36180

parser = argparse.ArgumentParser(description='Process input Ros Rate Mult.')
parser.add_argument('integers', metavar='N', type=int, nargs='?',
                   help='an integer for the accumulator', default=1)
args = parser.parse_args()
ROS_RATE_MULTIPLIER=args.integers

# Capture the Flag base configuration
print("Starting Capture the Flag")
red_base = Point(1400, 98, 0)
blue_base = Point(518, 979, 0)

red_center = Point(0, 0, 0)
blue_center = Point(0, 0, 0)
red_flag = False
blue_flag = False
red_score = 0
blue_score = 0
red_start_time = time.time()

def print_game_results_tofile():
    global red_score, blue_score
    filename = 'results.txt'
    with open(filename, 'a') as filehandle:
        if blue_score > red_score:
            filehandle.write("Winner: Blue Team ")
        elif blue_score < red_score:
            filehandle.write("Winner: Red Team ")
        else:
            filehandle.write("Draw! ")
        filehandle.write("Final Score - Red: {}, Blue: {}\n".format(red_score, blue_score))

def print_time_to_score_tofile():
    global red_start_time
    filename = 'time_to_score.txt'
    with open(filename, 'a') as filehandle:
        filehandle.write("{}\n".format(time.time() - red_start_time))

def reset_red_start_time():
    global red_start_time
    red_start_time = time.time()

def receive_image(image_data):
    global red_center, blue_center

    image_array = np.fromstring(image_data.data, np.uint8)
    cv2_image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

    # Mask by hue and find center
    hsv = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)

    red_lower_1 = np.array([0, 50, 50])
    red_upper_1 = np.array([int(1.0 * 180. / 6.), 255, 255])
    red_mask_1 = cv2.inRange(hsv, red_lower_1, red_upper_1)

    red_lower_2 = np.array([int(5.5 * 180. / 6.), 50, 50])
    red_upper_2 = np.array([255, 255, 255])
    red_mask_2 = cv2.inRange(hsv, red_lower_2, red_upper_2)

    red_mask = red_mask_1 + red_mask_2
    red_mask = cv2.erode(red_mask, None, iterations=2)
    red_mask = cv2.dilate(red_mask, None, iterations=2)

    blue_lower = np.array([int(3.0 * 180. / 6.), 50, 50])
    blue_upper = np.array([int(4.5 * 180. / 6.), 255, 255])
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    blue_mask = cv2.erode(blue_mask, None, iterations=2)
    blue_mask = cv2.dilate(blue_mask, None, iterations=2)

    red_contours = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(red_contours) > 0:
        red_M = cv2.moments(max(red_contours, key=cv2.contourArea))
        red_center = Point(int(red_M['m10'] / red_M['m00']), int(red_M['m01'] / red_M['m00']), 0)

    blue_contours = cv2.findContours(blue_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(blue_contours) > 0:
        blue_M = cv2.moments(max(blue_contours, key=cv2.contourArea))
        blue_center = Point(int(blue_M['m10'] / blue_M['m00']), int(blue_M['m01'] / blue_M['m00']), 0)

    return

# Scoring logic
def host():
    global red_center, blue_center, red_base, blue_base
    global red_flag, blue_flag, red_score, blue_score
    red_at_away = False
    red_at_home = False
    blue_at_away = False
    blue_at_home = False

    if red_flag != False:
        distance = np.sqrt((red_center.x - red_base.x) ** 2 + 
                           (red_center.y - red_base.y) ** 2)
        if distance < 70:
            red_at_home = True
    else:
        distance = np.sqrt((red_center.x - blue_base.x) ** 2 + 
                           (red_center.y - blue_base.y) ** 2)
        if distance < 70:
            red_at_away = True

    if blue_flag != False:
        distance = np.sqrt((blue_center.x - blue_base.x) ** 2 + 
                           (blue_center.y - blue_base.y) ** 2)
        if distance < 70:
            blue_at_home = True
    else:
        distance = np.sqrt((blue_center.x - red_base.x) ** 2 + 
                           (blue_center.y - red_base.y) ** 2)
        if distance < 70:
            blue_at_away = True

    if red_at_home and blue_at_home:
        red_score += 1
        blue_score += 1
        red_flag = False
        blue_flag = False
        print_time_to_score_tofile()
        reset_red_start_time()
    elif red_at_home:
        red_score += 1
        red_flag = False
        blue_flag = False
        print_time_to_score_tofile()
        reset_red_start_time()
    elif blue_at_home:
        blue_score += 1
        red_flag = False
        blue_flag = False
        reset_red_start_time()
    else:
        if red_at_away:
            red_flag = True
        if blue_at_away:
            blue_flag = True
    return

def pub_sub_init():
    global red_center, blue_center, red_flag, blue_flag, red_score, blue_score
    current_game_counter = 1

    pub_red_center = rospy.Publisher('/red_sphero/center', Point, queue_size=1)
    pub_blue_center = rospy.Publisher('/blue_sphero/center', Point, queue_size=1)
    pub_red_base = rospy.Publisher('/red_sphero/base', Point, queue_size=1)
    pub_blue_base = rospy.Publisher('/blue_sphero/base', Point, queue_size=1)
    pub_red_flag = rospy.Publisher('/red_sphero/flag', Bool, queue_size=1)
    pub_blue_flag = rospy.Publisher('/blue_sphero/flag', Bool, queue_size=1)
    pub_red_score = rospy.Publisher('/red_sphero/score', Int16, queue_size=1)
    pub_blue_score = rospy.Publisher('/blue_sphero/score', Int16, queue_size=1)
    pub_current_game_counter = rospy.Publisher('/current_game_counter', Int16, queue_size=1)

    pub_game_over = rospy.Publisher('/game_over', Bool, queue_size=1)

    sub_image = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, receive_image, queue_size=1)

    rospy.init_node('sphere_tracker', anonymous=True)

    rate = rospy.Rate(10 * ROS_RATE_MULTIPLIER) # Hz
    start = time.time()
    while not rospy.is_shutdown():
        host()
        pub_red_center.publish(red_center)
        pub_blue_center.publish(blue_center)
        pub_red_base.publish(red_base)
        pub_blue_base.publish(blue_base)
        pub_red_flag.publish(red_flag)
        pub_blue_flag.publish(blue_flag)
        pub_red_score.publish(red_score)
        pub_blue_score.publish(blue_score)
        pub_game_over.publish(False)

        print("Time: {} / {}".format((time.time() - start), GAME_LENGTH))
        print("Red: [{}, {}], [{}, {}]".format(red_center.x, red_center.y, 
            red_flag, red_score))
        print("Blue: [{}, {}], [{}, {}]".format(blue_center.x, blue_center.y, 
            blue_flag, blue_score))

        if time.time() - start > GAME_LENGTH:
            print_game_results_tofile()
            if current_game_counter >= NUM_GAMES:
                pub_game_over.publish(True)
                break
            else:
                current_game_counter += 1
                red_score = 0
                blue_score = 0
                start = time.time()

        rate.sleep()

    if blue_score > red_score:
        print("Winner: Blue Team")
    elif blue_score < red_score:
        print("Winner: Red Team")
    else:
        print("Draw!")
    print("Final Score - Red: {}, Blue: {}".format(red_score, blue_score))

    return

if __name__ == '__main__':
    try:
        pub_sub_init()
    except rospy.ROSInterruptException:
        pass

