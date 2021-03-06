import numpy as np
import rospy
import random
import argparse
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import Point, Twist, Vector3

parser = argparse.ArgumentParser(description='Process input Ros Rate Mult.')
parser.add_argument('integers', metavar='N', type=int, nargs='?',
                   help='an integer for the accumulator', default=1)
args = parser.parse_args()
ROS_RATE_MULTIPLIER=args.integers

# Global variables
blue_center = Point()
blue_flag = False
blue_base = Point()
red_base = Point()
blue_twist = Twist()
blue_score = 0
red_score = 0
prevBlueScore = 0
game_over = False
accumulated_error = 0.
neutral_zone = False
#initializes path of opponent agent to one of two policies: (lower neutral zone -> 1) (upper neutral zone -> 2)
neutral_route = random.randint(1,2)

# Helper functions
def set_center(sphere_center):
    global blue_center
    blue_center = sphere_center
    return

def set_flag(flag_status):
    global blue_flag, neutral_zone
    # Logic for needing to go back through neutral zone 
    if blue_flag != flag_status.data:
        neutral_zone = False
    blue_flag = flag_status.data
    return

def set_game_over(game_state):
    global game_over
    game_over = game_state.data
    return

def set_blue_base(base):
    global blue_base
    blue_base = base
    return

def set_red_base(base):
    global red_base
    red_base = base
    return

def set_red_score(score):
    global red_score
    red_score = score.data
    return

def set_blue_score(score):
    global blue_score
    blue_score = score.data
    return

def yaw_vel_to_twist(yaw, vel): 
    twist_msg = Twist() 
    twist_msg.linear = Vector3(0, 0, 0) 
    twist_msg.angular.x = np.cos(yaw) * vel 
    twist_msg.angular.y = np.sin(yaw) * vel 
    twist_msg.angular.z = 0 
    return twist_msg



def get_heading_and_distance():
    global blue_center, blue_flag, blue_base, red_base, neutral_zone
    if neutral_zone and blue_flag:
        # Have flag, go home
        target_x = blue_base.x
        target_y = blue_base.y
    elif not blue_flag and (neutral_zone != False):
        # Don't have flag, go to opponent's base
        target_x = red_base.x
        target_y = red_base.y
    else:
        if neutral_route == 1:

            #Haven't passed through (lower) neutral zone, go there
            target_x = (0.75 * (max(blue_base.x, red_base.x) 
                            - min(blue_base.x, red_base.x)) 
                            + min(blue_base.x, red_base.x))
            target_y = (0.75 * (max(blue_base.y, red_base.y) 
                            - min(blue_base.y, red_base.y)) 
                            + min(blue_base.y, red_base.y))
        else:
            #Haven't passed through (upper) neutral zone, go there
            target_x = (0.25 * (max(blue_base.x, red_base.x) 
                            - min(blue_base.x, red_base.x)) 
                            + min(blue_base.x, red_base.x))
            target_y = (0.25 * (max(blue_base.y, red_base.y) 
                            - min(blue_base.y, red_base.y)) 
                            + min(blue_base.y, red_base.y))

    delta_x = target_x - blue_center.x
    delta_y = target_y - blue_center.y
    print("[{}, {}]".format(delta_x, delta_y))
    
    distance = np.sqrt(delta_x ** 2 + delta_y ** 2)
    if not neutral_zone and distance < 50:
        neutral_zone = True
    heading = np.arctan2(delta_y, delta_x)
    return heading, distance

# Agent function
def proportional_control():
    global blue_twist, accumulated_error

    if blue_center != Point():
        heading, distance = get_heading_and_distance()
        heading = -heading # Switch from camera to world coordinates
        if distance < 100:
            accumulated_error = 0
        else:
            accumulated_error += distance
        speed = distance / 100. + accumulated_error / 10000.
    else:
        speed = 0
        heading = 0
    blue_twist = yaw_vel_to_twist(heading, speed)
    return

# Init function
def simple_agent():
    global game_over, neutral_route, blue_score, prevBlueScore
    # Setup ROS message handling
    rospy.init_node('blue_agent', anonymous=True)

    pub_blue_cmd = rospy.Publisher('/blue_sphero/twist_cmd', Twist, queue_size=1)
    sub_blue_center = rospy.Subscriber('/blue_sphero/center', Point, set_center, queue_size=1)
    sub_blue_flag = rospy.Subscriber('/blue_sphero/flag', Bool, set_flag, queue_size=1)
    sub_blue_base = rospy.Subscriber('/blue_sphero/base', Point, set_blue_base, queue_size=1)
    sub_red_base = rospy.Subscriber('/red_sphero/base', Point, set_red_base, queue_size=1)
    sub_game_over = rospy.Subscriber('/game_over', Bool, set_game_over, queue_size=1)
    sub_red_score = rospy.Subscriber('/red_sphero/score', Int16, set_red_score, queue_size=1)
    sub_blue_score = rospy.Subscriber('/blue_sphero/score', Int16, set_blue_score, queue_size=1)
    
    
    # Agent control loop
    rate = rospy.Rate(2*ROS_RATE_MULTIPLIER) # Hz
    while not rospy.is_shutdown():
        proportional_control()
        pub_blue_cmd.publish(blue_twist)
        print("\n Blue score:" + str(blue_score) + " datatype: " + str(type(blue_score)))
        print("\n prevBlueScore:" + str(prevBlueScore) + " datatype: " + str(type(prevBlueScore)))
        #checks if blue scores to randomly change the next neutral_route

        if blue_score > prevBlueScore:
            neutral_route = random.randint(1,2)
            print("\nAfter randomization:" + str(neutral_route))
            prevBlueScore = blue_score  
        if game_over != False:
            break
        rate.sleep()
    print("Game ended. No agent to save.")
    return

if __name__ == '__main__':
    try:
        simple_agent()
    except rospy.ROSInterruptException:
        pass

