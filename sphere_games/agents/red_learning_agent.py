import os
import time

import numpy as np
import rospy
import argparse
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import Twist, Point, Vector3

parser = argparse.ArgumentParser(description='Process input Ros Rate Mult.')
parser.add_argument('integers', metavar='N', type=int, nargs='?',
                   help='an integer for the accumulator', default=1)
args = parser.parse_args()
ROS_RATE_MULTIPLIER=args.integers

# Global variables
red_center = Point()
red_flag = False
red_base = Point()
blue_base = Point()
game_over = False
blue_score = 0
prev_blue_score = 0
red_score = 0
prev_red_score = 0
red_twist = Twist()
Q_table = {}
yaw_actions = np.array(list(range(8))) * np.pi / 4
vel_actions = np.array(list(range(1, 2))) * 15 # One speed
center_nogo = 0
# Current Ration Inches-To-Pixels is 45in:882pixels
# Sphero Radius Value goes from 1in -> 20 pixels
sphero_radius = 20
current_game_counter = 1
start_time = time.time()
longest_time = 100.0

gridBuffer = {}
choiceBuffer = {}
buffCounter = 0

# Helper functions
def set_center(sphere_center):
    global red_center
    red_center = sphere_center
    return

def set_flag(flag_status):
    global red_flag
    red_flag = flag_status.data
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

def set_current_game_counter(game_counter):
    global current_game_counter
    current_game_counter = game_counter.data
    return

def yaw_vel_to_twist(yaw, vel):
    twist_msg = Twist()
    twist_msg.linear = Vector3(0, 0, 0)
    twist_msg.angular.x = np.cos(yaw) * vel
    twist_msg.angular.y = np.sin(yaw) * vel
    twist_msg.angular.z = 0
    return twist_msg

def parse_dict(unformatted):
    formatted = {}
    for key in unformatted.item().keys():
        formatted[key] = unformatted.item().get(key)
    return formatted

def get_heading_and_distance():
    global red_center, red_flag, red_base, blue_base
    if red_flag != False: # Have flag, go home
        target_x = red_base.x
        target_y = red_base.y
    else: # Don't have flag, go to opponent's base
        target_x = blue_base.x
        target_y = blue_base.y
    delta_x = target_x - red_center.x
    delta_y = target_y - red_center.y
    distance = np.sqrt(delta_x ** 2 + delta_y ** 2)
    #calculates distance from agent to centerpoint between the two bases (no-go zone)
    delta_distCx = ((red_base.x + blue_base.x)/2) - red_center.x
    delta_distCy = ((red_base.y + blue_base.y)/2) - red_center.y
    distCenter = np.sqrt(delta_distCx ** 2 + delta_distCy ** 2)
    heading = np.arctan2(delta_y, delta_x)
    return heading, distance, distCenter


# Agent function
def Q_learning():
    global Q_table, red_twist, yaw_actions, vel_actions, center_nogo
    global red_score, prev_red_score, blue_score, prev_blue_score
    global start_time, longest_time
    expectation = 0.

    # Resets capture time when someone scores
    if(red_score > prev_red_score or blue_score > prev_blue_score):
        prev_red_score = red_score
        prev_blue_score = blue_score
        start_time = time.time()

    # Determine Reward
    heading, distance, distCenter = get_heading_and_distance()
    
    #initial reward function, seek goal
    current_value = (1 - distance / 1250) 
    
    # avoid center capability
    if distCenter < center_nogo:
        current_value = current_value - .01 # Scale to [1, ~0], .01 is the weighting of punishing the agent from going into the center area
    
    # reward negatively impacted for longer times
    current_duration = time.time() - start_time
    if (current_duration > longest_time):
        #update longest time if it changes
        longest_time = current_duration
    current_value -= (current_value * (current_duration/longest_time))

    heading = int(4 * heading / np.pi)   # Convert to range(8)
    distance = int(8 * distance / 1250.)  # Convert to range(8)
    
    if 'previous_value' in Q_table:
        previous_value = Q_table['previous_value']
        previous_grid = Q_table['previous_grid']
        previous_choice = Q_table['previous_choice']
        reward = (current_value - previous_value) - 0.005
        Q_value = Q_table[previous_grid][previous_choice]
        Q_table[previous_grid][previous_choice] += reward

    
    if (np.random.random() < 0.2 
        or  (heading, distance) not in Q_table):
        yaw_choice = np.random.choice(yaw_actions)
        vel_choice = np.random.choice(vel_actions)
    else:
        options = Q_table[(heading, distance)].keys()
        highest = options[0]
        highest_value = -1000
        for option in options:
            option_value = Q_table[(heading, distance)][option]
            if option_value > highest_value:
                highest = option
                highest_value = option_value
        if highest_value > 0:
            print(highest_value)
            yaw_choice, vel_choice = highest
            expectation = highest_value
        else:
            yaw_choice = np.random.choice(yaw_actions)
            vel_choice = np.random.choice(vel_actions)

    if (heading, distance) not in Q_table:
        Q_table[(heading, distance)] = {}
    if (yaw_choice, vel_choice) not in Q_table[(heading, distance)]:
        Q_table[(heading, distance)][(yaw_choice, vel_choice)] = 0.
    Q_table['previous_value'] = current_value #reward
    Q_table['previous_grid'] = (heading, distance) #state
    Q_table['previous_choice'] = (yaw_choice, vel_choice) #action
    
    print("Yaw: {}, Vel: {}, Value: {}".format(yaw_choice, vel_choice, 
        current_value))
    yaw_choice = -yaw_choice # Switch from camera to world coordinates
    red_twist = yaw_vel_to_twist(yaw_choice, vel_choice)

    
    return

def Q_learningV2():
    global Q_table, red_twist, yaw_actions, vel_actions, red_score, prev_red_score, center_nogo, buffCounter, choiceBuffer, gridBuffer
    expectation = 0.

    #Chose 10 actions because program would crash if agent scores without buffer being fully populated
    #Assumed impossible to score in <2s therefore chose 10 commands @5Hz
    SIZEOFBUFFER = 10
    
    #captures scoring event
    if(red_score > prev_red_score):
        #TODO reward faster times between scores
        #TODO create circles around bad zones 
        #TODO combos and DOE to find best combination
        for entry in gridBuffer:
            Q_table[gridBuffer[entry]][choiceBuffer[entry]] += 0.1
            print('This is the new reward: ' + str(Q_table[gridBuffer[entry]][choiceBuffer[entry]]))
        prev_red_score = red_score
        
    # Determine Reward
    heading, distance, distCenter = get_heading_and_distance()
    current_value = (1 - distance / 1250) 
    if distCenter < center_nogo: # Scale
        current_value - .01 # Scale to [1, ~0], .01 is the weighting of punishing the agent from going into the center area
    
    heading = int(4 * heading / np.pi)   # Convert to range(8)
    distance = int(8 * distance / 1250.)  # Convert to range(8)
    
    if 'previous_value' in Q_table:
        previous_value = Q_table['previous_value']
        previous_grid = Q_table['previous_grid']
        previous_choice = Q_table['previous_choice']
        reward = (current_value - previous_value) - 0.005
        Q_value = Q_table[previous_grid][previous_choice]
        Q_table[previous_grid][previous_choice] += reward

    
    if (np.random.random() < 0.2 
        or  (heading, distance) not in Q_table):
        yaw_choice = np.random.choice(yaw_actions)
        vel_choice = np.random.choice(vel_actions)
    else:
        options = Q_table[(heading, distance)].keys()
        highest = options[0]
        highest_value = -1000
        for option in options:
            option_value = Q_table[(heading, distance)][option]
            if option_value > highest_value:
                highest = option
                highest_value = option_value
        if highest_value > 0:
            print(highest_value)
            yaw_choice, vel_choice = highest
            expectation = highest_value
        else:
            yaw_choice = np.random.choice(yaw_actions)
            vel_choice = np.random.choice(vel_actions)

    if (heading, distance) not in Q_table:
        Q_table[(heading, distance)] = {}
    if (yaw_choice, vel_choice) not in Q_table[(heading, distance)]:
        Q_table[(heading, distance)][(yaw_choice, vel_choice)] = 0.
    Q_table['previous_value'] = current_value #reward
    Q_table['previous_grid'] = (heading, distance) #state
    Q_table['previous_choice'] = (yaw_choice, vel_choice) #action

    #populates the buffer of size SIZEOFBUFFER
    choiceBuffer[buffCounter%SIZEOFBUFFER] = (yaw_choice, vel_choice)
    gridBuffer[buffCounter%SIZEOFBUFFER] = (heading, distance)
    buffCounter += 1

    print("Yaw: {}, Vel: {}, Value: {}".format(yaw_choice, vel_choice, 
        current_value))
    yaw_choice = -yaw_choice # Switch from camera to world coordinates
    red_twist = yaw_vel_to_twist(yaw_choice, vel_choice)

    
    return

# Init function
def learning_agent():
    # Load any existing agent
    global Q_table, game_over, red_base, blue_base, current_game_counter
     
    agent_file = 'new_test_agent.npy'
    if os.path.isfile(agent_file):
        Q_table = parse_dict(np.load(agent_file))
        print("Loaded red agent from file.")
    else:
        print("New agent started.")

    # Setup ROS message handling
    rospy.init_node('red_agent', anonymous=True)

    pub_red_cmd = rospy.Publisher('/red_sphero/twist_cmd', Twist, queue_size=1)
    sub_red_center = rospy.Subscriber('/red_sphero/center', Point, set_center, queue_size=1)
    sub_red_flag = rospy.Subscriber('/red_sphero/flag', Bool, set_flag, queue_size=1)
    sub_blue_base = rospy.Subscriber('/blue_sphero/base', Point, set_blue_base, queue_size=1)
    sub_red_base = rospy.Subscriber('/red_sphero/base', Point, set_red_base, queue_size=1)
    sub_game_over = rospy.Subscriber('/game_over', Bool, set_game_over, queue_size=1)
    sub_red_score = rospy.Subscriber('/red_sphero/score', Int16, set_red_score, queue_size=1)
    sub_blue_score = rospy.Subscriber('/blue_sphero/score', Int16, set_blue_score, queue_size=1)
    sub_current_game_counter = rospy.Subscriber('/current_game_counter', Int16, set_current_game_counter, queue_size=1)
    
    previous_game_counter = current_game_counter

    #calculate size of circular no-go zone around center obstacle
    sphero_radius = (red_base.x - blue_base.x) * (1 / 48)
    center_nogo = (red_base.x - blue_base.x)*.25/2 + sphero_radius

    # Agent control loop
    rate = rospy.Rate(5*ROS_RATE_MULTIPLIER) # Hz
    while not rospy.is_shutdown():
        Q_learning()
        pub_red_cmd.publish(red_twist)
        if current_game_counter > previous_game_counter:
            np.save(agent_file, Q_table)
            previous_game_counter = current_game_counter
        if game_over != False:
            break
        rate.sleep()

    np.save(agent_file, Q_table)
    print("Game ended. Agent saved.")
    return

if __name__ == '__main__':
    try:
        learning_agent()
    except rospy.ROSInterruptException:
        pass

