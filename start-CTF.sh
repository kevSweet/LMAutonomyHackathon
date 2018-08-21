#!/bin/bash
lxterminal --command="bash -i -c 'roscore'"
sleep 3
lxterminal --command="bash -i -c 'source /usr/share/gazebo/setup.sh && vglrun rosrun gazebo_ros gazebo /home/hackathon-user/Desktop/samples/sphere_games/arenas/ctf_1v1_arena.world --verbose'"
sleep 5
lxterminal --command="bash -i -c 'source ~/python2_env/bin/activate && python /home/hackathon-user/Desktop/samples/sphere_games/host/sim_tracker.py'"
sleep 1
lxterminal --command="bash -i -c 'source ~/python2_env/bin/activate && python /home/hackathon-user/Desktop/samples/sphere_games/agents/red_learning_agent.py'"
sleep 1
lxterminal --command="bash -i -c 'source ~/python2_env/bin/activate && python /home/hackathon-user/Desktop/samples/sphere_games/agents/blue_simple_agent.py'"

