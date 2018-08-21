Hello!

Welcome to you're own GPU backed VM in the cloud provided to you by the Lockheed Martin 2018 Autonomy Hackathon Team!

First thing first, this is an "on your own time" effort. Lockheed is providing the VM's but beyond that, you're on your own here. Please do not use this VM to upload, generate, copy or do anything at all with any LMPI, ECI or any other sensitive data outlined in your command media.

This effort is all in the spirit of learning and FUN, please don't break any rules or these types of efforts will not be allowed in the future and that is the opposite of what we want.

This VM has some basic programs (ROS, Gazebo etc) installed and will be updated when we deem necesarry. Keep in mind this is not a production environment so some things might not work exactly as expected. If you want additional programs installed, that is a possibility. Bring your request to the forums and our team will discuss it.

You're not intended to have sudo or root access, so if you find a security flaw, please do the right thing and immediately report it to cary.j.caruthers@lmco.com and arantza.e.flores@lmco.com

We will do our best to make sure your work and experience here is maintained. However, these VM's get expensive fast! This machine is not guaranteed to persist! Please use your personal github account (yes this is approved, but remember no LM sensitive data should be on these machines) to store your code. If you leave you're work open, don't save, and step away from the computer, it may be automatically shut down if it is "inactive" for an hour and you will loose your work. You may also use our provided GitLab instance, built specifically for this effort. It can be found at lm-hackathon-2018-gitlab.global.lmco.com.

It appears that we will have users from across the globe participating in this effort, if there are any firewall rules (or anything else) that prevent anyone from accessing something required, please let us know on the hackathon forums and we will address it if and when we can.


Quick start:
On the destkop, you will find a couple of scripts to run from the terminal (start-CTF.sh and launch-gazebo-empty-world.sh)
Feel free to analyze them, and what they do.
start-CTF.sh will start a simple machine learning algorithm created by Christopher Aasted. Details and instructions can be found on his Github here https://github.com/caasted/sphere_games
launch-gazebo-empty-world.sh will launch an empty gazebo environment
to open a terminal window: hit 'Ctl+Alt+T' on your keyboard or find it in the apps menu
to run the start-CTF.sh, type each line in the terminal window then hit 'enter':
cd ~/Desktop
./start-CTF.sh
when you run the script, it will open a few more terminals and Gazebo and you should see a red and blue "agent" start rolling around in a simulated environment. These agents are learning and optimizing their ability to get points as defined in the CTF program. Refer to the Github page for more information.

Good luck and have fun!