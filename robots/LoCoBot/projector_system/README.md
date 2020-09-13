## Projector System

The Projector System currently consists of a catkin package resding in this directory. 


#### Basic Setup:

Currently, the system is equipped to display an animation on the display of the LoCoBot's machine that consists of a circle counting down the time until the robot reaches its navigational goal. To run this animation:

1. SSH into the LoCoBot.

2. Run `source ~/pyenv_pyrobot/bin/activate` and `export DISPLAY:=0`on all terminals you will be using. (`export DISPLAY:=0` re-routes any GUIs or windows that launch as result of you commands to launch on the LoCoBot's display)

3. [Calibrate the robot](https://pyrobot.org/docs/calibration) if needed.

4. Run `roslaunch locobot_control main.launch use_base:=true` to launch the main LoCoBot control session.

5. In a new terminal, run `roslaunch projector_system projector.launch animation:="countdown"` - this will launch the Projector System using the countdown animation. 

6. In a new terminal, run any navigational script. Once the robot begins moving, any display (monitor, projector) connected to the LoCoBot will display a Matplotlib plot that counts down the time until the robot reaches its goal!
