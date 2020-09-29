## Projector System

The Projector System currently consists of a catkin package residing in this directory. 


#### Basic Setup:

Currently, the system is equipped to display an animation on the display of the LoCoBot's machine that consists of a circle counting down the time until the robot reaches its navigational goal. To run this animation:

1. SSH into the LoCoBot.

2. Run `source ~/pyenv_pyrobot/bin/activate` and `export DISPLAY:=0`on all terminals you will be using. (`export DISPLAY:=0` re-routes any GUIs or windows that launch as result of you commands to launch on the LoCoBot's display)

3. [Calibrate the robot](https://pyrobot.org/docs/calibration) if needed.

4. Run
      ```
      roslaunch locobot_control main.launch use_base:=true
      ```
     
      to launch the main LoCoBot control session.

5. In a new terminal, run 

      ```
      roslaunch projector_system projector.launch animation:="countdown"
      ```

    This will launch the Projector System using the countdown animation. If processes have been launched correctly, the terminal will resemble the following:
    
    ```
    process[main_projector-1]: started with pid [9882]
    process[countdown_node-2]: started with pid [9883]
    ```
    
    
6. In a new terminal, run any navigational script (`navigation_example.py` located in the `scripts/` directory of the`projector_system` package is a simple example). Once the robot begins moving, any display (monitor, projector) connected to the LoCoBot will display a Matplotlib plot that counts down the time until the robot reaches its goal!




### Possible Issues and Errors

* **Error**: `main.launch` terminal session displays an error resembling:

    ```
    [ERROR] [1601392634.885228008]: Kobuki : malformed sub-payload detected. [45][170][2D AA 55 4D 01 0F ]

    ```

* **Answer**: We're unsure about why this error runs, but so far, it hasn't affected the capabilities of the robot in this project!


* **Error**: a launch or control terminal session displays an error indicating there is no module name Pyrobot:

    ```
    Traceback (most recent call last):
    File "/home/locobot/low_cost_ws/src/pyrobot/robots/LoCoBot/projector_system/scripts/main_projector.py", line 5, in <module>
      from pyrobot import Robot
    ImportError: No module named pyrobot
    ```

* **Answer**: Make sure to run `source ~/pyenv_pyrobot/bin/activate` on each terminal session. Running it should fix this error. 


* **Error**: a launch or control terminal session displays an error resembling:

    ```
    _tkinter.TclError: no display name and no $DISPLAY environment variable
    ```

* **Answer**: Run `export DISPLAY:=0` on the terminal. This command seems to have some lasting power between sessions (i.e. you don't always need to run it for each new terminal/work session) but to avoid errors, it's helpful to simply run it for each terminal alongside `source ~/pyenv_pyrobot/bin/activate` command. 





