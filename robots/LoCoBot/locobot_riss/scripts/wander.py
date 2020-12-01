# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import sys
import rospy
import numpy as np
import dev.riss as Riss
from pyrobot import Robot
from absl import flags, app

BOT_NAME = 'locobot'
CONFIG = {'base_controller': 'ilqr', 'base_planner': 'movebase'} # not sure why but the tutorials say so


def main(_):
	name = f'[{sys.argv[0]}]'
	visualize = False
	verbose = False
	bound1 = 275
	bound2 = 375
	sim = False

	if 'visualize' in sys.argv:
		visualize = True
	if 'verbose' in sys.argv:
		verbose = True
	if 'sim' in sys.argv:
		sim = True

	robot = Robot(BOT_NAME, CONFIG)
	print('Locobot Initialized')
	current_state = robot.base.get_state('odom') # can also be 'vslam' for visual slam
	print(f'Starting at {current_state}')

	image = robot.camera.get_depth()
	riss = Riss.RISS(current_state, image.shape, bound1, bound2, verbose=verbose, visualize=visualize)

	while not rospy.is_shutdown():
		"""
			This loop is where you can implement your algorithm/control
			robot.base.set_vel(fwd_speed=s,turn_speed=s,exe_time=t)
			robot.base.go_to_absolute(target_position, close_loop=False, smooth=True)

			currently the script imports a tinker method from fgm.py that calculates
			a heading based on the open space in front of it
		"""
		if sim:
			image_d = (robot.camera.get_depth() * 1000).astype(np.int)
		else:
			image_d = (robot.camera.get_depth()).astype(np.int)
		image_rgb = robot.camera.get_rgb()

		(fwd_speed, turn_speed), exe_time = riss.get_heading(image_d, image_rgb, robot.base.get_state('odom'), name=name)
		print(f'{name} Alpha ({fwd_speed},{turn_speed})')

		robot.base.set_vel(fwd_speed=fwd_speed,turn_speed=turn_speed, exe_time=exe_time)

	robot.base.stop()


if __name__=="__main__":
	app.run(main)
