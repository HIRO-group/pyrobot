# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import sys
import time
import rospy
import numpy as np
from pyrobot import Robot
from absl import flags, app
import matplotlib.pyplot as plt
from dev.fgm import get_alpha_from_image_slice
from dev.image_processor import Image_Processor


BOT_NAME = 'locobot'
CONFIG = {'base_controller': 'ilqr', 'base_planner': 'movebase'} # not sure why but the tutorials say so


def get_time_str():
	return time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())    

def main(_):

	name = f'[{sys.argv[0]}]'
	visualize = False
	if 'visualize' in sys.argv:
		visualize = True

	robot = Robot(BOT_NAME, CONFIG)
	print('Locobot Initialized')
	current_state = robot.base.get_state('odom') # can also be 'vslam' for visual slam
	print(f'Starting at {current_state}')

	#planner = simple_planner(current_state)

	img_processor = Image_Processor()
	cp = (int(640/2),480 - 50)
	ctr = 0

	while not rospy.is_shutdown():
		"""
			This loop is where you can implement your algorithm/control
			robot.base.set_vel(fwd_speed=s,turn_speed=s,exe_time=t)
			robot.base.go_to_absolute(target_position, close_loop=False, smooth=True)

			currently the script imports a tinker method from fgm.py that calculates
			a heading based on the open space in front of it
		"""
		image_d = (robot.camera.get_depth() * 1000).astype(np.int)
		image_rgb = robot.camera.get_rgb()
		bound1 = 275
		bound2 = 375
		(fwd_speed, turn_speed), exe_time = get_alpha_from_image_slice(image_d, bound1, bound2, name=name)
		print(f'{name} Alpha ({fwd_speed},{turn_speed})')

		#if planner.observe(robot.base.get_state('odom')):
		 #   fwd_speed, turn_speed = -0.5, np.random.random_integers(-1, 2)
		  #  exe_time = 5

		if visualize:
			slice_u = list(zip(np.arange(0,image_d.shape[1]),480-(60*image_d[bound1])))
			slice_l = list(zip(np.arange(0,image_d.shape[1]),480-(60*image_d[bound2])))
			protocol=[
				{'draw-line' : [[[cp[0],cp[1],cp[0]-(turn_speed * 100) ,cp[1]-(fwd_speed * 100)]],(1,0,0),3]},
				{'crop' : [[0,0,1,1]],
				'plot' : [[slice_u,slice_l],[(0,0,1),(1,0,0)]]}
			]
			items = img_processor.process(image_rgb.copy(),protocol=protocol[0])
			items = img_processor.process(image_d.copy(),protocol=protocol[1],new_sequence=False,sequence=False)
			img_processor.display(save=f'scripts/dev/gen/Target-Bound-{ctr}{get_time_str()}.png')
			ctr += 1

		robot.base.set_vel(fwd_speed=fwd_speed,turn_speed=turn_speed, exe_time=exe_time)

	robot.base.stop()


if __name__ == "__main__":
	app.run(main)
