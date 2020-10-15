# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import time
import rospy
import numpy as np
from pyrobot import Robot
from absl import flags, app
import matplotlib.pyplot as plt
from dev.image_processor import Image_Processor

BOT_NAME = 'locobot'
GAIN = (5,20)
CONFIG = {'base_controller': 'ilqr', 'base_planner': 'movebase'} # not sure why but the tutorials say so


def get_time_str():
    return time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())

def wrap_angle(radians):
    return (radians + np.pi) % (2 * np.pi) - np.pi

def get_alpha_from_image_slice(image, bound1, bound2):
    u_slice = image[bound1]
    l_slice = image[bound2]
    mp = int(image.shape[1]/2)
    weight = np.zeros(2)
    init_point = 0
    edge_regions = (mp - 107, mp + 107)
    center_regions = (mp - 107, mp + 107)
    for x,d in enumerate(u_slice[1:]):
        if u_slice[init_point] != d or x == len(u_slice) - 2:
            mp = (x + init_point) / 2
            depth_relative = int(u_slice[x]) - int(l_slice[x])
            if mp < edge_regions[0]:
                weight[0] += depth_relative * (x - init_point) / GAIN[1]
            elif mp > edge_regions[1]:
                weight[0] -= depth_relative * (x - init_point) / GAIN[1]
            if center_regions[0] < mp and center_regions[1] > mp:
                weight[1] += depth_relative * (x - init_point) / GAIN[0]
            init_point = x + 1

    if np.sum(weight) == 0:
        weight = (-1,-1)
        
    turn_signal = weight[0] / np.abs(weight[0] + 1e-8)
    print(f'Accumulated weights: {weight}')
    return wrap_angle(np.arctan(weight[1]/(weight[0]+1e-8))),turn_signal

def main(_):

    robot = Robot(BOT_NAME, CONFIG)
    print('Locobot Initialized')
    current_state = robot.base.get_state('odom') # can also be 'vslam' for visual slam
    print(f'Starting at {current_state}')

    img_processor = Image_Processor()
    cp = (int(640/2),480 - 50)
    ctr = 0

    while not rospy.is_shutdown():
        """
            This loop is where you can implement your algorithm/control
            robot.base.set_vel(fwd_speed=s,turn_speed=s,exe_time=t)
            robot.base.go_to_absolute(target_position, close_loop=False, smooth=True)
        """
        image = (robot.camera.get_depth() * 1000).astype(np.int)
        alpha, turn_signal = get_alpha_from_image_slice(image, 275, 375)
        print(f'Alpha {alpha}')
        fwd_speed = np.sin(alpha)
        turn_speed = np.cos(alpha) * turn_signal
        print(f'Trajectory components <{fwd_speed}, {turn_speed}>')
        #slice_l = list(zip(np.arange(0,image.shape[1]),480-(60*image[375])))
        #slice_u = list(zip(np.arange(0,image.shape[1]),480-(60*image[275])))
        #protocol={
        #    'draw-line' : [[[cp[0],cp[1],cp[0]+(turn_speed * 100) ,cp[1]-(fwd_speed * 100)]],255,3],
        #    'plot' : [[slice_u,slice_l],[(0,0,1),(1,0,0)]]
        #}
        #items = img_processor.process(image.copy(),protocol=protocol,sequence=False)
        #img_processor.display(save=f'scripts/dev/gen/Target-Bound-{ctr}.png')
        #ctr += 1

        robot.base.set_vel(fwd_speed=fwd_speed,turn_speed=turn_speed, exe_time=1)

    robot.base.stop()


if __name__ == "__main__":
    app.run(main)
