#!/home/m_dyse/pyenvs/pyro3_ws/bin/python
# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
import numpy as np
import matplotlib.pyplot as plt
try:
	from image_processor import Image_Processor
except:
	from dev.image_processor import Image_Processor


def get_time_str():
	return time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime()) 


class simple_planner():
	def __init__(self, start_point, max_steps=200, max_trace=10, reset_threshold=1e-3):
		self.origin = start_point
		self.memory = np.array([])
		self.tracer = np.array([])
		self.max_steps = max_steps
		self.max_trace = max_trace
		self.reset_threshold=reset_threshold

	def observe(self, state, trajectory):
		if self.memory.shape[0] >= self.max_steps:
			self.memory = np.concatenate([self.memory[1:],state])
		else:
			self.memory = np.concatenate([self.memory, state])

		if self.tracer.shape[0] >= self.max_trace:
			self.tracer = np.concatenate([self.tracer[1:], trajectory])
		else:
			self.tracer = np.concatenate([self.tracer, trajectory])

		if self.memory.shape[0] > 4:
			if self.memory[-1] - self.memory[-2] - self.memory[-3] > self.reset_threshold:
				return True 

		if self.tracer.shape[0] > 4:
			if self.tracer[-1] - self.tracer[-2] - self.tracer[-3] > self.reset_threshold:
				return True 

		return False

	def display_path(self):
		fig = plt.figure()
		ax = fig.add_subplot()
		ax.plot(self.memory[:][:2])
		ax.set_title('Pose over time')
		plt.show()



class RISS():

	def __init__(self, start_point, image_resolution, high, low, gains=(1e-1,1e-1), memory_length=200, verbose=False, visualize=False):
		self.gain = gains # Mostly to prevent overflow of the accumulator
		self.planner = simple_planner(start_point) # simple path tracking and adjustments
		self.resolution = image_resolution
		self.processor = Image_Processor()
		self.high = high
		self.low = low
		self.verbose = verbose
		self.visualize = visualize

	def get_alpha_from_image_slice(self, u_slice, l_slice, name='[INFO]'):
		""" This function takes two slices of an image and returns a 
			heading towards the most open space.
			@params:
				- u_slice : the upper slice of the image (usually from px:275)
				- l_slice : the lower slice of the image (usually from px:375)
				- name : String to make print log more readable
			@returns:
				- vector : normal vector (forward speed, turn_speed)
				- exe_time : duration to execute the trajectory
		"""
		exe_time = 0.25

		# meta-data
		mp_actual = int(u_slice.shape[0]/2)
		weight = np.zeros(2)
		init_point = 0

		# compute intrest of sections
		for x,d in enumerate(u_slice[1:]):
			# if found section
			if u_slice[init_point] != d or x == len(u_slice) - 1:
				mp = (x + init_point) / 2	# midpoint of section
				depth_relative = int(u_slice[x]) - int(l_slice[x]) # difference between slices
				depth_actual = int(u_slice[x]) + int(l_slice[x]) / 2 # average depth of two slices
				# weight of sections
				left_gain = (1 - (mp / mp_actual)) * self.gain[0]
				center_gain = (mp / mp_actual) * self.gain[1]
				# get Interest
				edge_intrest = left_gain * (x - init_point) * (depth_relative + depth_actual - 1.5)
				center_intrest = center_gain * (x - init_point) * (depth_relative + depth_actual - 1.25)
				#update
				weight[0] += edge_intrest
				weight[1] += center_intrest
				init_point = x + 1

		# when no interests turn around
		if np.sum(weight) == 0:
			weight = [100, -1]
			exe_time = np.random.rand() * 3
			
		
		vector = [weight[1], weight[0]]

		if self.verbose:
			print(f'{name} Accumulated weights: {weight}')
			print(f'{name} Non-normal vector: {vector}')

		return vector / np.linalg.norm(vector), exe_time

	def get_heading(self, image_d, image_rgb, state, name='[INFO]'):
		u_slice = image_d[self.high]
		l_slice = image_d[self.low]

		(fwd_speed, turn_speed), exe_time = self.get_alpha_from_image_slice(u_slice, l_slice, name=name)

		if self.planner.observe(state, (fwd_speed, turn_speed)):
			fwd_speed = -0.1
			turn_speed = np.random.choice([-1, 1])
			exe_time = 3

		if self.visualize:
			cp = np.array(self.resolution / 2).astype(np.int)
			protocol=[
				{'draw-line' : [[[cp[0],cp[1],cp[0]-(turn_speed * 100 * exe_time) ,cp[1]-(fwd_speed * 100 * exe_time)]],(1,0,0),3]},
				{'crop' : [[0,0,1,1]],
				'plot' : [[u_slice, l_slice],[(0,0,1),(1,0,0)]]}
				]
			_ = self.processor.process(image_rgb.copy(), protocol=protocol[0])
			_ = self.processor.process(image_d.copy(), protocol=protocol[1], new_sequence=False, sequence=False)
			self.processor.display(save=f'scripts/dev/gen/img-processor-{ctr}{get_time_str()}.png')
