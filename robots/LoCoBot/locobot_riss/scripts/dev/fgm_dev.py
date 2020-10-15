#!/home/m_dyse/pyenvs/pyro3_ws/bin/python
import os
import numpy as np
import matplotlib.pyplot as plt
from image_processor import Image_Processor
from pyrobot.utils.util import try_cv2_import
cv = try_cv2_import()

GAIN = (5,30)

IM_PROTOCOL = {
	#'convert': cv.COLOR_GRAY2RGB,
	#'draw-line':[[[320,480,640,275]],10,3],
	#'crop': [[0,0.57,1,0.78],True],
	#'canny':[0,1],
	#'bitwise-and': [[2,10]],
	#'contours': [cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE]
	#'houghP-lines': [1, 1, 1,0,0]
}


def load_images(path='/home/m_dyse/workspaces/pyro_ws/src/pyrobot/robots/LoCoBot/locobot_riss/Data',ext='depth_img_'):
	im_files = [f for f in os.listdir(path) if ext in f]
	images = []
	for f in im_files:
		print(f'Reading {f}:...')
		images.append(cv.imread(os.path.join(path,f),cv.IMREAD_UNCHANGED))
		print(f'Size: {images[-1].shape}')
	return images


def get_depth_shards(slice):
	shards = []
	prev = 0
	for i,value in enumerate(slice):
		if value != slice[i-1]:
			shards.append((slice[prev:i],i-prev,prev)) # [(shard, len, start index)...]
			prev = i
	return shards

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
			print(init_point, x)
			depth_relative = int(u_slice[x]) - int(l_slice[x])
			if mp < edge_regions[0]:
				print(f'segment {mp} in left, value {depth_relative} * {(x - init_point)}')
				weight[0] -= depth_relative * (x - init_point) / GAIN[1]
			elif mp > edge_regions[1]:
				print(f'segment {mp} in right, value {depth_relative} * {(x - init_point)}')
				weight[0] += depth_relative * (x - init_point) / GAIN[1]
			if center_regions[0] < mp and center_regions[1] > mp:
				print(f'segment {mp} in center, value {depth_relative} * {(x - init_point)}')
				weight[1] += depth_relative * (x - init_point) / GAIN[0]
			init_point = x + 1
	print(f'Accumulated weights {weight}')
	return wrap_angle(np.arctan(weight[1]/(weight[0]+1e-6)))


def main():
	images = load_images()
	img_processor = Image_Processor(IM_PROTOCOL)
	cp = (int(images[0].shape[1]/2),images[0].shape[0] - 50)
	for i,image in enumerate(images):
		alpha = get_alpha_from_image_slice(image, 275, 375)
		print(f'Alpha {alpha}')
		fwd_speed = np.sin(alpha)
		turn_speed = np.cos(alpha)
		print(f'Trajectory components <{fwd_speed}, {turn_speed}>')
		slice_l = list(zip(np.arange(0,image.shape[1]),480-(60*image[375])))
		slice_u = list(zip(np.arange(0,image.shape[1]),480-(60*image[275])))
		protocol={
			'draw-line' : [[[cp[0],cp[1],cp[0]+(turn_speed * 100) ,cp[1]-(fwd_speed * 100)]],255,3],
			'plot' : [[slice_u,slice_l],[(0,0,1),(1,0,0)]]
		}
		items = img_processor.process(image.copy(),protocol=protocol,sequence=False)
		img_processor.display(save=f'scripts/dev/gen/Target-Bound-{i}.png')

	cv.destroyAllWindows()


if __name__=='__main__':
	main()