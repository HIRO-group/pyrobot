#!/home/m_dyse/pyenvs/pyro3_ws/bin/python
import os
import numpy as np
import matplotlib.pyplot as plt
try:
	from image_processor import Image_Processor
except:
	from dev.image_processor import Image_Processor
from pyrobot.utils.util import try_cv2_import
cv = try_cv2_import()

GAIN = (1e-1,1e-1) # Mostly to prevent overflow of the accumulator

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

def get_alpha_from_image_slice(image, bound1, bound2, name='[INFO]'):
	exe_time = 0.5
	# get image slices
	u_slice = image[bound1]
	l_slice = image[bound2]

    # meta-data
	mp_actual = int(image.shape[1]/2)
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
			left_gain = (1 - (mp / mp_actual)) * GAIN[0]
			center_gain = (mp / mp_actual) * GAIN[1]
            # get Interest
			edge_intrest = left_gain * (x - init_point) * (depth_relative + depth_actual - 1.5)
			center_intrest = center_gain * (x - init_point) * (depth_relative + depth_actual - 1.25)
            #update
			weight[0] += edge_intrest
			weight[1] += center_intrest
			init_point = x + 1

    # when no interests turn around
	if np.sum(weight) == 0:
		weight = [1, -0.5]
		exe_time = 5
        
	print(f'{name} Accumulated weights: {weight}')
	vector = [weight[1], weight[0]]
	print(f'{name} Non-normal vector: {vector}')
	return vector / np.linalg.norm(vector), exe_time




def main():
	images = load_images()
	img_processor = Image_Processor(IM_PROTOCOL)
	cp = (int(images[0].shape[1]/2),images[0].shape[0] - 50)
	for i,image in enumerate(images):
		fwd_speed, turn_speed = get_alpha_from_image_slice(image, 275, 375)
		print(f'Alpha: ({fwd_speed}, {turn_speed})')
		slice_l = list(zip(np.arange(0,image.shape[1]),480-(60*image[400])))
		slice_u = list(zip(np.arange(0,image.shape[1]),480-(60*image[275])))
		protocol={
			'draw-line' : [[[cp[0],cp[1],cp[0]-(np.linalg.norm(turn_speed) * 100) ,cp[1]-(np.linalg.norm(fwd_speed) * 100)]],0,3],
			'plot' : [[slice_u,slice_l],[(0,0,1),(1,0,0)]]
		}
		items = img_processor.process(image.copy(),protocol=protocol,sequence=False)
		img_processor.display(save=f'Data/Target-Bound-{i}.png')

	cv.destroyAllWindows()


if __name__=='__main__':
	main()