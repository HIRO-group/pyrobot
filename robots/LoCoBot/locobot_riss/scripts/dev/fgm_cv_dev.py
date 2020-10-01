#!/home/m_dyse/pyenvs/pyro3_ws/bin/python
import os
import numpy as np
import matplotlib.pyplot as plt
from image_processor import Image_Processor
from pyrobot.utils.util import try_cv2_import
cv = try_cv2_import()

IM_PROTOCOL = {
	#'convert': cv.COLOR_GRAY2RGB,
	'draw-line':[[[0,275,640,275],[0,375,640,375]],10,3],
	'crop': [[0,0.57,1,0.78],True],
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


def display_depth_of_target_bound(image,bound1,bound2,id=0):
	fig,axes = plt.subplots(2,1,figsize=(5,10))
	axes[0].imshow(image)
	axes[0].set_title('Source Image')
	h1, = axes[1].plot(np.linspace(0,image.shape[1]-1,image.shape[1]).astype(np.int),image[bound1],color='b')
	h2, = axes[1].plot(np.linspace(0,image.shape[1]-1,image.shape[1]).astype(np.int),image[bound2],color='r')
	axes[1].set_title('Top Down Depth Estimate of Target region')
	axes[1].set_xlabel('X-Value (pixels)')
	axes[1].set_ylabel('Depth (m)')
	plt.tight_layout()
	plt.legend([h1,h2],['UpperBound','LowerBound'])
	plt.show()
	fig.savefig(f'scripts/dev/gen/Depth-plot-{id}.png')
	cv.waitKey()


def main():
	images = load_images()
	img_processor = Image_Processor(IM_PROTOCOL)
	for i,image in enumerate(images):
		items = img_processor.process(image.copy(),sequence=False)
		img_processor.display(save=f'scripts/dev/gen/Target-Bound-{i}.png')
		display_depth_of_target_bound(image,275,375,id=i)
	cv.destroyAllWindows()


if __name__=='__main__':
	main()
