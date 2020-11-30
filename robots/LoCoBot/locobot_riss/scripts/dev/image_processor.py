#!/home/m_dyse/pyenvs/pyro3_ws/bin/python
import numpy as np
import matplotlib.pyplot as plt
from pyrobot.utils.util import try_cv2_import
cv = try_cv2_import()


def get_end_points(line, x1, x2):
	y1 = int(line[0] * x1 + line[1])
	y2 = int(line[0] * x2 + line[1])
	return (x1, y1), (x2, y2)


class Image_Processor():
	def __init__(self, protocol={}, resolution=(480, 640)):
		self.protocol = protocol
		self.width = resolution[0]
		self.height = resolution[1]
		if len(resolution) > 2:
			self.channels = resolution[2]
		else:
			self.channels = -1
		self.results = []
		self.returns = {'frames':0}

	def display(self, save=False):
		cols = np.min([4,len(self.results)])
		rows = np.max([int(len(self.results) / cols + .9),1])
		step = 0
		fig, axes = plt.subplots(rows, cols, figsize=(12,7))
		if rows > 1:
			for r in range(rows):
				for c in range(cols):
					axes[r][c].imshow(self.results[step][0])
					axes[r][c].set_title(self.results[step][1])
					step += 1
					if step >= len(self.results):
						break
		else:
			for c in range(cols):
				axes[c].imshow(self.results[c][0])
				axes[c].set_title(self.results[c][1])

		plt.tight_layout(pad=0.4, w_pad=0.5, h_pad=0.5)
		plt.show()
		if save:
			fig.savefig(save)
		cv.waitKey()

	def process(self, frame, protocol={}, sequence=True, new_sequence=True):
		if new_sequence or len(self.results) < 1:
			self.results = [(frame,'original-0')]
		self.width = frame.shape[1]
		self.height = frame.shape[0]
		self.returns['frames'] = 0
		if len(protocol) == 0:
			protocol = self.protocol

		for step in protocol:
			self.returns['frames'] += 1
			if sequence:
				source = self.results[-1][0].copy()
			else:
				source = frame.copy()

			if step == 'crop': # 'crop': [bounds]
				y_min = int(protocol[step][0][1] * self.height)
				y_max = int(protocol[step][0][3] * self.height)
				x_min = int(protocol[step][0][0] * self.width)
				x_max = int(protocol[step][0][2] * self.width)
				self.width = x_max - x_min
				self.height = y_max - y_min
				self.results.append((source[y_min:y_max,x_min:x_max],f'Cropped-{len(self.results)}'))

			if step == 'convert': # 'convert': cv.COLOR_*
				self.results.append((cv.cvtColor(source, protocol[step]),f'Convert-{len(self.results)}'))

			if step == 'bilateral-blur': # 'bilateral-blur': [bound1, bound2, bound3]
					self.results.append((cv.bilateralFilter(source, protocol[step][0], protocol[step][1], protocol[step][2]),f'Bilateral-Blur-{len(self.results)}'))

			if  step == 'bitwise-and': # 'bitwise_and': [bound1, bound2... boundn]
				""" Generate masks of source in a range boundn
					Masks are Xor'd together then &'d with the source """
				mask = cv.inRange(source, np.array(protocol[step][0][0]), np.array(protocol[step][0][1]))
				for bound in protocol[step][1:]:
					mask = mask ^ cv.inRange(source, np.array(bound[0]), np.array(bound[1]))
				#self.results.append((mask,f'Image_Mask-{len(self.results)}'))
				# apply masks
				self.results.append((cv.bitwise_and(source, source, mask=mask),f'Bitwise-And-{len(self.results)}'))

			if step == 'canny': # 'canny': [bound1, bound2 ]
				self.results.append((cv.Canny(source, protocol[step][0], protocol[step][1]),f'Cannies-{len(self.results)}'))

			if step == 'houghP-lines': # 'houghP-lines' : [houghp_args]
				if self.channels == -1:
					temp_img = cv.cvtColor(source, cv.COLOR_GRAY2RGB)
				else:
					temp_img = source
				line_img = np.zeros((self.height, self.width, 3),dtype=np.uint8)
				lines = cv.HoughLinesP(source, protocol[step][0], protocol[step][1], protocol[step][2], protocol[step][3])
				for x1,y1,x2,y2 in lines[0]:
					cv.line(line_img, (x1,y1), (x2,y2), (0,255,0))
				self.results.append((cv.addWeighted(temp_img, 0.8, line_img, 1.0, 0.0),f'Houghp lines-{len(self.results)}'))
				self.returns[step] = lines

			if step == 'draw-line': # 'draw-line': [[[x,y,x,y],[...]],color,thickness]
				annotated_img = source.copy()
				for x1,y1,x2,y2 in protocol[step][0]:
					start_point = (int(x1),int(y1))
					end_point = (int(x2),int(y2))
					cv.line(annotated_img, start_point, end_point, protocol[step][1], protocol[step][2])
				self.results.append((annotated_img,f'Draw-Line-{len(self.results)}'))

			if step == 'draw-circle': # 'draw_circle' : [[(x,y)...],radius,color,thickness]
				annotated_img = source.copy()
				for point in protocol[step][0]:
					cv.circle(annotated_img, point, protocol[step][1], protocol[step][2], protocol[step][3])
				self.results.append((annotated_img, f'Draw-Circle-{len(self.results)}'))

			if step == 'contours': # 'contours': [return style, approx style]
				if self.channels == -1 :
					temp_img = cv.cvtColor(self.results[-1][0], cv.COLOR_GRAY2RGB)
				else:
					temp_img = source
				contours, hierarchy = cv.findContours(source,protocol[step][0],protocol[step][1])
				self.results.append((cv.drawContours(temp_img, contours, -1, (0,255,0), 3),f'Contours-{len(self.results)}'))
				self.returns[step] = (contours, hierarchy)

			if step == 'plot': # 'plot' [[[(x,y)...],[(x,y)...]],[(r,g,b),(r,g,b)..], depth]
				annotated_img = np.zeros((protocol[step][2], self.width, 3))
				for i,plot in enumerate(protocol[step][0]):
					for j,point in enumerate(plot[1:]):
						cv.line(annotated_img, plot[j], point, protocol[step][1][i], 10)
				self.results.append((annotated_img, f'Plot-{len(self.results)}'))


		return self.returns
