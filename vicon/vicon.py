from pyvicon_datastream import tools
import numpy as np
from math import *
import time

VICON_TRACKER_IP = "192.168.50.56"
OBJECT_NAME = "drone1"

mytracker = tools.ObjectTracker(VICON_TRACKER_IP)
while(True):
	position = mytracker.get_position(OBJECT_NAME)
	if position:
		try:
			latency, frame_num, object_pos = position
			# subject_name, segment_name, px, py, pz, rx, ry, rz = tuple(object_pos)
						
			print(f"Position:\n {object_pos[0][2]} \n {object_pos[0][3]} \n {object_pos[0][4]}")
			print(f"Orientation:\n {object_pos[0][5]} \n {object_pos[0][6]} \n {object_pos[0][7]}")
			
		except:
			pass


##############################################################
	
position = (
	0.008402954307524205, 
	17283, 
	[
		['test1', 
		'test1', 
		-156.68384024693685, 
		-2406.4368530387587, 
		21.29743389121431, 
		0.003559562936943974, 
		0.0011852637513303677, 
		-0.0002947714419617846
		]
	]
	)

# my_tracker.get_position(OBJECT_NAME) =>
# (
# latency, 
# frame_number, 
# 	[
#	subject_name,
# 	segment_name,
# 	pos_x,
# 	pos_y,
# 	pos_z,
# 	euler_x,
# 	euler_y,
# 	euler_z
# ])




