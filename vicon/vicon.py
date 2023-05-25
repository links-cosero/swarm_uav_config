from pyvicon_datastream import tools
import time

VICON_TRACKER_IP = "192.168.50.56"
OBJECT_NAME = "test1"

mytracker = tools.ObjectTracker(VICON_TRACKER_IP)
while(True):
    position = mytracker.get_position(OBJECT_NAME)
    print(f"Position: {position}")
    # time.sleep(0.5)
    
# Position: (0.008402954307524205, 17283, [['test1', 'test1', -156.68384024693685, -2406.4368530387587, 21.29743389121431, 0.003559562936943974, 0.0011852637513303677, -0.0002947714419617846]])
