import numpy as np

if __name__ == "__main__":
	positions = np.load('/home/igor/src/catkin_ws/src/px4_controller/rrt_pruning_smoothing/Code/Results/rrt_path_coords.npy', allow_pickle=True)
	
	f = open('pos.txt', 'w+')
	for p in positions:
		f.write(str(p) + '\n')
	f.close()
