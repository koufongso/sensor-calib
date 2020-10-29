#!/usr/bin/env python

import rospy
import rosbag
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt
import pandas as pd

bagFile =  '/home/ubuntu/static.bag'
bagTopics = ['/mavros/imu/data']
imgSignalFile = '/home/ubuntu/usb_cam_ws/static.txt'
scale = 10**9


step=0
t0 = 0

time_imu=[]
imu_x = []
imu_y =[]	
imu_z =[]
imu_gyro_x = []
imu_gyro_y =[]	
imu_gyro_z =[]

time_img=[]
img_diff=[]


try:
	bag = rosbag.Bag(bagFile)

	for topic, msg, t in bag.read_messages(topics=bagTopics):
		if(step==0):
			t0 = msg.header.stamp.to_nsec();
			step+=1;

		imu_x.append(msg.linear_acceleration.x)
		imu_y.append(msg.linear_acceleration.y)
		imu_z.append(msg.linear_acceleration.z)
		time_imu.append(msg.header.stamp.to_nsec()-t0)

	bag.close()

except Exception as e:
	print("Error: Could not open "+bagFile)
	raise e


try:
	df = pd.read_csv(imgSignalFile, sep=" \n", header=None,engine="python")
	for i in range(len(df[0])):
		[t,data] = df[0][i].split(" ")
		time_img.append(float(t)*(10**9)-t0)
		img_diff.append(float(data)/scale)
except Exception as e:
	print("Error: Could not open "+imgSignalFile)
	raise e

[mx,sx] = [np.mean(imu_x),np.std(imu_x)]
[my,sy] = [np.mean(imu_y),np.std(imu_y)]
[mz,sz] = [np.mean(imu_z),np.std(imu_z)]
[mi,si] = [np.mean(img_diff),np.std(img_diff)]


print('x: mean {},	std {}'.format(mx,sx))
print('y: mean {},	std {}'.format(my,sy))
print('z: mean {},	std {}'.format(mz,sz))
print('image: mean {},	std {}'.format(mi,si))


# find 3 std interval
[xlb,xub] = stats.norm.interval(0.999, loc=mx, scale=sx)

[ylb,yub] = stats.norm.interval(0.999, loc=my, scale=sy)

[zlb,zub] = stats.norm.interval(0.999, loc=mz, scale=sz)


# plt.plot(time_imu,imu_x,label="accel_x")
plt.plot(time_imu,imu_y,label="accel_y")
# plt.plot(time_imu,imu_z,label="accel_z")
time_img.pop(0);
img_diff.pop(0);
plt.plot(time_img,img_diff,label="img_signal")

plt.xlabel("time[nsec]")
plt.ylabel("linear_acceleration[g], pixel_difference[-]  ")
plt.legend()
plt.grid()
plt.show()
