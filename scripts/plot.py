#!/usr/bin/env python

import rospy
import rosbag
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt
import pandas as pd

def extract(BagFile,topics, imageDataFile,scale):
	scale = 10**9 # scale down the img value (typical value = 10^9)

	step=0
	t0 = 0.0

	time_imu=[]
	imu_x = []
	imu_y =[]	
	imu_z =[]
	imu_gyro_x = []
	imu_gyro_y =[]	
	imu_gyro_z =[]

	time_img=[]
	img=[]

	try:
		bag = rosbag.Bag(BagFile)
		for topic, msg, t in bag.read_messages(topics=bagTopics):
			if(step==0):
				t0 = msg.header.stamp.to_nsec();
				step+=1;

			time_imu.append(msg.header.stamp.to_nsec()-t0)
			imu_x.append(msg.linear_acceleration.x)
			imu_y.append(msg.linear_acceleration.y)
			imu_z.append(msg.linear_acceleration.z)
			imu_gyro_x.append(msg.angular_velocity.x)
			imu_gyro_y.append(msg.angular_velocity.y)
			imu_gyro_z.append(msg.angular_velocity.z)

		bag.close()

	except Exception as err:
		raise err


	try:
		df = pd.read_csv(imageDataFile, sep=" \n", header=None,engine="python")
		for i in range(1,len(df[0])):
			[t,data] = df[0][i].split(" ")
			time_img.append(float(t)*(10**9)-t0) # 10**9 : convert [s] to [ns]
			img.append(float(data)/scale)

	except Exception as err:
		raise err

	return time_imu,imu_x,imu_y,imu_z,imu_gyro_x,imu_gyro_y,imu_gyro_z, time_img, img

## Obtain stats
bagFile =  '/home/ubuntu/gyro/static.bag' 		# change this to the bag file path
bagTopics = ['/mavros/imu/data'] 				# change to topics that conatine sensor_msgs/Imu message
imgSignalFile = '/home/ubuntu/usb_cam_ws/static.txt' 	# change this to the txt file path
										
#find value threshold
time_imu,imu_x,imu_y,imu_z,imu_gyro_x,imu_gyro_y,imu_gyro_z, time_img, img = extract(bagFile,bagTopics,imgSignalFile,10**9)
[mx,sx] = [np.mean(imu_x),np.std(imu_x)]
[my,sy] = [np.mean(imu_y),np.std(imu_y)]
[mz,sz] = [np.mean(imu_z),np.std(imu_z)]
[mgx,sgx] = [np.mean(imu_gyro_x),np.std(imu_gyro_x)]
[mgy,sgy] = [np.mean(imu_gyro_y),np.std(imu_gyro_y)]
[mgz,sgz] = [np.mean(imu_gyro_z),np.std(imu_gyro_z)]
[mi,si] = [np.mean(img),np.std(img)]

[gzlb,gzub] = stats.norm.interval(0.999999, loc=mgz, scale=sgz)
[ilb,iub] = stats.norm.interval(0.9999999999999, loc=mi, scale=si)

print('----------------------------------------------------------------')
print('imu_x: mean {},	std {}'.format(mx,sx))
print('imu_y: mean {},	std {}'.format(my,sy))
print('imu_z: mean {},	std {}'.format(mz,sz))
print('imu_gyro_x: mean {},	std {}'.format(mgx,sgx))
print('imu_gyro_y: mean {},	std {}'.format(mgy,sgy))
print('imu_gyro_z: mean {},	std {} [{},{}]'.format(mgz,sgz,gzlb,gzub))
print('image: mean {},	std {} [{},{}]'.format(mi,si,ilb,iub))
print('----------------------------------------------------------------')


plt0 = plt.figure(0)
plt.plot(time_imu,imu_gyro_z,label="gyro_z",marker='.')
plt.plot(time_img,img,label="img_signal",marker='.')
plt.xlabel("time[nsec]")
plt.ylabel("signal [-]")
plt.legend()
plt.grid()


bagFile =  '/home/ubuntu/gyro/d1.bag' 		# change this to the bag file path
bagTopics = ['/mavros/imu/data'] 				# change to topics that conatine sensor_msgs/Imu message
imgSignalFile = '/home/ubuntu/usb_cam_ws/d1.txt' 	# change this to the txt file path
										
#find value threshold
time_imu,imu_x,imu_y,imu_z,imu_gyro_x,imu_gyro_y,imu_gyro_z, time_img, img = extract(bagFile,bagTopics,imgSignalFile,10**9)


active_gz=[]

for i in range(len(imu_gyro_z)):
	val = imu_gyro_z[i]
	if(val<gzlb or val>gzub):
		active_gz.append(1)
	else:
		active_gz.append(0)


active_img=[]
counter=0
for i in range(len(img)):
	val = img[i]
	if(val>0.002):
		active_img.append(1)
	else:
		active_img.append(0)




# plot graph
plt1 = plt.figure(1)
plt.plot(time_imu,imu_gyro_z,label="gyro_z",marker='.')
plt.plot(time_img,img,label="img_signal",marker='.')
plt.xlabel("time[nsec]")
plt.ylabel("signal [-]")
plt.legend()
plt.grid()

plt2 = plt.figure(2)
plt.plot(time_imu,active_gz,label="gyro_z",marker='.')
plt.plot(time_img,active_img,label="img_signal",marker='.')
plt.xlabel("time[nsec]")
plt.ylabel("signal [-]")
plt.legend()
plt.grid()

plt.show()