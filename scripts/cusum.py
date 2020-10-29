#!/usr/bin/env python

import rospy
import rosbag
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt
import pandas as pd

#############################################################

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
		for i in range(len(df[0])):
			[t,data] = df[0][i].split(" ")
			time_img.append(float(t)*(10**9)-t0) # 10**9 : convert [s] to [ns]
			img.append(float(data)/scale)

	except Exception as err:
		raise err

	return time_imu,imu_x,imu_y,imu_z,imu_gyro_x,imu_gyro_y,imu_gyro_z, time_img, img



def cusum(input,mean,std,k,SLmax,SHmax):
	Z=[input[0]]
	SL=[0.0]			#Positive detection
	SH=[0.0]   		#Negative detection
	SLIndex=[]		#
	SHIndex=[]	
	N = len(input)
	for n in range(1,N):
		Z.append((input[n]-mean)/std)	#normalize
		sh = max(0.0,SH[n-1]+Z[n-1]-k)
		sl = max(0.0,SL[n-1]-Z[n-1]-k)
		if(sh>=SHmax):
			SHIndex.append(n)
			sh=0
		if(sl>=SLmax):
			SLIndex.append(n)
			sl=0

		SH.append(sh)
		SL.append(sl)

	return SL,SH,SLIndex,SHIndex

#############################################################



## Obtain stats
bagFile =  '/home/ubuntu/static.bag' 		# change this to the bag file path
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

print('----------------------------------------------------------------')
print('imu_x: mean {},	std {} '.format(mx,sx))
print('imu_y: mean {},	std {} '.format(my,sy))
print('imu_z: mean {},	std {} '.format(mz,sz))
print('imu_gyro_x: mean {},	std {} '.format(mgx,sgx))
print('imu_gyro_y: mean {},	std {} '.format(mgy,sgy))
print('imu_gyro_z: mean {},	std {} '.format(mgz,sgz))
print('image: mean {},	std {} '.format(mi,si))
print('----------------------------------------------------------------')

plt0=plt.figure(0)
plt.plot(time_imu,imu_y,label='imu_y')
plt.plot(time_img,img,label='image')
plt.legend()
plt.grid()


## Obtain stats
bagFile =  '/home/ubuntu/fft2.bag' 		# change this to the bag file path
bagTopics = ['/mavros/imu/data'] 				# change to topics that conatine sensor_msgs/Imu message
imgSignalFile = '/home/ubuntu/usb_cam_ws/fft2.txt' 	# change this to the txt file path
										
#find value threshold
time_imu,imu_x,imu_y,imu_z,time_img,img = extract(bagFile,bagTopics,imgSignalFile,10**9)



### Cusum
SL_y,SH_y,SLIndex_y,SHIndex_y = cusum(imu_y,0.05,sy*2,1.5,5.0,5.0)
SL_img,SH_img,SLIndex_img,SHIndex_img = cusum(img,mi,si,1.5,5.0,5.0)

active_y=[]
counter=0
for i in range(len(SH_y)):
	if(counter<len(SHIndex_y) and i==SHIndex_y[counter]):
		active_y.append(1)
		counter+=1
	else:
		active_y.append(0)


active_img=[]
counter=0
for i in range(len(SH_img)):
	if(counter<len(SHIndex_y) and i==SHIndex_y[counter]):
		active_y.append(1)
		counter+=1
	else:
		active_y.append(0)




plt1=plt.figure(1)
plt.plot(time_imu,imu_y,label='imu_y')
plt.plot(time_img,img,label='image')
plt.legend()
plt.grid()

plt2=plt.figure(2)
plt.plot(time_imu,SH_y,label='imu_y')
#plt.plot(time_img,img,label='image')
plt.legend()
plt.grid()

plt3=plt.figure(3)
plt.plot(time_imu,active_y,label='imu_y')
#plt.plot(time_img,img,label='image')
plt.legend()
plt.grid()



plt.show()
