#!/usr/bin/env python

import rospy
import rosbag
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt
import pandas as pd

#############################################################

# read imu data from the bag file and read the image data from the txt file
# return the extracted data list
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


# direct filtering
# input 	input data
# lb,ub 	threshold
def filter(input,lb,ub,):
	data_filtered = map(lambda x: 0 if (x>=lb and x<=ub) else 1, input)
	return data_filtered


# cusum
# input 		input data
# mean  		expected mean value 
# std   		expected std
# k 			weight
# SLmax,SHmax 	threshold 
def cusum(input,mean,std,k,SLmax,SHmax):
	Z=[input[0]]
	SL=[0.0]			#Positive detection
	SH=[0.0]   		#Negative detection
	SLIndex=[]		#
	SHIndex=[]
	data_filtered=[0]
	N = len(input)
	for n in range(1,N):
		Z.append((input[n]-mean)/std)	#normalize
		sh = max(0.0,SH[n-1]+Z[n]-k)
		sl = max(0.0,SL[n-1]-Z[n]-k)

		data_filtered.append(1 if(sh>SHmax or sl>SLmax) else 0)
		
		#reset value
		if(sh>SHmax): sh=0
		if(sl>SLmax): sl=0

		SH.append(sh)
		SL.append(sl)

	return data_filtered


# find all the time index when the signal change from 0 to 1
def findActivateTime(time,input,threshold):
	t=[]
	active=False
	inactiveTime=0 # prevent falsely detecing quick changing as an "activation"
	for i in range(len(input)):
		if((input[i]!=0) and (not active) and (inactiveTime>=threshold)):
			t.append(time[i])
			active=True
			inactiveTime=0
		else:
			active=False
			inactiveTime+=1

	return t

# assume input2 is slower than input 1 and less noisy
def computeDelay(input1,input2,threshold):
	n1=len(input1)
	n2=len(input2)
	i=0
	j=0
	delay=[]
	while(j<n2 and i<n1):
		d=input2[j]-input1[i]
		if(d<0):
			j+=1
		elif(d<=threshold): # match
			delay.append(d)
			i+=1
			j+=1
		else:
			i+=1

	return delay


bagFile =  '/home/ubuntu/gyro/static.bag' 		# change this to the bag file path
bagTopics = ['/mavros/imu/data'] 				# change to topics that conatine sensor_msgs/Imu message
imgSignalFile = '/home/ubuntu/usb_cam_ws/static.txt' 	# change this to the txt file path
										
# find value threshold
time_imu,imu_x,imu_y,imu_z,imu_gyro_x,imu_gyro_y,imu_gyro_z, time_img, img = extract(bagFile,bagTopics,imgSignalFile,10**9)

# stat
[mgz,sgz] = [np.mean(imu_gyro_z),np.std(imu_gyro_z)]
[gzlb,gzub] = stats.norm.interval(0.999, loc=mgz, scale=sgz)
[mi,si] = [np.mean(img),np.std(img)]
[ilb,iub] = stats.norm.interval(0.999, loc=mi, scale=si)


bagFile =  '/home/ubuntu/gyro/d1.bag' 		# change this to the bag file path
bagTopics = ['/mavros/imu/data'] 				# change to topics that conatine sensor_msgs/Imu message
imgSignalFile = '/home/ubuntu/usb_cam_ws/d1.txt' 	# change this to the txt file path
										
#find value threshold
time_imu,imu_x,imu_y,imu_z,imu_gyro_x,imu_gyro_y,imu_gyro_z, time_img, img = extract(bagFile,bagTopics,imgSignalFile,10**9)

#gz1 = filter(imu_gyro_z,gzlb,gzub)
imu_gyro_z_new = cusum(imu_gyro_z,mgz,sgz,10,5.0,5.0)
img_new = cusum(img,mi,si,10,5.0,5.0)

t_gz=findActivateTime(time_imu,imu_gyro_z_new,100)
t_img=findActivateTime(time_img,img_new,100)

delay=computeDelay(t_gz,t_img,0.5*(10**9))
delay=map(lambda x: x/(10**9),delay)
[mean_delay,std_delay] = [np.mean(delay),np.std(delay)]

print('------------------------------------------------')
print('number of samples: {}\nmean:{:.5f} [s]\nnormalized std:{:.5f}'.format(len(delay),mean_delay,std_delay/mean_delay))
print('------------------------------------------------')


# raw data
plt.plot(time_imu,imu_x,label='raw accel x',marker='.')
plt.plot(time_imu,imu_gyro_z,label='raw gyro z',marker='.')
plt.plot(time_img,img,label='raw image',marker='.')
# cusum data
#plt.plot(time_imu,imu_gyro_z_new,label='cusum gyro z')
#plt.plot(time_img,img_new,label='cusum image')

# activate time plot
for xt in t_gz:
    plt.plot([xt,xt],[0,1],color='r')

for xt in t_img:
    plt.plot([xt,xt],[0,1],color='g')

plt.xlabel('time [ns]')
plt.ylabel('val [-]')
plt.legend()
plt.grid()
plt.show()


# cusum method




