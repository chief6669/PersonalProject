import math
import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
def main():
	#First define time vector
	tInitial = 0
	tFinal = 10
	time=list(range(tInitial,tFinal))
	#Generated my time vector now to generate local acceleration
	#Define anglular velcoity 
	angularVelocity = [0,0]
	localAcceleration = [[0,0],[1,0]]
	#define intitial velocity
	vg = [[0,0],[0,0]]
	#define intial position
	rg = [[0,0],[0,0]]
	for i in range(2,len(time)):
		localAcceleration.append([0,0])
		angularVelocity.append(0)
		vg.append([0,0])
		rg.append([0,0])
	angularVelocity[3]=1.5
	localAcceleration[3]=[1,0]
	#Define yaw
	yaw = 0

	#See documentation about rotation matrix theory
	
	yawMatrix = [[math.cos(yaw), -math.sin(yaw)],
				 [math.sin(yaw), math.cos(yaw)]]

	identity=[[1, 0],
			  [0, 1]]
	#This is to debug angles later, used to retrieve Euler angles from rotation matrices
	angles = []

	#Define gravity
	for t in range(1,len(time)):
		w=angularVelocity[t]
		al=localAcceleration[t]
		#See documentation to see how to update rotation matrices
		#update yaw
		yaw = yaw+w*(time[t]-time[t-1])
		yawMatrix = [[math.cos(yaw), -math.sin(yaw)],
				 	 [math.sin(yaw), math.cos(yaw)]]
		#convert acceleration to global coordinates
		ag=numpy.dot(yawMatrix,al)
		#integrate to get velocity
		vg[t]=vg[t-1]+ag*(time[t]-time[t-1])
		#integrate to get position
		rg[t]=rg[t-1]+vg[t]*(time[t]-time[t-1])
	#Starting Plot Section to help with Debugging of state estimation program
	xData = []
	yData = []
	for i in range(0,len(time)):
		xData.append(rg[i][0])
		yData.append(rg[i][1])
	fig = plt.figure()
	ax = fig.add_subplot(111)
	#Plot my data
	ax.plot(xData,yData, color='red')
	plt.show()

if __name__ == "__main__": main()