import math
import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def main():
	#First define time vector
	tInitial = 0
	tFinal = 5
	time=list(range(tInitial,tFinal))
	#Generated my time vector now to generate local acceleration
	gravity = 0
	#Define anglular velcoity 
	angularVelocity = [[0,0,0],[0,0,0]]
	localAcceleration = [[0, 0, gravity],[1,0,gravity+0]]
	#define intitial velocity
	vg = [[0,0,0],[0,0,0]]
	#define intial position
	rg = [[0,0,0],[0,0,0]]
	for i in range(2,len(time)):
		localAcceleration.append([1,0,gravity])
		angularVelocity.append([0,0,0])
		vg.append([0,0,0])
		rg.append([0,0,0])
	angularVelocity[3]=[0,1.5,0]
	#Define intial roll, pitch, and yaw
	roll = 0
	pitch = 0
	yaw = 0

	rollMatrix = [[1, 0, 0],
				  [0, math.cos(roll), -math.sin(roll)],
				  [0, math.sin(roll), math.cos(roll)]]
	pitchMatrix = [[math.cos(pitch), 0, math.sin(pitch)],
				   [0, 1, 0],
				   [-math.sin(pitch), 0, math.cos(pitch)]]
	yawMatrix = [[math.cos(yaw), -math.sin(yaw), 0],
				 [math.sin(yaw), math.cos(yaw), 0],
				 [0, 0, 1]]
	rotationMatrix = numpy.dot(rollMatrix,pitchMatrix)
	rotationMatrix = numpy.dot(rotationMatrix, yawMatrix)

	identity=[[1, 0, 0],
			  [0, 1, 0],
			  [0, 0, 1]]

	#Define gravity
	for t in range(1,len(time)):
		w=angularVelocity[t]
		al=localAcceleration[t]

		previousRoll = roll
		previousPitch = pitch
		previousYaw = yaw

		#Update my roll, pitch and yaw
		thetaDot = [w[0],w[1],w[2]]
		#Update theta dot
		thetaDot=numpy.dot(rotationMatrix,thetaDot)
		roll = roll + thetaDot[0]*(time[t]-time[t-1])
		pitch = pitch + thetaDot[1]*(time[t]-time[t-1])
		yaw = yaw + thetaDot[2]*(time[t]-time[t-1])

		rollDiff = roll-previousRoll
		pitchDiff = pitch-previousPitch
		yawDiff = yaw-previousYaw

		#Update my Rotation matrices
		rollMatrix = [[1, 0, 0],
				  	  [0, math.cos(roll), -math.sin(roll)],
				  	  [0, math.sin(roll), math.cos(roll)]]
		pitchMatrix = [[math.cos(pitch), 0, math.sin(pitch)],
				   	   [0, 1, 0],
				   	   [-math.sin(pitch), 0, math.cos(pitch)]]
		yawMatrix = [[math.cos(yaw), -math.sin(yaw), 0],
				 	 [math.sin(yaw), math.cos(yaw), 0],
				     [0, 0, 1]]
		#Update overall rotation matrix
		rotationMatrix = numpy.dot(rollMatrix,pitchMatrix)
		rotationMatrix = numpy.dot(rotationMatrix, yawMatrix)
		#convert acceleration to global coordinates
		ag=numpy.dot(rotationMatrix,al)
		#Remove gravity term from ag
		ag[2]=ag[2]-gravity
		#integrate to get velocity
		vg[t]=vg[t-1]+ag*(time[t]-time[t-1])
		#integrate to get position
		rg[t]=rg[t-1]+vg[t]*(time[t]-time[t-1])
	#Next step is to figure out how to plot the position so that I can test variation in the code
	#Starting Plot Section to help with Debugging of state estimation program
	xData = []
	yData = []
	zData = []
	for i in range(0,len(time)):
		xData.append(rg[i][0])
		yData.append(rg[i][1])
		zData.append(rg[i][2])
	print(zData)
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	#Make x,y,z axes on the plot
	#Plot my x axis
	xAxis1 = [0,1,2,3,4,5]
	xAxis2 = [0,0,0,0,0,0]
	xAxis3 = [0,0,0,0,0,0]
	ax.plot(xAxis1,xAxis2,xAxis3, color='green')
	#Plot my y axis
	yAxis1 = [0,0,0,0,0,0]
	yAxis2 = [0,1,2,3,4,5]
	yAxis3 = [0,0,0,0,0,0]
	ax.plot(yAxis1,yAxis2,yAxis3, color='green')
	#Plot my z axis
	zAxis1 = [0,0,0,0,0,0]
	zAxis2 = [0,0,0,0,0,0]
	zAxis3 = [0,1,2,3,4,5]
	ax.plot(zAxis1,zAxis2,zAxis3, color='green')
	#Plot my data
	ax.plot(xData,yData,zData, color='red')
	plt.show()

if __name__ == "__main__": main()