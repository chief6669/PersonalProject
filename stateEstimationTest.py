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
	localAcceleration = [[0, 0, gravity],[1,1,gravity+1]]
	#define intitial velocity
	vg = [[0,0,0],[0,0,0]]
	#define intial position
	rg = [[0,0,0],[0,0,0]]
	for i in range(2,len(time)):
		localAcceleration.append([0,0,gravity])
		angularVelocity.append([0,0,0])
		vg.append([0,0,0])
		rg.append([0,0,0])
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
		#Calculate my rotaional velocity matrix
		omega=[[0,-w[2],w[1]],
			   [w[2],0,-w[0]],
			   [-w[1],w[0],0]]
		#Update the rotation matrix
		omega=omega*(time[t]-time[t-1])
		intermediate = numpy.add(omega,identity)
		#print intermediate
		rotationMatrix=numpy.dot(rotationMatrix,intermediate)
		#Normalize Matrix
		rotationMatrix=numpy.divide(rotationMatrix,numpy.cbrt(numpy.linalg.det(rotationMatrix)))
		print(numpy.linalg.det(rotationMatrix))
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
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.plot(xData,yData,zData)
	plt.show()

if __name__ == "__main__": main()