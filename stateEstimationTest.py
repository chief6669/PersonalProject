import math
import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
def main():
	#First define time vector
	tInitial = 0
	tFinal = 10
	time=list(range(tInitial,tFinal))
	# define gravity
	gravity = 0
	#Generated my time vector now to generate local acceleration
	#Define anglular velcoity 
	angularVelocity = [[0,0,0],[0,0,0]]
	localAcceleration = [[0, 0, gravity],[1,0,gravity]]
	#define intitial velocity
	vg = [[0,0,0],[0,0,0]]
	#define intial position
	rg = [[0,0,0],[0,0,0]]
	for i in range(2,len(time)):
		localAcceleration.append([0,0,gravity])
		angularVelocity.append([0,0,0])
		vg.append([0,0,0])
		rg.append([0,0,0])
	angularVelocity[3]=[0,1.5,0]
	localAcceleration[3]=[1,0,gravity]
	#Define intial roll, pitch, and yaw. Have yet to understand how to intiate these using a sensor (Magnetometer?)
	roll = 0
	pitch = 0
	yaw = 0

	#See documentation about rotation matrix theory
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
	#This is to debug angles later, used to retrieve Euler angles from rotation matrices
	angles = []

	#Define gravity
	for t in range(1,len(time)):
		w=angularVelocity[t]
		al=localAcceleration[t]
		#See documentation to see how to update rotation matrices
		#Calculate my rotaional velocity matrix
		omega=[[0,-w[2],w[1]],
			   [w[2],0,-w[0]],
			   [-w[1],w[0],0]]
		#Update the rotation matrix
		omega=omega*(time[t]-time[t-1])
		intermediate = numpy.add(omega,identity)
		#print intermediat
		rotationMatrix=numpy.dot(rotationMatrix,intermediate)
		#Normalize Matrix
		rotationMatrix=numpy.divide(rotationMatrix,numpy.cbrt(numpy.linalg.det(rotationMatrix)))
		#retrieve euler angles from rotation matrix
		e1=math.atan2(rotationMatrix[2][1],rotationMatrix[2][2])
		e2=math.atan2(-rotationMatrix[2][0],math.sqrt(math.pow(rotationMatrix[2][1],2)+math.pow(rotationMatrix[2][2],2)))
		e3=math.atan2(rotationMatrix[1][0],rotationMatrix[0][0])
		angles.append([e1,e2,e3])
		#convert acceleration to global coordinates
		ag=numpy.dot(rotationMatrix,al)
		#Remove gravity term from ag
		ag[2]=ag[2]-gravity
		#integrate to get velocity
		vg[t]=vg[t-1]+ag*(time[t]-time[t-1])
		#integrate to get position
		rg[t]=rg[t-1]+vg[t]*(time[t]-time[t-1])
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
	#Make x,y,z axes on the plot
	#Plot my x axis
	xAxis1 = list(range(0,11))
	xAxis2 = [0]*11
	xAxis3 = [0]*11
	ax.plot(xAxis1,xAxis2,xAxis3, color='green')
	#Plot my y axis
	yAxis1 = [0]*11
	yAxis2 = list(range(0,11))
	yAxis3 = [0]*11
	ax.plot(yAxis1,yAxis2,yAxis3, color='green')
	#Plot my z axis
	zAxis1 = [0]*11
	zAxis2 = [0]*11
	zAxis3 = list(range(0,11))
	ax.plot(zAxis1,zAxis2,zAxis3, color='green')
	#Plot my data
	ax.plot(xData,yData,zData, color='red')
	plt.show()

if __name__ == "__main__": main()