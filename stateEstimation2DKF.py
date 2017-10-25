import math
import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#--------------------------------------------------------------------------------
# Generated my sample datat in the file sampleData.csv. Next step is to copy it
# into this file as a list of sample data that has a gaussian applied to it.
# Generation of this data is in the file generateSampleData.py
#--------------------------------------------------------------------------------
def main():
	#First define time vector
	tInitial = 0
	tFinal = 10
	time=list(range(tInitial,tFinal))
	#Generated my time vector now to generate local acceleration
	#Define anglular velcoity 
	angularVelocity = [0,0]
	localAcceleration = [[0,0],[1,0]]
	#define intitial acceleration
	ag = [[0,0],[0,0]]
	#define intitial velocity
	vg = [[0,0],[0,0]]
	#define intial position
	rg = [[0,0],[0,0]]
	for i in range(2,len(time)):
		localAcceleration.append([0,0])
		angularVelocity.append(0)
		ag.append([0,0])
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
	initPred = [rg[0],
				vg[0],
				ag[0]]

	#Define gravity
	prevState = initPred
	newPred = [[0,0],
				[0,0],
				[0,0]]
	prevP = [[1, 0, 0],
			 [0, 1, 0],
			 [0, 0, 1]]
	newP = [[1, 0, 0],
			 [0, 1, 0],
			 [0, 0, 1]]
	I = [[1, 0, 0],
			 [0, 1, 0],
			 [0, 0, 1]]
	xData = []
	yData = []
	R = [[0, 0, 0],
		 [0, 0, 0],
		 [0, 0, 0]]

	Q = [[0, 0, 0],
		 [0, 0, 0],
		 [0, 0, 0]]
	for t in range(1,len(time)):
		w=angularVelocity[t]
		al=localAcceleration[t]
		#See documentation to see how to update rotation matrices
		#update yaw
		yaw = yaw+w*(time[t]-time[t-1])
		yawMatrix = [[math.cos(yaw), -math.sin(yaw)],
				 	 [math.sin(yaw), math.cos(yaw)]]
		#convert acceleration to global coordinates
		ag[t]=numpy.dot(yawMatrix,al)
		#Prediction step
		deltaT = time[t]-time[t-1]
		A = [[1, deltaT, math.pow(deltaT,2)],
			 [0, 1, deltaT],
			 [0, 0, 1]]
		newPred = numpy.dot(A,prevState)

		#Project error covariance
		Atranspose = [[1, 0 ,0],
					  [deltaT, 1, 0],
					  [math.pow(deltaT,2), deltaT, 1]]
		Pestimate = numpy.dot(A,numpy.dot(prevP,Atranspose)) + Q

		#Measurement update
		#We will make it simple by doing the math and saying H is identity
		#integrate to get velocity
		vg[t]=prevState[1]+ag[t]*deltaT
		#integrate to get position
		rg[t]=prevState[0]+vg[t]*deltaT
		H = [[1, 0, 0],
			 [0, 1, 0],
			 [0, 0, 1]]
		sensorReadings = [rg[t],
						  vg[t],
						  ag[t]]
		z = numpy.dot(H,sensorReadings)

		#Correction
		S = numpy.dot(H,numpy.dot(Pestimate,H))+R #since H = H transpose in the identity case
		K = numpy.dot(Pestimate,numpy.dot(H,numpy.linalg.inv(S)))

		y = z - numpy.dot(H,newPred)
		newState = newPred + numpy.dot(K,y)
		
		inter = I-numpy.dot(K,H)
		newP = numpy.dot(inter,Pestimate)

		prevState = newState

		xData.append(prevState[0][0])
		yData.append(prevState[0][1])

		
	#Starting Plot Section to help with Debugging of state estimation program
	
	fig = plt.figure()
	ax = fig.add_subplot(111)
	#Plot my data
	ax.plot(xData,yData, color='red')
	plt.show()

if __name__ == "__main__": main()