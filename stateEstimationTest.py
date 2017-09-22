import math
import numpy

def main():
	#First define time vector
	time=list(range(0,11))
	#Generated my time vector now to generate local acceleration
	#Define anglular velcoity 
	angularVelocity = [[0,0,0],[0,0,0]]
	localAcceleration = [[0, 0, 0],[1,1,10.8]]
	#define intitial velocity
	vg = [[0,0,0],[0,0,0]]
	#define intial position
	rg = [[0,0,0],[0,0,0]]
	for i in range(2,11):
		localAcceleration.append([0,0,9.8])
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

	#Define gravity
	g = 9.8
	for t in range(1,len(time)):
		w=angularVelocity[t]
		al=localAcceleration[t]
		#Calculate my rotaional velocity matrix
		omega=[[0,-w[2],w[1]],
			   [w[2],0,-w[0]],
			   [-w[1],w[0],0]]
		#Update the rotation matrix
		rotationMatrix=rotationMatrix+numpy.dot(rotationMatrix,omega*(time[t]-(time[t-1])))
		#convert acceleration to global coordinates
		ag=numpy.dot(rotationMatrix,al)
		#Remove gravity term from ag
		ag[2]=ag[2]-9.8
		#integrate to get velocity
		vg[t]=vg[t-1]+ag*(time[t]-time[t-1])
		#integrate to get position
		rg[t]=rg[t-1]+vg[t]*(time[t]-time[t-1])
	#Next step is to figure out how to plot the position so that I can test variation in the code
if __name__ == "__main__": main()