import math
import numpy
import csv

def main():
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

	mu = 0
	sigma1 = 0.1
	sigma2 = 0.01

	for i in range(0,len(time)):
		localAcceleration[i][0] += numpy.random.normal(mu,sigma1,1)
		localAcceleration[i][1] += numpy.random.normal(mu,sigma1,1)
		angularVelocity[i] += numpy.random.normal(mu,sigma2,1)
	with open('sampleGeneratedData.csv', 'wb') as csvfile:
		writer = csv.writer(csvfile, delimiter = ',', quoting=csv.QUOTE_MINIMAL)
		writer.writerow(['Angular Velocity'] + ['Local Acceleration x'] + ['Local Acceleration y'])
		for i in range(0,len(time)):
			row = []
			row.append(angularVelocity[i])
			row.append(localAcceleration[i][0])
			row.append(localAcceleration[i][1])
			writer.writerow(row)

if __name__ == "__main__": main()