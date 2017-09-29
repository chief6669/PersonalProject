import numpy as np
import matplotlib.pyplot as plt
import random
import math

#GPS, IMU, compass. (Pressure sensor, altimeter)
class KalmanFilter:

	# Constructor for Kalman Filter
	# initialState : needs to be a row vector, and initialCovariance needs to be a square matrix with
	#			     n = len(initialState)
	# modelTrust   : represents the weight you assign to your prediction step in the KF.
	# ts           : time step -- how quickly is a state updated.  
	
	def __init__(self, initialState, initialCovariance, modelTrust = 0.1, ts = 0.1, numStates = 15):
		self.initialState      = initialState
		self.initialCovariance = initialCovariance
		self.modelTrust        = modelTrust
		self.ts 			   = ts
		self.numStates         = numStates

		# Assuming a constant acceleration model
		#					   x, y, z, p, t, p,xd,yd,zd,pd,td,pd, xdd, ydd, zdd
		self.stateTransitionMat = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (0.5 * ts**2), 0, 0],\
							  [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (0.5 * ts**2), 0],\
							  [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (0.5 * ts**2)],\
							  [0, 0, 0, 1, 0, 0, 0, 0, 0, ts, 0, 0, 0,            0, 0],\
							  [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, ts, 0, 0,            0, 0],\
							  [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, ts, 0,            0, 0],\
							  [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,  0, ts,           0, 0],\
							  [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,  0,  0,           ts,0],\
							  [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,  0,  0,           0, ts],\
							  [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,  0,  0,           0, 0],\
							  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,  0,  0,           0, 0],\
							  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  1,  0,           0, 0],\
							  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  1,           0, 0],\
							  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,           1, 0],\
							  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,           0, 1]])

		# process noise represents the faith we have in our motion model.
		# Same as the Q matrix in Probabalistic Robotics.
		self.processNoise = np.dot(self.modelTrust , np.identity(self.numStates))

		# Obervation Matrix (H) represents how the measurement maps to our
		# sensor readings. The rows represent the respective sensors, and
		# the columns represent the different states.
		self.observationMat = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
						  [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
						  [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
						  [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
						  [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
						  [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
						  [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],\
						  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],\
						  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],\
						  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],\
						  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],\
						  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1]])

	# This is a linear model simple Kalman Filter Algorithm implementation.
	def algorithm(self, previousState, previousCovariance, measurement, \
				  measurementError):

		ts = self.ts
		
		# We are making the assumption that the sensor errors are uncorrelated
		measurementNoise = np.dot(measurementError.T , np.identity(len(measurementError)))

		# Prediction Step
		predictedState   = np.dot(self.stateTransitionMat , previousState.T)
		
		predictedCovar   = np.dot(np.dot(self.stateTransitionMat , previousCovariance) ,\
						   self.stateTransitionMat.T) + self.processNoise

		# Compute the Kalman Gain
		toInvert    = np.dot(np.dot(self.observationMat , predictedCovar) , self.observationMat.T) + measurementNoise
		inverted    = np.linalg.inv(toInvert)
		kalmanGain  = np.dot(np.dot(predictedCovar , self.observationMat.T) , inverted)

		# Estimated state and covariance after measurement update
		estimatedState      = predictedState + np.dot(kalmanGain , (measurement - np.dot(self.observationMat , predictedState)))
		estimatedCovariance = np.dot(np.identity(self.numStates) - np.dot(kalmanGain , self.observationMat) \
							  , predictedCovar)

		return (estimatedState, estimatedCovariance)

	# This is just the prediction step of the Kalman Filter. Call this explicitly
	# if you want only a prediction (when you don't have a measurement available).
	def predict(self, previousState, previousCovariance):
		
		ts = self.ts
		
		# We are making the assumption that the sensor errors are uncorrelated
		measurementNoise = np.dot(measurementError.T , np.identity(len(measurementError)))

		# Prediction Step
		predictedState   = np.dot(self.stateTransitionMat , previousState.T)
		predictedCovar   = np.dot(np.dot(self.stateTransitionMat , previousCovariance), self.stateTransitionMat.T) \
							+ self.processNoise

		return (predictedState, predictedCovar)

# Simple 2D testing function.
def testTrajectory():
	# Simulated path that we are supposed to track.

	NUM_POINTS = 100
	T_STEP 		   = 0.1

	xdot = 1
	ydot = 1
	x = range(0, NUM_POINTS)
	y = range(0, NUM_POINTS)
	
	for i in range(1,NUM_POINTS):
		x[i] = x[i-1] + T_STEP*xdot
		y[i] = x[i-1]**2 + T_STEP*ydot
	
	plt.plot(x, y,'r-',label='Desired Trajectory')
	plt.xlabel('x')
	plt.ylabel('y')
	
	return [x,y]

# Simple 2D testing function.
def testAddNoise():
# This function introduces sensor noise
	GPS_ERROR  = 1
	NUM_POINTS = 100

	xNoise = np.random.normal(0,GPS_ERROR**2,NUM_POINTS)
	yNoise = np.random.normal(0,GPS_ERROR**2,NUM_POINTS)
	
	return [xNoise, yNoise]

# Main simple 2D testing function. 
def test2Dims():
	[x, y] = testTrajectory()
	[xNoise, yNoise] = testAddNoise()
	z_x    = x
	z_y    = y
	NUM_POINTS = 100
	for i in range(0,NUM_POINTS):
		z_x[i] = x[i] + xNoise[i]
		z_y[i] = y[i] + yNoise[i]

	plt.plot(z_x, z_y,'b-',label='GPS Measurements')
	plt.xlabel('x')
	plt.ylabel('y')
	plt.title('Red = Ideal, Blue = Noisy Path, Green = Kalman Filtered Path')
	plt.legend()

	numStates          = 15
	previousState      = np.zeros(numStates)
	previousCovariance = 0.02 * np.ones((numStates, numStates))
	measurement 	   = np.array(12 * [1.2]).T
	measurementError   = np.array(12 * [0.2]).T
	kalX			   = np.zeros(len(x))
	kalY			   = np.zeros(len(y))

	kf = KalmanFilter(previousState, previousCovariance)

	for i in range(len(x)):
		[currentState, currentCovariance] = kf.algorithm(previousState, previousCovariance,\
											measurement, measurementError)
		kalX[i] = currentState[0]
		kalY[i] = currentState[1]

		previousState 	   = currentState.T
		previousCovariance = currentCovariance
		measurement 	   = np.array([z_x[i], z_y[i], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]).T

	plt.plot(kalX, kalY, 'k')
	plt.show()

if __name__ == '__main__':
	test2Dims()