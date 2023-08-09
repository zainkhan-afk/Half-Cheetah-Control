import numpy as np

def GetRotationMatrix(theta):
	R = np.array([
				[np.cos(theta), -np.sin(theta)],
				[np.sin(theta),  np.cos(theta)]
				])

	return R


def GetTransformationMatrix(theta, x, y):
	R = GetRotationMatrix(theta)
	T = np.append(R, np.array([[x, y]]).T, axis = 1)
	T = np.append(T, np.array([[0, 0, 1]]), axis = 0)

	return T


def GetInverseMatrix(M):
	return np.linalg.inv(M)