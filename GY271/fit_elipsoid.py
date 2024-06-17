
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt

# Read raw data and apply calibration
rawData = np.genfromtxt('D:\intership\GY271\mag-cal-example\examplesmag-readings.txt', delimiter='\t')  # Read raw measurements
F0 = 52.9 #magnitude of actual megnetic field in microTesla
N = len(rawData)

#using least squart to find the value of A,b
def elipsoid_fitting(s): # this function to find the value M,n,d in equation of estimated elipsoid
    # this vector D equal to data vector X_i in article
    D = np.array([s[0]**2., s[1]**2., s[2]**2.,
                2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
                2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])
    S = np.dot(D, D.T)
    # define submatrix S11, S12, S13, S14 (equation 11 in article)
    S11 = S[:6, :6]
    S12 = S[:6, 6:]
    S21 = S[6:, :6]
    S22 = S[6:, 6:]
    # C (Eq. 8, k=4)
    k = 4
    #define matrix C1, equation 7 in article
    C1 = np.array([ [-1,  1,  1,  0,  0,  0],
                    [ 1, -1,  1,  0,  0,  0],
                    [ 1,  1, -1,  0,  0,  0],
                    [ 0,  0,  0, -4,  0,  0],
                    [ 0,  0,  0,  0, -4,  0],
                    [ 0,  0,  0,  0,  0, -4]])
    
    # define a matrix E to find eigenvalue lamda, eigenvector in equation 14+15: E = C^-1 *(S11 - S12*S22^-1*S21)
    E = np.dot(np.linalg.inv(C1), S11 - np.dot(S12, np.dot(np.linalg.inv(S22),S21)))
    # find eigenvector, eigenvalue of matrix E
    E_value, E_vector = np.linalg.eig(E)
    # solution v_1 shoulde be the eigenvector, correspond with maximum eigenvalue (the only positive one)
    # some where have condition that if v_1[0] < 0, then replace v_1 to -v_1
    v_1 = E_vector[:, np.argmax(E_value)]
    if v_1[0] < 0: v_1 = -v_1
    # find v2 form equation 13
    v_2 = np.dot(np.dot(-np.linalg.inv(S22), S21), v_1) # becareful with demenssion in this
    # quadric-form parameters 
    M = np.array([[v_1[0], v_1[5], v_1[4]],
                    [v_1[5], v_1[1], v_1[3]],
                    [v_1[4], v_1[3], v_1[2]]])
    n = np.array([[v_2[0]],
                    [v_2[1]],
                    [v_2[2]]])
    d = v_2[3]
    return M, n, d

M_es, n_es, d_es = elipsoid_fitting(rawData.T)
#  find the value matrix A, b, not check yet!!!!
# initial condition of b_es, A_es_inv
b_es = np.zeros([3, 1])
A_es_inv = np.eye(3)

b_es = np.dot(-np.linalg.inv(M_es), n_es)
A_es_inv = np.real((F0/np.sqrt(np.dot(n_es.T, np.dot(np.linalg.inv(M_es), n_es)) - d_es))*sp.linalg.sqrtm(M_es))

# assign the value to plot
b = np.zeros([3, 1])
A = np.eye(3)
A = A_es_inv
b = b_es.reshape((3,)) #transpose to row
print(A)
print(b)
#after find the value of matrix A, b
calibData = np.zeros((N, 3), dtype='float')
for i in range(N):
    currMeas = np.array([rawData[i, 0], rawData[i, 1], rawData[i, 2]])
    calibData[i, :] = A @ (currMeas - b)


# Plot XY data
plt.figure()
plt.plot(rawData[:, 0], rawData[:, 1], 'b*', label='Raw Meas.')
plt.plot(calibData[:, 0], calibData[:, 1], 'r*', label='Calibrated Meas.')
plt.title('XY Magnetometer Data')
plt.xlabel('X [uT]')
plt.ylabel('Y [uT]')
plt.legend()
plt.grid()
plt.axis('equal')

# Plot YZ data
plt.figure()
plt.plot(rawData[:, 1], rawData[:, 2], 'b*', label='Raw Meas.')
plt.plot(calibData[:, 1], calibData[:, 2], 'r*', label='Calibrated Meas.')
plt.title('YZ Magnetometer Data')
plt.xlabel('Y [uT]')
plt.ylabel('Z [uT]')
plt.legend()
plt.grid()
plt.axis('equal')

# Plot XZ data
plt.figure()
plt.plot(rawData[:, 0], rawData[:, 2], 'b*', label='Raw Meas.')
plt.plot(calibData[:, 0], calibData[:, 2], 'r*', label='Calibrated Meas.')
plt.title('XZ Magnetometer Data')
plt.xlabel('X [uT]')
plt.ylabel('Z [uT]')
plt.legend()
plt.grid()
plt.axis('equal')


# Plot 3D scatter
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for i in range(N):
    xraw = rawData[i, 0]
    yraw = rawData[i, 1]
    zraw = rawData[i, 2]

    xcalib = calibData[i, 0]
    ycalib = calibData[i, 1]
    zcalib = calibData[i, 2]
    ax.scatter(xraw, yraw, zraw, color='r')
    ax.scatter(xcalib, ycalib, zcalib, color='b')

ax.set_title('3D Scatter Plot of Magnetometer Data')
ax.set_xlabel('X [uT]')
ax.set_ylabel('Y [uT]')
ax.set_zlabel('Z [uT]')


plt.show()
