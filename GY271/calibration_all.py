"""

READ MAGNETOMETER MEASUREMENTS AND OUTPUT TO TEXT FILE FOR CALIBRATION

This code is designed to create a text file of points to calibrate the 
magnetometer with Magneto.

Code By: Michael Wrona
Created: 14 Jan 2021

Resources
---------

Magnetometer/IMU I used:
    https://www.adafruit.com/product/3463

"""


import csv
import time
import serial
import numpy as np
import matplotlib.pyplot as plt
import os
import scipy as sp

# GLOBAL VARIABLES
SER_PORT = 'COM3'  # Serial port
SER_BAUD = 115200  # Serial baud rate
SAMPLE_FREQ = 15  # Frequency to record magnetometer readings at [Hz]
T_SAMPLE = 250  # Total time to read mangetometer readings [sec]
F0 = 52.991 #magnitude of actual megnetic field in microTesla

OUTPUT_FILENAME = 'D:\intership\GY271\mag-cal-example\examplesmag-readings.txt'  # Output data file name
if os.path.exists(OUTPUT_FILENAME):
    os.remove(OUTPUT_FILENAME)


class SerialPort:
    """Create and read data from a serial port.

    Attributes:
        read(**kwargs): Read and decode data string from serial port.
    """
    def __init__(self, port, baud=SER_BAUD):
        """Create and read serial data.

        Args:
            port (str): Serial port name. Example: 'COM4'
            baud (int): Serial baud rate, default 9600.
        """
        if isinstance(port, str) == False:
            raise TypeError('port must be a string.')

        if isinstance(baud, int) == False:
            raise TypeError('Baud rate must be an integer.')

        self.port = port
        self.baud = baud

        # Initialize serial connection
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        self.ser.flushInput()
        self.ser.flushOutput()
    

    def Read(self, clean_end=True):
        """
        Read and decode data string from serial port.

        Args:
            clean_end (bool): Strip '\\r' and '\\n' characters from string. Common if used Serial.println() Arduino function. Default true
        
        Returns:
            (str): utf-8 decoded message.
        """
        # while self.ser.in_waiting:
        bytesToRead = self.ser.readline()
        
        decodedMsg = bytesToRead.decode('utf-8')

        if clean_end == True:
            decodedMsg = decodedMsg.strip('\r').strip('\n')  # Strip extra chars at the end
        
        return decodedMsg
    

    def Write(self, msg):
        """
        Write string to serial port.

        Args
        ----
            msg (str): Message to transmit
        
        Returns
        -------
            (bool) True if successful, false if not
        """
        try:
            self.ser.write(msg.encode())
            return True
        except:
            print("Error sending message.")
    

    def Close(self):
        """Close serial connection."""
        self.ser.close()



class PlotPoints3D:
    """Plot magnetometer readings as 3D scatter plot.
    
    Attributes:
        AddPoint(x, y, z): Add 3D point to plot.

    """

    def __init__(self, fig, ax, live_plot=True, marker='o', c='r'):
        """Plot points as 3D scatter plot.

        Args:
            fig (matplotlib.pyplot.figure): MPL figure object.
            ax (matplotlib.pyplot.figure.add_suplot): MPL 3D axes object
            live_plot (bool): Choose wheter or not to plot the data live
            marker (str): MPL marker symbol. See reference [1].
            c (str): MPL color letter. See reference [2].
        
        Resources:
            [1] Matplotlib plot marker symbols,
                https://matplotlib.org/3.2.1/api/markers_api.html#module-matplotlib.markers
            [2] Matplotlib plot/line color letters,
                https://matplotlib.org/2.0.2/api/colors_api.html
        """
        self.fig = fig  # fig and axes
        self.ax = ax
        self.live_plot = live_plot
        self.ptMarker = marker  # Point symbol
        self.ptColor = c  # Point color
        self.edgeColor = 'k'  # Border/edge of point
        self.ax.set_xlim((-80, 80))  # Set axes limits to keep plot shape
        self.ax.set_ylim((-80, 80))
        self.ax.set_zlim((-80, 80))
    

    def AddPoint(self, x, y, z):
        """Add 3D point to scatter plot.

        Args:
            x (float): X-coordinate.
            y (float): Y-coordinate.
            z (float): Z-coordinate.
        """
        self.ax.scatter(x, y, z, marker=self.ptMarker, 
            c=self.ptColor, edgecolors=self.edgeColor)
        
        if self.live_plot == True:
            plt.pause(0.001)


Arduino = SerialPort(SER_PORT, SER_BAUD)
N = int(SAMPLE_FREQ * T_SAMPLE)  # Number of readings
DT = 1.0 / SAMPLE_FREQ  # Sample period [sec]


# Create live plot for logging points
fig_rawReadings = plt.figure()
ax_rawReadings = fig_rawReadings.add_subplot(111, projection='3d')
measurementsPlot = PlotPoints3D(fig_rawReadings, ax_rawReadings, live_plot=False)


# Take a few readings to 'flush' out bad ones
for _ in range(4):
    data = Arduino.Read().split(',')  # Split into separate values
    time.sleep(0.25)


measurements = np.zeros((N, 3), dtype='float')

for currMeas in range(N):
    data = Arduino.Read().split(',')  # Split into separate values

    mx, my, mz = float(data[0]), float(data[1]), float(data[2])  # Convert to floats, [uT]
    
    print('[%0.4f, %0.4f, %0.4f] uT  |  Norm: %0.4f uT  |  %0.1f %% Complete.' % 
        (mx, my, mz, np.sqrt(mx**2 + my**2 + mz**2), (currMeas / N) * 100.0)
    )

    # Store data to array
    measurements[currMeas, 0] = mx
    measurements[currMeas, 1] = my
    measurements[currMeas, 2] = mz
    
    measurementsPlot.AddPoint(mx, my, mz)  # Add point to 3D plot



# After measurements are complete, write data to file
Arduino.Close()
print('Sensor Reading Complete!')
plt.show()
print('Writing data to {} ...'.format(OUTPUT_FILENAME))
for i in range(N):
    with open(OUTPUT_FILENAME, 'a', newline='') as f:
        writer = csv.writer(f, delimiter='\t')
        writer.writerow([measurements[i, 0], measurements[i, 1], measurements[i, 2]])

# apply calibration
#  this contain implementation of ellipsoid fitting algorithm
#  acknowledgement https://teslabs.com/articles/magnetometer-calibration/
# rawData = np.genfromtxt('D:\intership\GY271\mag-cal-example\examplesmag-readings.txt', delimiter='\t')  # Read raw measurements

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

M_es, n_es, d_es = elipsoid_fitting(measurements.T)
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
    currMeas = np.array([measurements[i, 0], measurements[i, 1], measurements[i, 2]])
    calibData[i, :] = A @ (currMeas - b)


# Plot XY data
plt.figure()
plt.plot(measurements[:, 0], measurements[:, 1], 'b*', label='Raw Meas.')
plt.plot(calibData[:, 0], calibData[:, 1], 'r*', label='Calibrated Meas.')
plt.title('XY Magnetometer Data')
plt.xlabel('X [uT]')
plt.ylabel('Y [uT]')
plt.legend()
plt.grid()
plt.axis('equal')

# Plot YZ data
plt.figure()
plt.plot(measurements[:, 1], measurements[:, 2], 'b*', label='Raw Meas.')
plt.plot(calibData[:, 1], calibData[:, 2], 'r*', label='Calibrated Meas.')
plt.title('YZ Magnetometer Data')
plt.xlabel('Y [uT]')
plt.ylabel('Z [uT]')
plt.legend()
plt.grid()
plt.axis('equal')

# Plot XZ data
plt.figure()
plt.plot(measurements[:, 0], measurements[:, 2], 'b*', label='Raw Meas.')
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
    xraw = measurements[i, 0]
    yraw = measurements[i, 1]
    zraw = measurements[i, 2]

    xcalib = calibData[i, 0]
    ycalib = calibData[i, 1]
    zcalib = calibData[i, 2]
    ax.scatter(xraw, yraw, zraw, color='r')
    ax.scatter(xcalib, ycalib, zcalib, color='b')

ax.set_title('3D Scatter Plot of Magnetometer Data')
ax.set_xlabel('X [uT]')
ax.set_ylabel('Y [uT]')
ax.set_zlabel('Z [uT]')

# plot after calibration
plt.show()
