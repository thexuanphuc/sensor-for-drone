import time
import serial
import numpy as np
import matplotlib.pyplot as plt

# GLOBAL VARIABLES
SER_PORT = 'COM3'  # Serial port
SER_BAUD = 115200  # Serial baud rate
SAMPLE_FREQ = 75  # Frequency to record magnetometer readings at [Hz]
T_SAMPLE = 50  # Total time to read mangetometer readings [sec]
F0 = 52.991 #magnitude of actual megnetic field in microTesla


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

Arduino = SerialPort(SER_PORT, SER_BAUD)
N = int(SAMPLE_FREQ * T_SAMPLE)  # Number of readings
DT = 1.0 / SAMPLE_FREQ  # Sample period [sec]

# # Create live plot for logging points
# fig_rawReadings = plt.figure()
# ax_rawReadings = fig_rawReadings.add_subplot(111, projection='3d')
# measurementsPlot = PlotPoints3D(fig_rawReadings, ax_rawReadings, live_plot=False)


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
    
# calibration
A = np.array([  [1.09745885,  0.00531246,  0.03279459],
                [0.00531246,  1.09457112, -0.02298825],
                [0.03279459, -0.02298825,  1.17092349]])

b = np.array([-9.14714356, 13.06060926,  2.62056838])
# b = np.zeros([3, 1])
# A = np.eye(3)

#after find the value of matrix A, b
calibData = np.zeros((N, 3), dtype='float')
for i in range(N):
    currMeas = np.array([measurements[i, 0], measurements[i, 1], measurements[i, 2]])
    calibData[i, :] = A @ (currMeas - b)


# probability plot
angle = np.arctan2(calibData[:,1], calibData[:,0])*180/np.pi
print(angle.mean())
print(np.std(angle))
# Clear the plot
# plt.clf()
plt.figure()
plt.hist(angle, bins=100, density=True)
plt.xlabel('Data Value')
plt.ylabel('density')
plt.title('Probability Distribution of heading angle')
plt.show()
