import time
import serial
import numpy as np
import matplotlib.pyplot as plt

# GLOBAL VARIABLES
SER_PORT = 'COM3'  # Serial port
SER_BAUD = 115200  # Serial baud rate
SAMPLE_FREQ = 50  # Frequency to record magnetometer readings at [Hz]
T_SAMPLE = 100  # Total time to read mangetometer readings [sec]
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

measurements = np.zeros((N, 6), dtype='float')

for currMeas in range(N):
    data = Arduino.Read().split(',')  # Split into separate values

    ax, ay, az, gx, gy, gz = float(data[0]), float(data[1]), float(data[2]),float(data[3]), float(data[4]), float(data[5])  # Convert to floats, [uT]
    
    print('[%0.1f, %0.1f, %0.1f] 10*G  |[%0.1f, %0.1f, %0.1f] rad/s  | %0.1f %% Complete.' % 
        (ax, ay, az, gx, gy, gz, (currMeas / N) * 100.0)
    )

    # Store data to array
    measurements[currMeas, 0] = ax
    measurements[currMeas, 1] = ay
    measurements[currMeas, 2] = az
    measurements[currMeas, 3] = gx
    measurements[currMeas, 4] = gy
    measurements[currMeas, 5] = gz
# Calculate the mean of each cluster
cluster_bias = np.array([np.mean(measurements[i:i+100, 5]) for i in range(0, N, 100)])
print('deviation of bias = %0.4f (in degree)' % (np.std(cluster_bias)))

# probability plot
angle_roll = np.arctan2(measurements[:, 1], np.sqrt(measurements[:, 0]**2 + measurements[:, 2]**2 ))*180/np.pi
angle_pitch = np.arctan2(-measurements[:, 0], np.sqrt(measurements[:, 1]**2 + measurements[:, 2]**2 ))*180/np.pi
print('Roll Angle: mean = %0.4f, deviation = %0.4f(in degree)' % (angle_roll.mean(),np.std(angle_roll)))
print('Pitch Angle: mean = %0.4f, deviation = %0.4f(in degree)' % (angle_pitch.mean(),np.std(angle_pitch)))
print('roll rate: mean = %0.4f, deviation = %0.4f(in degree)' % (measurements[:, 3].mean(),np.std(measurements[:, 3])))
print('pitch rate: mean = %0.4f, deviation = %0.4f(in degree)' % (measurements[:, 4].mean(),np.std(measurements[:, 4])))
print('yaw rate: mean = %0.4f, deviation = %0.4f(in degree)' % (measurements[:, 5].mean(),np.std(measurements[:, 5])))
fig0, axs0 = plt.subplots(1, 2, figsize=(15, 5))
# for roll
axs0[0].hist(angle_roll, bins=100, density=True)
axs0[0].set_xlabel('Data Value')
axs0[0].set_ylabel('Probability')
axs0[0].set_title(f'Probability Distribution of angle roll' )
    
# for pitch
axs0[1].hist(angle_pitch, bins=100, density=True)
axs0[1].set_xlabel('Data Value')
axs0[1].set_ylabel('Probability')
axs0[1].set_title(f'Probability Distribution of angle pitch' )
# Adjust the spacing between subplots
plt.subplots_adjust(hspace=0.5)

fig, axs = plt.subplots(1, 3, figsize=(15, 5))
for fig_num in range (3,6):
    axs[fig_num-3].hist(measurements[:, fig_num], bins=100, density=True)
    axs[fig_num-3].set_xlabel('Data Value')
    axs[fig_num-3].set_ylabel('Probability')
    axs[fig_num-3].set_title(f'Probability Distribution {fig_num}' )
# Adjust the spacing between subplots
plt.subplots_adjust(hspace=0.5)

plt.show()
