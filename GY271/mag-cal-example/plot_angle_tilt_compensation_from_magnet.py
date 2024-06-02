import time
import serial
import numpy as np
import matplotlib.pyplot as plt

# GLOBAL VARIABLES
SER_PORT = 'COM3'  # Serial port
SER_BAUD = 115200  # Serial baud rate
SAMPLE_FREQ = 50  # Frequency to record magnetometer readings at [Hz]
T_SAMPLE = 60  # Total time to read mangetometer readings [sec]
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

measurements = np.zeros((N), dtype='float')

for currMeas in range(N):
    data = Arduino.Read()  # Split into separate values

    measurements[currMeas] = float(data)
    
    print('%0.1f %% Complete.' % 
        ( (currMeas / N) * 100.0)
    )

    # Calculate the mean of each cluster
cluster_bias = np.array([np.mean(measurements[i:i+100]) for i in range(0, N, 100)])
print('deviation of bias = %0.4f (in degree)' % (np.std(cluster_bias)))
print('mean = %0.4f (in degree), std-deviation = %0.4f (in degree)' % ( measurements.mean(),np.std(measurements)))

