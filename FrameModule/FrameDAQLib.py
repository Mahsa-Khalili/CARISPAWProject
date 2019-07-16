"""
Author:         Kevin Ta
Date:           2019 May 17th
Purpose:        This code aims to do perform two primary objectives:

                1. Establish TCP connection with the Raspberry for data acquisition.
                2. Receive IMU data from Raspberry Pi for storage and real-time display.

                To do so, the code utilizes Python sockets for TCP connection, cobs for byte en/decoding, and Google's
                protobuf protocol for serializing the structured data. The protobuf interpreter can be found as
                frameUnitMsg.

                The data is displayed using the PyQTgraph library, updating at 100 ms intervals. When the display window
                is closed, the code will than dump the data into a file found in the IMU Data subdirectory.
"""


# IMPORTED LIBRARIES

import os
import time
import datetime
import numpy as np
import pandas as pd
import pickle as pkl
import math
import socket
import bluetooth
from multiprocessing import Queue
# import frameUnitMsg_pb2 as frameUnitMsg
import carisPAWBuffers_pb2 as carisPAWBuffers
from cobs import cobs
from scipy.signal import butter, lfilter


# DEFINITIONS

dir_path = os.path.dirname(os.path.realpath(__file__))  # Current file directory

# Library containing all the relecant information for the Pi
RaspberryPi = {'Name': 'Frame', 'Address': 'B8:27:EB:A3:ED:6F', 'Host': '', 'Port': 65432, 'Placement': 'Middle', 'Device': 'Frame',
               'AccPath6050': os.path.join('IMU Data', '{} Frame6050Acc.csv'.format(
                   datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
               'GyroPath6050': os.path.join('IMU Data', '{} Frame6050Gyro.csv'.format(
                   datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
               'AccPath9250': os.path.join('IMU Data', '{} Frame9250Acc.csv'.format(
                   datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
               'GyroPath9250': os.path.join('IMU Data', '{} Frame9250Gyro.csv'.format(
                   datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
                'Path': '',
               'DisplayData': np.zeros((13, 1000)), # 9250 then 6050
               'Queue': Queue(),
               'RunMarker': Queue()
               }

# CLASSES

class ClFrameDataParsing:
    """
    Class that instantiates ClTCPServer to connect to Pi, receiving MPU6050/9250 data.
    """

    def __init__(self, dataSource, commPort = 65432, protocol = 'UDP'):
        """
        Purpose:    Connect to frame module using UDP/TCP/BT and sends data to main program
        Passed:     dataSource dicitionary with all relevant information
                    Expected WiFi port to connect to
                    Expected protocol for wireless connection
        """

        self.runStatus = dataSource['RunMarker']  # Queue to check when terminate signal is sent from main program

        self.Queue = dataSource['Queue'] # Queue for data transfer to main program

        self.address = dataSource['Address'] # BT Address

        self.path = dataSource['Path'] # Save path name
        self.proximityPath = dataSource['ProximityPath']  # Save path name

        self.protocol = protocol

        # Initialize class variables
        self.refTime = 0

        self.FrameUnit = ClWirelessServer(self.address, dataSource['Host'], dataSource['Port'], self.protocol) # Create Server

        self.activeSensors = pkl.loads(self.FrameUnit.activeSensors)

        # Create class storage variables
        self.timeReceived = []
        self.timeStamp6050 = []
        self.timeStamp9250 = []
        self.timeStampUSS = []

        self.xData6050 = []
        self.yData6050 = []
        self.zData6050 = []
        self.xGyro6050 = []
        self.yGyro6050 = []
        self.zGyro6050 = []

        self.xData9250 = []
        self.yData9250 = []
        self.zData9250 = []
        self.xGyro9250 = []
        self.yGyro9250 = []
        self.zGyro9250 = []
        self.xMag9250 = []
        self.yMag9250 = []
        self.zMag9250 = []

        self.proximityDown = []

        self.compHeading = {'6050': [], '9250':[]}
        self.compPitch = {'6050': [], '9250':[]}
        self.compRoll = {'6050': [], '9250':[]}

        self.valPitch = {'6050': 0, '9250':0}
        self.valRoll = {'6050': 0, '9250':0}
        self.valHeading = {'6050': 0, '9250':0}

    def fnRun(self):
        """
        Purpose:    Main program that continuously runs.
                    Decodes messages from TCP and stores data.
        Passed:     None.
        """

        status = 'Active.' # Set marker to active
        sensorCount = [-50, -50, -15, 0, 0]

        receivedCalPy = []
        receivedCalPi = []

        if self.protocol == ('TCP' or 'BT'):
            self.FrameUnit.fnCOBSIntialClear() # Wait until message received starts at the correct location

        # Cycle through data retrieval to clear out buffered messages
        for i in range(1000):
            if status != 'Disconnected.':
                status = self.FrameUnit.fnRetievePiMessage()
                self.fnReceiveData(self.FrameUnit.cobsMessage, state = 'startup')

        for i in range(1000):
            if status != 'Disconnected.':
                status = self.FrameUnit.fnRetievePiMessage()
                receivedCalPi.append(self.fnReceiveData(self.FrameUnit.cobsMessage, state = 'wait'))
                receivedCalPy.append(time.time())

        self.refTime = np.mean(np.subtract(receivedCalPy, receivedCalPi))

        # Initializes response for time synchronization
        if status != 'Disconnected.':
            status = self.FrameUnit.fnRetievePiMessage()
            self.fnReceiveData(self.FrameUnit.cobsMessage, state='init')
            status = self.FrameUnit.fnRetievePiMessage()
            self.fnReceiveData(self.FrameUnit.cobsMessage)

        # Cycle through data retrieval until client disconnects or terminate signal received
        while status != 'Disconnected.' and self.runStatus.empty():
            status = self.FrameUnit.fnRetievePiMessage()
            frameBuffer = self.fnReceiveData(self.FrameUnit.cobsMessage)
            sensorCount[frameBuffer.sensorType] += 1

            if sensorCount[0] >= 500:
                print('IMU-9 Frequency: {} Hz'.format(500/(self.timeStamp9250[-1] - self.timeStamp9250[-501])))
                sensorCount[0] = 0
            elif sensorCount[1] >= 500:
                print('IMU-6 Frequency: {} Hz'.format(500/(self.timeStamp6050[-1] - self.timeStamp6050[-501])))
                sensorCount[1] = 0
            elif sensorCount[2] >= 100:
                print('USS-DOWN Frequency: {} Hz'.format(100/(self.timeStampUSS[-1] - self.timeStampUSS[-101])))
                sensorCount[2] = 0
        self.fnSaveData()

        # Close socket connection
        self.FrameUnit.fnShutDown()

    def fnReceiveData(self, msg, state = 'stream'):
        """
        Purpose:    Unpack data coming from Pi frame module and calls fnStoreData to store data.
        Passed:     Cobs deciphered byte string message.
        """

        # Try to decipher message based on preset protobuf specifications
        try:
            # Pass msg to frameUnitMsg to parse into float values stored in imuMsg instance
            data = msg
            frameUnitMsgRcv = carisPAWBuffers.frameUnit()
            frameUnitMsgRcv.ParseFromString(data)

            # Initialize time offset and reference time for time synchronization
            if state == 'stream':
                self.timeReceived.append(time.time())
                # Append data to display data and class variables
                self.fnStoreData(frameUnitMsgRcv)
                return frameUnitMsgRcv

            # Record data into appropriate class lists and display data array
            elif state == 'wait':
                # Append data to display data and class variables
                return frameUnitMsgRcv.time_stamp

        # Returns exceptions as e to avoid code crash but still allow for debugging
        except Exception as e:
            print (e)

    def fnStoreData(self, frameUnitPB):
        """
        Purpose:    Store data into display data and class variables.
        Passed:     Frame data format containing Pi timestamps, acceleration in m/s^2,
                    (x, y, z) angular velocity in deg/s.
        """

        # Appends class lists

        if frameUnitPB.sensorType == carisPAWBuffers.frameUnit.IMU_6:

            self.timeStamp6050.append(self.refTime + frameUnitPB.time_stamp)
            self.xData6050.append(frameUnitPB.acc_x)
            self.yData6050.append(frameUnitPB.acc_y)
            self.zData6050.append(frameUnitPB.acc_z)
            self.xGyro6050.append(frameUnitPB.angular_x * math.pi / 180)
            self.yGyro6050.append(frameUnitPB.angular_y * math.pi / 180)
            self.zGyro6050.append(frameUnitPB.angular_z * math.pi / 180)

            if len(self.timeStamp6050) > 1:
                self.fnCalculateAngles(self.timeStamp6050[-1] - self.timeStamp6050[-2],  frameUnitPB, source = '6050')
            else:
                self.fnCalculateAngles(0, frameUnitPB, source='6050')

            self.Queue.put([carisPAWBuffers.frameUnit.IMU_6, self.timeStamp6050[-1], frameUnitPB.acc_x, frameUnitPB.acc_y, frameUnitPB.acc_z,
                            frameUnitPB.angular_x * math.pi / 180, frameUnitPB.angular_y * math.pi / 180,
                            frameUnitPB.angular_z * math.pi / 180, self.valPitch['6050'], self.valRoll['6050'], self.valHeading['6050']])

        elif frameUnitPB.sensorType == carisPAWBuffers.frameUnit.IMU_9:
            self.timeStamp9250.append(self.refTime + frameUnitPB.time_stamp)
            self.xData9250.append(frameUnitPB.acc_x)
            self.yData9250.append(frameUnitPB.acc_y)
            self.zData9250.append(frameUnitPB.acc_z)
            self.xMag9250.append(frameUnitPB.mag_x)
            self.yMag9250.append(frameUnitPB.mag_y)
            self.zMag9250.append(frameUnitPB.mag_z)
            self.xGyro9250.append(frameUnitPB.angular_x * math.pi / 180)
            self.yGyro9250.append(frameUnitPB.angular_y * math.pi / 180)
            self.zGyro9250.append(frameUnitPB.angular_z * math.pi / 180)

            if len(self.timeStamp9250) > 1:
                self.fnCalculateAngles(self.timeStamp9250[-1] - self.timeStamp9250[-2], frameUnitPB, source='9250')
            else:
                self.fnCalculateAngles(0, frameUnitPB, source='9250')

            self.Queue.put([carisPAWBuffers.frameUnit.IMU_9, self.timeStamp9250[-1], frameUnitPB.acc_x, frameUnitPB.acc_y, frameUnitPB.acc_z,
                            frameUnitPB.angular_x * math.pi / 180, frameUnitPB.angular_y * math.pi / 180, frameUnitPB.angular_z * math.pi / 180,
                            self.valPitch['9250'], self.valRoll['9250'], self.valHeading['9250'],
                            frameUnitPB.mag_x, frameUnitPB.mag_y, frameUnitPB.mag_z
                            ])

        elif frameUnitPB.sensorType == carisPAWBuffers.frameUnit.USS_DOWN:
            self.timeStampUSS.append(self.refTime + frameUnitPB.time_stamp)
            self.proximityDown.append(frameUnitPB.USensorDownward)

            self.Queue.put([carisPAWBuffers.frameUnit.USS_DOWN, self.timeStampUSS[-1], frameUnitPB.USensorDownward])

    def fnCalculateAngles(self, timeDiff, frameUnitPB, source = '9250'):
        """
        Purpose:    Utilize complimentary filter to get angle of device
        passed:     None.
        """

        alpha = 0.98

        # Integrate the gyroscope data -> int(angularSpeed) = angle
        self.valRoll[source] += -frameUnitPB.angular_x * timeDiff # Angle around the X-axis
        self.valPitch[source] -= frameUnitPB.angular_y * timeDiff   # Angle around the Y-axis

        # Compensate for drift with accelerometer data if !bullshit
        # Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
        forceMagnitudeApprox = abs(frameUnitPB.acc_x) + abs(frameUnitPB.acc_y) + abs(frameUnitPB.acc_z)

        # if (forceMagnitudeApprox > 8 and forceMagnitudeApprox < 11):
        if (forceMagnitudeApprox > 4.905 and forceMagnitudeApprox < 19.62):
            # Turning around the X axis results in a vector on the Y-axis
            rollAcc = math.atan2(frameUnitPB.acc_y, -frameUnitPB.acc_z)# - math.pi
            self.valRoll[source] = self.valRoll[source] * alpha + rollAcc * (1 - alpha) * 180 / math.pi

            # Turning around the Y axis results in a vector on the X-axis
            pitchAcc = math.atan2(-frameUnitPB.acc_x, -frameUnitPB.acc_z)# + math.pi
            self.valPitch[source] = self.valPitch[source] * alpha + pitchAcc * (1 - alpha) * 180 / math.pi

        self.compPitch[source].append(self.valPitch[source])
        self.compRoll[source].append(self.valRoll[source])
        self.compHeading[source].append(0)

    def fnSaveData(self):
        """
        Purpose:    Store data into specified path.
        Passed:     None.
        """

        if self.timeStamp9250:
            timeString = [datetime.datetime.fromtimestamp(utcTime).strftime('%Y-%m-%d %H:%M:%S:%f')[:-3] for utcTime in
                          self.timeStamp9250]

            IMUData = pd.DataFrame({'ACCELEROMETER X (m/s²)': np.array(self.xData9250),
                                    'ACCELEROMETER Y (m/s²)': np.array(self.yData9250),
                                    'ACCELEROMETER Z (m/s²)': np.array(self.zData9250),
                                    'GYROSCOPE X (rad/s)': np.array(self.xGyro9250),
                                    'GYROSCOPE Y (rad/s)': np.array(self.yGyro9250),
                                    'GYROSCOPE Z (rad/s)': np.array(self.zGyro9250),
                                    'Time since start in ms ': np.array(self.timeStamp9250) - self.timeStamp9250[0],
                                    'YYYY-MO-DD HH-MI-SS_SSS': timeString,
                                    'Pitch (Deg)': self.compPitch['9250'],
                                    'Roll (Deg)': self.compRoll['9250'],
                                    'Heading (Deg)': self.compHeading['9250'],
                                    'MagX': self.xMag9250,
                                    'MagY': self.yMag9250,
                                    'MagZ': self.zMag9250}
                                   )
            IMUData.to_csv(self.path + '.csv', index=False)

        if self.timeStamp6050:
            timeString = [datetime.datetime.fromtimestamp(utcTime).strftime('%Y-%m-%d %H:%M:%S:%f')[:-3] for utcTime in
                          self.timeStamp6050]
            IMUData6050 = pd.DataFrame({'ACCELEROMETER X (m/s²)': np.array(self.xData6050),
                                        'ACCELEROMETER Y (m/s²)': np.array(self.yData6050),
                                        'ACCELEROMETER Z (m/s²)': np.array(self.zData6050),
                                        'GYROSCOPE X (rad/s)': np.array(self.xGyro6050),
                                        'GYROSCOPE Y (rad/s)': np.array(self.yGyro6050),
                                        'GYROSCOPE Z (rad/s)': np.array(self.zGyro6050),
                                        'Pitch (Deg)': self.compPitch['6050'],
                                        'Roll (Deg)': self.compRoll['6050'],
                                        'Heading (Deg)': self.compHeading['6050'],
                                        'Time since start in ms ': np.array(self.timeStamp6050) - self.timeStamp6050[0],
                                        'YYYY-MO-DD HH-MI-SS_SSS': timeString
                                        }
                                       )
            IMUData6050.to_csv(self.path + '6050.csv', index=False)

        if self.timeStampUSS:
            timeString = [datetime.datetime.fromtimestamp(utcTime).strftime('%Y-%m-%d %H:%M:%S:%f')[:-3] for utcTime in
                          self.timeStampUSS]
            proximityData = pd.DataFrame({'Proximity (cm)': np.array(self.proximityDown),
                                        'Time since start in ms ': np.array(self.timeStampUSS) - self.timeStampUSS[0],
                                        'YYYY-MO-DD HH-MI-SS_SSS': timeString
                                        }
                                       )
            proximityData.to_csv(self.proximityPath + '.csv', index=False)

class ClWirelessServer:
    """
    Class
    """

    def __init__(self, address, host, port, protocol):
        """
        Purpose:
        Passed:
        """
        self.protocol = protocol
        self.cobsMessage = ''  # Create variable for storing COBS decoded message

        if self.protocol == 'TCP':
            self.TCPSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            print ("{}: Began connection".format(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))

            self.TCPSocket.bind((host, port))
            self.TCPSocket.listen(1)
            self.socket, self.addr = self.TCPSocket.accept()

            print ("{}: Established connection".format(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))

        elif self.protocol == 'UDP':

            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print('Initialized.')
            self.socket.bind((host, port))

        elif self.protocol =='BT':
            self.socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            print('Initialized.')
            self.socket.connect((address, port))

        self.activeSensors = self.socket.recv(24)

    def fnCOBSIntialClear(self):
        """
        Purpose:    Clear out initial code until at the start of a message.
        Passed:     None.
        """
        byte = self.fnReceive(1)

        # Keep looping while byte received is not 0, i.e. the end/start of a cobs message.
        while ord(byte) != 0:

            # Keep looping while not 0
            byte = self.fnReceive(1)
            print("Not 0")

            # Clear out potential initial garbage
            pass

    def fnShutDown(self):
        """
        Purpose:    Close socket connections on shutdown.
        """

        print("Disconnecting server.")
        self.socket.close()

    def fnReceive(self, MSGLEN):
        """
        Purpose:    Retrieve data for fnCOBSInitialClear.
        Passed:     Length of byte to receive.
        Return:     Joined byte string.
        """
        chunks = []
        bytes_recd = 0

        while bytes_recd < MSGLEN:

            print("Waiting for msg")
            chunk = self.socket.recv(1)
            print(chunk[0])
            print(ord(chunk))

            if chunk == '':
                print("socket connection broken shutting down this thread")
                self.socket.close()
                print("Disconnected.")
                return 0

            chunks.append(chunk)
            bytes_recd = bytes_recd + len(chunk)
        return b''.join(chunks)

    def fnRetievePiMessage(self):
        """
        Purpose:    Decode received COBS byte string to
        Passed:     None.
        Return:     Status of message.
        """

        if self.protocol == ('TCP' or 'BT'):
            data = []  # List containing characters of byte string
            c = self.socket.recv(1)  # Receive 1 byte of information

            # Continue acquiring bytes of data until end point is reached. Combine into byte string.
            while c != b'\x00':
                if c == b'':
                    self.socket.close()
                    return "Disconnected."
                data.append(c)
                c = self.socket.recv(1)
            data = b''.join(data)

        elif self.protocol == 'UDP':
            data = self.socket.recv(128)

        # Try to decode message and returnes exception to avoid closing the program
        try:
            self.cobsMessage = cobs.decode(data)
            return 'Received.'
        except Exception as e:
            print("Failed to decode message due to {}".format(e))


if __name__ == "__main__":

    if not os.path.exists(os.path.join(dir_path, 'IMU Data')):
        os.mkdir(os.path.join(dir_path, 'IMU Data'))

    instFrameDataParsing = ClFrameDataParsing(RaspberryPi)
    instFrameDataParsing.fnRun()