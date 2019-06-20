"""
Author:         Kevin Ta
Date:           2019 May 7th
Purpose:        This library aims to do perform two primary objectives:

                1. Establish Bluetooth connection with Teensy wheel modules for data acquisition.
                2. Parse and store data.

                To do so, the code utilizes pybluez for bluetooth connection, cobs for byte en/decoding, and Google's
                protobuf protocol for serializing the structured data. The protobuf interpreter can be found as imuMsg.
"""


# IMPORTED LIBRARIES

import os
import datetime
import numpy as np
import pandas as pd
import bluetooth
from cobs import cobs
from libraries.imumsg import imumsg_pb2 as imuMsg
import math
import time
from multiprocessing import Queue


# DEFINITIONS

dir_path = os.path.dirname(os.path.realpath(__file__))  # Current file directory

# Python dictionaries storing name of data source, bluetooth address, data storage path, and the recorded data
Left = {'Name': 'Left', 'Address': '98:D3:51:FD:AD:F5', 'Placement': 'Left', 'Device': 'Wheel',
        'AccPath': os.path.join('IMU Data', '{} leftAcc.csv'.format(
            datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
        'GyroPath': os.path.join('IMU Data', '{} leftGyro.csv'.format(
            datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
        'Path': '',
        'DisplayData': np.zeros((7, 1000)),
        'Queue': Queue(),
        'RunMarker': Queue()
        }

Right = {'Name': 'Right', 'Address': '98:D3:81:FD:48:C9', 'Placement': 'Right', 'Device': 'Wheel',
         'AccPath': os.path.join('IMU Data', '{} rightAcc.csv'.format(
             datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
         'GyroPath': os.path.join('IMU Data', '{} rightGyro.csv'.format(
             datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
         'Path': '',
         'DisplayData': np.zeros((7, 1000)),
         'Queue': Queue(),
         'RunMarker': Queue()
         }

# Dictionary associating measurement descriptions to array space
IMUDataDict = {'X Acceleration (m/s^2)': 1, 'Y Acceleration (m/s^2)': 2, 'Z Acceleration (m/s^2)': 3,
               'X Angular Velocity (rad/s)': 4, 'Y Angular Velocity (rad/s)': 5, 'Z Angular Velocity (rad/s)': 6}


# CLASSES

class ClWheelDataParsing:
    """
    Class that instantiates ClBluetoothConnect to connect to BT, parses data from Teensy wheel module, and stores data
    in data lists.
    """

    def __init__(self, dataSource):
        """
        Purpose:    Initialize bluetooth connection and storage lists.
        Passed:     Source information containing bluetooth address data and display data shared variable.
        """

        self.runStatus = dataSource['RunMarker']  # Queue to check when terminate signal is sent from main program

        self.Queue = dataSource['Queue'] # Queue for data transfer to main program

        self.address = dataSource['Address'] # BT Address

        self.path = dataSource['Path'] # Save path name

        self.refTime = 0
        self.timeOffset = 0

        # Create class storage variables
        self.timeStamp = []
        self.timeReceived = []
        self.xData = []
        self.yData = []
        self.zData = []
        self.xGyro = []
        self.yGyro = []
        self.zGyro = []

    def fnRun(self):
        """
        Purpose:    Main program that continuously runs.
                    Decodes messages from BT signal and stores data.
        Passed:     None.
        """

        self.IMU = ClBluetoothConnect(self.address)  # Creates bluetooth connection instance with wheel module

        freqCount = 0  # Frequency counter

        status = 'Active.' # Set marker to active

        self.IMU.fnCOBSIntialClear() # Wait until message received starts at the correct location

        # Cycle through data retrieval to clear out buffered messages
        for i in range(1000):
            if status != 'Disconnected.':
                status = self.IMU.fnRetieveIMUMessage()
                self.fnReceiveData(self.IMU.cobsMessage, 'wait')

        # Initializes response for time synchronization
        if status != 'Disconnected.':
            status = self.IMU.fnRetieveIMUMessage()
            self.fnReceiveData(self.IMU.cobsMessage, 'init')

        # Cycle through data retrieval until bluetooth disconnects or terminate signal received
        while status != 'Disconnected.' and self.runStatus.empty():
            status = self.IMU.fnRetieveIMUMessage()
            self.fnReceiveData(self.IMU.cobsMessage)
            freqCount += 1
            if freqCount >= 500:
                freqCount = 0
                print('Wheel Frequency: {} Hz'.format(500/(self.timeReceived[-1] - self.timeReceived[-501])))

        self.fnSaveData()

        # Close socket connection
        self.IMU.sock.close()


    def fnReceiveData(self, msg, state = 'stream'):
        """
        Purpose:    Unpack data coming from Teensy wheel module and calls fnStoreData to store data.
        Passed:     Cobs deciphered byte string message.
                    State of data collection.
                        1. wait - record no data, allow for buffered messages to clear
                        2. init - set initial timestamp and time offset for time synchronization.
                        3. stream (default) - collect and store data.
        """

        # Try to decipher message based on preset protobuf specifications
        try:
            # # Get data size
            # dataSizeArray = msg[:4]
            # dataSize = struct.unpack("<L", dataSizeArray)[0]

            # Pass msg to imuMsg to parse into float values stored in imuMsg instance
            data = msg[4:]
            imuMsgBT = imuMsg.IMUInfo()
            imuMsgBT.ParseFromString(data)

            # Initialize time offset and reference time for time synchronization
            if state == 'init':
                self.refTime = time.time()
                self.timeOffset = imuMsgBT.time_stamp / 1000000
                self.timeReceived.append(time.time())
                # Append data to display data and class variables
                self.fnStoreData(imuMsgBT)

            # Record data into appropriate class lists and display data array
            elif state == 'stream':
                self.timeReceived.append(time.time())
                self.fnStoreData(imuMsgBT)

        # Returns exceptions as e to avoid code crash but still allow for debugging
        except Exception as e:
            print(e)

    def fnStoreData(self, wheelDataPB):
        """
        Purpose:    Store data into display data and class variables.
        Passed:     wheel data format with Teensy time values, (x, y, z) acceleration in Gs,
                    (x, y, z) angular velocity in deg/s.
        """

        # Sets adjusted timestamp
        timeStamp = self.refTime + wheelDataPB.time_stamp / 1000000 - self.timeOffset

        # Sends received data to queue
        self.Queue.put([timeStamp, wheelDataPB.acc_x * 9.8065, wheelDataPB.acc_y * 9.8065, wheelDataPB.acc_z * 9.8065,
                                     wheelDataPB.angular_x * math.pi / 180, wheelDataPB.angular_y * math.pi / 180, wheelDataPB.angular_z * math.pi / 180])

        # Appends class lists
        self.timeStamp.append(timeStamp)
        self.xData.append(wheelDataPB.acc_x * 9.8065)
        self.yData.append(wheelDataPB.acc_y * 9.8065)
        self.zData.append(wheelDataPB.acc_z * 9.8065)
        self.xGyro.append(wheelDataPB.angular_x * math.pi / 180)
        self.yGyro.append(wheelDataPB.angular_y * math.pi / 180)
        self.zGyro.append(wheelDataPB.angular_z * math.pi / 180)

    def fnSaveData(self):
        """
        Purpose:    Records data into csv file.
        Passed:     dataSource containing file save path.
        """

        # Create time string list based on the Androsensor format
        timeString = [datetime.datetime.fromtimestamp(utcTime).strftime('%Y-%m-%d %H:%M:%S:%f')[:-3] for utcTime in self.timeStamp]

        # Save using pandas
        IMUData = pd.DataFrame({'ACCELEROMETER X (m/s²)': np.array(self.xData),
                     'ACCELEROMETER Y (m/s²)': np.array(self.yData),
                     'ACCELEROMETER Z (m/s²)': np.array(self.zData),
                     'GYROSCOPE X (rad/s)': np.array(self.xGyro),
                     'GYROSCOPE Y (rad/s)': np.array(self.yGyro),
                     'GYROSCOPE Z (rad/s)': np.array(self.zGyro),
                     'Time since start in ms ': np.array(self.timeStamp) - self.timeStamp[0],
                     'YYYY-MO-DD HH-MI-SS_SSS': timeString})

        IMUData.to_csv(self.path, index = False)


class ClBluetoothConnect:
    """
    Class for establishing communication and decoding messages using COBS.
    """

    def __init__(self, BTAddress):
        """
        Purpose:    Initialize bluetooth connection using PyBluez module.
        Passed:     Bluetooth address.
        """
        self.cobsMessage = '' # Create variable for storing COBS decoded message
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM) # Configure bluetooth connection to RFCOMM

        print ("{}: Began connection".format(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))

        self.sock.connect((BTAddress, 1))

        print ("{}: Established connection".format(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))


    def fnRetieveIMUMessage(self):
        """
        Purpose:    Decode received COBS byte string to
        Passed:     None.
        Return:     Status of message.
        """
        data = []  # List containing characters of byte string
        c = self.sock.recv(1) # Receive 1 byte of information

        # Continue acquiring bytes of data until end point is reached. Combine into byte string.
        while c != b'\x00':
            if c == b'':
                self.fnShutDown()
                return "Disconnected."
            data.append(c)
            c = self.sock.recv(1)
        data = b''.join(data)

        # Try to decode message and returnes exception to avoid closing the program
        try:
            self.cobsMessage = self.fnDecodeCOBS(data)
            return "Received."
        except Exception as e:
            print("Failed to decode message due to {}".format(e))

    def fnDecodeCOBS(self, encodedCobsMsg):
        """
        Purpose:    Wrapper for cobs module to decode message.
        Passed:     Encoded COBS message.
        Returns:    Decoded COBS message.
        """
        return cobs.decode(encodedCobsMsg)

    def fnShutDown(self):
        """
        Purpose:    Close socket connections on shutdown.
        """

        print("Disconnected from server.")
        self.sock.close()

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
            chunk = self.sock.recv(1)
            print(chunk[0])
            print(ord(chunk))

            if chunk == '':
                print("socket connection broken shutting down this thread")
                self.fnShutDown()
                return 0

            chunks.append(chunk)
            bytes_recd = bytes_recd + len(chunk)
        return b''.join(chunks)


if __name__ == "__main__":

    if not os.path.exists(os.path.join(dir_path, 'IMU Data')):
        os.mkdir(os.path.join(dir_path, 'IMU Data'))

    instWheelDataParsing = ClWheelDataParsing(Left)
    instWheelDataParsing.fnRun()