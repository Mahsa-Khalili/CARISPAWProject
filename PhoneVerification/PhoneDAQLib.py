"""
Author:         Kevin Ta
Date:           2019 May 24th
Purpose:        This custom library receives data from HyperIMU through TCP (JSON) or UDP (Byte string).
"""


# IMPORTED LIBRARIES

import socket
import os
import json
import datetime
import time
import numpy as np
import pandas as pd

# DEFINITIONS

HOST = ''              # Symbolic name meaning all available interfaces
PORT = 5555            # Arbitrary non-privileged port

dir_path = os.path.dirname(os.path.realpath(__file__))  # Current file directory

# Phone information dictionary
LeftPhone = {'Name': 'Left Phone', 'Port': 5555, 'Placement': 'Left', 'Device': 'Phone',
            'AccPath': os.path.join('IMU Data', '{} LeftphoneAcc.csv'.format(
                datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
            'GyroPath': os.path.join('IMU Data', '{} LeftphoneGyro.csv'.format(
                datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
            'Path': '',
            'DisplayData': np.zeros((7, 1000))}

RightPhone = {'Name': 'Right Phone', 'Port': 6666, 'Placement': 'Right', 'Device': 'Phone',
              'AccPath': os.path.join('IMU Data', '{} RightPhoneAcc.csv'.format(
                  datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
              'GyroPath': os.path.join('IMU Data', '{} RightPhoneGyro.csv'.format(
                  datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
              'Path': '',
              'DisplayData': np.zeros((7, 1000))}

FramePhone = {'Name': 'Frame Phone', 'Port': 7777,'Placement': 'Middle', 'Device': 'Phone',
              'AccPath': os.path.join('IMU Data', '{} RightPhoneAcc.csv'.format(
                  datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
              'GyroPath': os.path.join('IMU Data', '{} RightPhoneGyro.csv'.format(
                  datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
               'Path': '',
              'DisplayData': np.zeros((7, 1000))}


# CLASSES

class ClPhoneDataParsing:
    """
    Class for connecting to Phone IMU
    """

    def __init__(self, source, commPort = PORT, protocol = 'UDP'):
        """
        Purpose:    Initialize socket connections and class variables / lists.
        Passed:     Source dictionary for data storage.
                    Reserved port to listen to.
                    Data tranfer protocol. (UDP/TCP)
        """

        # Set self variables for displayData and protocol
        self.displayData = source['DisplayData']
        self.protocol = protocol

        # sets run status boolean to determine when to end data collection
        self.runStatus = True

        # Connect using TCP if specified
        if protocol is 'TCP':
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.bind((HOST, commPort))
            self.sock.listen(1)
            self.conn, self.addr = self.sock.accept()
            self.conn.setblocking(1)
            print('Connected by {}'.format(self.addr))

        # Connect using UDP, default
        if protocol is 'UDP':
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print('Initialized.')
            self.sock.bind((HOST, commPort))

        # Initialize class variables
        self.refTime = 0
        self.timeOffset = 0

        # initialize class list
        self.timeStamp = []
        self.timeReceived = []
        self.xData = []
        self.yData = []
        self.zData = []
        self.xGyro = []
        self.yGyro = []
        self.zGyro = []

    def fnClearInitial(self):
        """
        Purpose:    Cycle through received bytes until ending curly brace is detected from HyperIMU. (JSON TCP)
                    Prevents premature reading of byte stream.
        Passed:     None.
        """

        startMarker = b''

        while startMarker != b'}':
            startMarker = self.conn.recv(1)

    def fnReceivePhoneData(self):
        """
        Purpose:    Collects byte string.
        Returns:    Bytes string containing IMU data.
        """

        # Runs through byte-by-byte to reform complete JSON package through TCP
        if self.protocol is 'TCP':
            dataBuffer = []  # List of byte characters

            startMarker = self.conn.recv(1)  # Receives 1 byte
            dataBuffer.append(startMarker)  # Appends byte character to list

            while startMarker != b'}':
                startMarker = self.conn.recv(1)  # Receives 1 byte
                dataBuffer.append(startMarker)  # Appends byte character to list

            return b''.join(dataBuffer) # Return joined byte string to get JSON serialized data

        # Collects full message from UDP and returns
        elif self.protocol is 'UDP':
            data, addr = self.sock.recvfrom(128)
            return data

    def fnParsePhoneData(self, byteData, state = 'stream'):
        """
        Purpose:    Decodes byte string and stores data into relevant locations.
                    Also handles time synchronization.
        Passed:     Byte string for decoding and data storage.
                    State of data collection.
                        1. wait - record no data, allow for buffered messages to clear
                        2. init - set initial timestamp and time offset for time synchronization.
                        3. stream (default) - collect and store data.
        """

        # Reads byte data and saves to appropriate locations (JSON TCP)
        if self.protocol is 'TCP':
            dataParsed = json.loads(byteData.decode("utf-8")) # Decode JSON message

            # Sets timing adjustment variables
            if state == 'init':
                self.refTime = time.time()
                self.timeOffset = dataParsed['Timestamp']/1000

            # Collects data and stores into class lists, display data array
            elif state == 'stream':
                self.timeReceived.append(time.time())
                self.displayData[:, :] = np.roll(self.displayData, -1)
                self.displayData[0:7, -1] = [self.refTime + dataParsed['Timestamp']/1000 + self.timeOffset, dataParsed['accelerometer'][0], dataParsed['accelerometer'][1], dataParsed['accelerometer'][2],
                                             dataParsed['gyroscope-lsm6ds3'][0], dataParsed['gyroscope-lsm6ds3'],
                                             dataParsed['gyroscope-lsm6ds3'][2]]
                self.timeStamp.append(self.refTime + dataParsed['Timestamp']/1000 - self.timeOffset)
                self.xData.append(dataParsed['Accelerometer Sensor'][0])
                self.yData.append(dataParsed['Accelerometer Sensor'][1])
                self.zData.append(dataParsed['Accelerometer Sensor'][2])
                self.xGyro.append(dataParsed['Gyroscope Sensor'][0])
                self.yGyro.append(dataParsed['Gyroscope Sensor'][1])
                self.zGyro.append(dataParsed['Gyroscope Sensor'][2])

        # Reads byte data and saves to appropriate locations (UDP)
        elif self.protocol is 'UDP':
            dataParsed = byteData[:-2].split(sep=b',')  # Parses byte string into list, ignores \n at end of message
            dataParsed = [float(data) for data in dataParsed] # Converts values to float

            # Sets timing adjustment variables
            if state == 'init':
                self.refTime = time.time()
                self.timeOffset = dataParsed[0]/1000

            # Collects data and stores into class lists, display data array
            elif state == 'stream':
                self.timeReceived.append(time.time())
                self.displayData[:, :] = np.roll(self.displayData, -1)
                self.displayData[0:7, -1] = [self.refTime + dataParsed[0]/1000 - self.timeOffset, dataParsed[1], dataParsed[2],
                                             dataParsed[3], dataParsed[4], dataParsed[5], dataParsed[6]]
                self.timeStamp.append(self.refTime + dataParsed[0]/1000 + self.timeOffset)
                self.xData.append(dataParsed[1])
                self.yData.append(dataParsed[2])
                self.zData.append(dataParsed[3])
                self.xGyro.append(dataParsed[4])
                self.yGyro.append(dataParsed[5])
                self.zGyro.append(dataParsed[6])

    def fnRun(self):
        """
        Purpose:    Method that continuous runs and collects data.
        """

        freqCount = 0  # Frequency counter

        # Wait until start byte to collect data
        if self.protocol is 'TCP':
            self.fnClearInitial()

        # Allow for buffer messages to clear
        for i in range(500):
            dataReceived = self.fnReceivePhoneData()
            self.fnParsePhoneData(dataReceived, state = 'wait')

        # Initialize time offsets
        dataReceived = self.fnReceivePhoneData()
        self.fnParsePhoneData(dataReceived, state = 'init')

        # Run until connection ends
        while self.RunStatus:
            dataReceived = self.fnReceivePhoneData()
            self.fnParsePhoneData(dataReceived)
            freqCount += 1
            if freqCount >= 500:
                freqCount = 0
                print('Wheel Frequency: {} Hz'.format(500/(self.timeStamp[-1] - self.timeStamp[-501])))


    def fnSaveData(self, dataSource):
        """
        Purpose:    Method for saving data to CSV file.
        Passed:     dataSource dictionary that contains save path.
        """

        timeString = [datetime.datetime.fromtimestamp(utcTime).strftime('%Y-%m-%d %H:%M:%S:%f')[:-3] for utcTime in self.timeStamp]

        IMUData = pd.DataFrame({'ACCELEROMETER X (m/s²)': np.array(self.xData),
                     'ACCELEROMETER Y (m/s²)': np.array(self.yData),
                     'ACCELEROMETER Z (m/s²)': np.array(self.zData),
                     'GYROSCOPE X (rad/s)': np.array(self.xGyro),
                     'GYROSCOPE Y (rad/s)': np.array(self.yGyro),
                     'GYROSCOPE Z (rad/s)': np.array(self.zGyro),
                     'Time since start in ms ': np.array(self.timeStamp) - self.timeStamp[0],
                     'YYYY-MO-DD HH-MI-SS_SSS': timeString})

        IMUData.to_csv(dataSource['Path'], index = False)


if __name__ == "__main__":

    if not os.path.exists(os.path.join(dir_path, 'IMU Data')):
        os.mkdir(os.path.join(dir_path, 'IMU Data'))

    instDataPhoneParsing = ClPhoneDataParsing(LeftPhone)
    instDataPhoneParsing.fnRun()