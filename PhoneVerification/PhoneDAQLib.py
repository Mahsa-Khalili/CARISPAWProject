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
from multiprocessing import Queue

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
            'DisplayData': np.zeros((7, 1000)),
            'Queue': Queue(),
            'RunMarker': Queue()
            }

RightPhone = {'Name': 'Right Phone', 'Port': 6666, 'Placement': 'Right', 'Device': 'Phone',
              'AccPath': os.path.join('IMU Data', '{} RightPhoneAcc.csv'.format(
                  datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
              'GyroPath': os.path.join('IMU Data', '{} RightPhoneGyro.csv'.format(
                  datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
              'Path': '',
              'DisplayData': np.zeros((7, 1000)),
              'Queue': Queue(),
              'RunMarker': Queue()
              }

FramePhone = {'Name': 'Frame Phone', 'Port': 7777,'Placement': 'Middle', 'Device': 'Phone',
              'AccPath': os.path.join('IMU Data', '{} RightPhoneAcc.csv'.format(
                  datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
              'GyroPath': os.path.join('IMU Data', '{} RightPhoneGyro.csv'.format(
                  datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
               'Path': '',
              'DisplayData': np.zeros((7, 1000)),
              'Queue': Queue(),
              'RunMarker': Queue()
              }


# CLASSES

class ClPhoneDataParsing:
    """
    Class for connecting to Phone IMU
    """

    def __init__(self, dataSource, commPort = PORT, protocol = 'UDP'):
        """
        Purpose:    Initialize socket connections and class variables / lists.
        Passed:     Source dictionary for data storage.
                    Reserved port to listen to.
                    Data tranfer protocol. (UDP/TCP)
        """

        # Set self variables for displayData and protocol
        self.protocol = protocol
        self.commPort = commPort

        self.runStatus = dataSource['RunMarker']  # Queue to check when terminate signal is sent from main program

        self.Queue = dataSource['Queue'] # Queue for data transfer to main program

        self.path = dataSource['Path'] # Save path name

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
        self.heading = []
        self.pitch = []
        self.roll = []

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
        if self.protocol == 'TCP':
            dataBuffer = []  # List of byte characters

            startMarker = self.conn.recv(1)  # Receives 1 byte
            dataBuffer.append(startMarker)  # Appends byte character to list

            while startMarker != b'}':
                startMarker = self.conn.recv(1)  # Receives 1 byte
                dataBuffer.append(startMarker)  # Appends byte character to list

            return b''.join(dataBuffer) # Return joined byte string to get JSON serialized data

        # Collects full message from UDP and returns
        elif self.protocol == 'UDP':
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
        if self.protocol == 'TCP':
            dataParsed = json.loads(byteData.decode("utf-8")) # Decode JSON message

            # Sets timing adjustment variables
            if state == 'init':
                self.refTime = time.time()
                self.timeOffset = dataParsed['Timestamp']/1000

            # Collects data and stores into class lists, display data array
            elif state == 'stream':

                pitchAngle = dataParsed['CWGD Orientation Sensor'][0] + 180
                if pitchAngle > 180:
                    pitchAngle -= 360

                self.timeReceived.append(time.time())
                self.Queue.put([self.refTime + dataParsed['Timestamp']/1000 - self.timeOffset, dataParsed['accelerometer'][0], dataParsed['accelerometer'][1], dataParsed['accelerometer'][2],
                                             dataParsed['gyroscope-lsm6ds3'][0], dataParsed['gyroscope-lsm6ds3'],
                                             dataParsed['gyroscope-lsm6ds3'][2], pitchAngle, dataParsed['CWGD Orientation Sensor'][1], dataParsed['CWGD Orientation Sensor'][2]])

                self.timeStamp.append(self.refTime + dataParsed['Timestamp']/1000 - self.timeOffset)
                self.xData.append(dataParsed['Accelerometer Sensor'][0])
                self.yData.append(dataParsed['Accelerometer Sensor'][1])
                self.zData.append(dataParsed['Accelerometer Sensor'][2])
                self.xGyro.append(dataParsed['Gyroscope Sensor'][0])
                self.yGyro.append(dataParsed['Gyroscope Sensor'][1])
                self.zGyro.append(dataParsed['Gyroscope Sensor'][2])
                self.pitch.append(pitchAngle)
                self.roll.append(dataParsed['CWGD Orientation Sensor'][1])
                self.heading.append(dataParsed['CWGD Orientation Sensor'][2])

        # Reads byte data and saves to appropriate locations (UDP)
        elif self.protocol == 'UDP':

            dataParsed = byteData[:-2].split(sep=b',')  # Parses byte string into list, ignores \n at end of message
            dataParsed = [float(data) for data in dataParsed] # Converts values to float

            pitchAngle = dataParsed[8] + 180
            if pitchAngle > 180:
                pitchAngle -= 360

            # Sets timing adjustment variables
            if state == 'init':
                self.refTime = time.time()
                self.timeOffset = dataParsed[0]/1000

            # Collects data and stores into class lists, display data array
            elif state == 'stream':

                self.timeReceived.append(time.time())
                self.Queue.put([self.refTime + dataParsed[0]/1000 - self.timeOffset, -dataParsed[2], -dataParsed[1],
                                             -dataParsed[3], -dataParsed[4], -dataParsed[5], -dataParsed[6], dataParsed[7], -pitchAngle, dataParsed[9]])
                self.timeStamp.append(self.refTime + dataParsed[0]/1000 + self.timeOffset)
                self.xData.append(dataParsed[1])
                self.yData.append(dataParsed[2])
                self.zData.append(dataParsed[3])
                self.xGyro.append(dataParsed[4])
                self.yGyro.append(dataParsed[5])
                self.zGyro.append(dataParsed[6])
                self.pitch.append(pitchAngle)
                self.roll.append(dataParsed[9])
                self.heading.append(dataParsed[7])

    def fnRun(self):
        """
        Purpose:    Method that continuous runs and collects data.
        """

        # Connect using TCP if specified
        if self.protocol == 'TCP':
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.bind((HOST, self.commPort))
            self.sock.listen(1)
            self.conn, self.addr = self.sock.accept()
            self.conn.setblocking(1)
            print('Connected by {}'.format(self.addr))

        # Connect using UDP, default
        if self.protocol == 'UDP':
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print('Initialized.')
            self.sock.bind((HOST, self.commPort))

        freqCount = 0  # Frequency counter

        # Wait until start byte to collect data
        if self.protocol == 'TCP':
            self.fnClearInitial()

        # Allow for buffer messages to clear
        for i in range(500):
            dataReceived = self.fnReceivePhoneData()
            self.fnParsePhoneData(dataReceived, state = 'wait')

        # Initialize time offsets
        dataReceived = self.fnReceivePhoneData()
        self.fnParsePhoneData(dataReceived, state = 'init')
        dataReceived = self.fnReceivePhoneData()
        self.fnParsePhoneData(dataReceived)

        # Run until connection ends
        while self.runStatus.empty():
            dataReceived = self.fnReceivePhoneData()
            self.fnParsePhoneData(dataReceived)
            freqCount += 1
            if freqCount >= 500:
                freqCount = 0
                print('Phone Frequency: {} Hz'.format(500/(self.timeStamp[-1] - self.timeStamp[-501])))

        self.fnSaveData()

    def fnSaveData(self):
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
                     'Heading (deg)': np.array(self.heading),
                     'Pitch (deg)': np.array(self.pitch),
                     'Roll (deg)': np.array(self.roll),
                     'Time since start in ms ': np.array(self.timeStamp) - self.timeStamp[0],
                     'YYYY-MO-DD HH-MI-SS_SSS': timeString})

        IMUData.to_csv(self.path, index = False)


if __name__ == "__main__":

    if not os.path.exists(os.path.join(dir_path, 'IMU Data')):
        os.mkdir(os.path.join(dir_path, 'IMU Data'))

    instDataPhoneParsing = ClPhoneDataParsing(LeftPhone)
    instDataPhoneParsing.fnRun()