"""
Author:         Kevin Ta
Date:           2019 May 24th
Purpose:        This library receives data from HyperIMU through TCP (JSON) or UDP (Byte string).
"""


# IMPORTED LIBRARIES

import socket
import os
import json
import datetime
import time
import numpy as np

# DEFINITIONS

HOST = ''              # Symbolic name meaning all available interfaces
PORT = 5555            # Arbitrary non-privileged port

dir_path = os.path.dirname(os.path.realpath(__file__))  # Current file directory

# Phone information dictionary
Phone = {'Name': 'Phone', 'Address': '',
         'AccPath': os.path.join('IMU Data', '{} phoneAcc.csv'.format(
             datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
         'GyroPath': os.path.join('IMU Data', '{} phoneGyro.csv'.format(
             datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
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
                    Data tranfer protocol.
        """

        # Set self variables for displayData and protocol
        self.displayData = source['DisplayData']
        self.protocol = protocol

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
        Purpose:    Cycle through received bytes until ending curly brace is detected from HyperIMU.
                    Prevents premature reading of byte stream.
        """

        startMarker = b''

        while startMarker != b'}':
            startMarker = self.conn.recv(1)

    def fnReceivePhoneData(self):
        """
        Purpose:    Collects byte string.
        Returns:    Bytes tring containing IMU data.
        """

        # Runs through byte-by-byte to reform complete JSON package through TCP
        if self.protocol is 'TCP':
            dataBuffer = []

            startMarker = self.conn.recv(1)
            dataBuffer.append(startMarker)

            while startMarker != b'}':
                startMarker = self.conn.recv(1)
                dataBuffer.append(startMarker)

            return b''.join(dataBuffer)

        # Collects full message from UDP
        elif self.protocol is 'UDP':
            data, addr = self.sock.recvfrom(128)
            return data

    def fnParsePhoneData(self, byteData, state = 'stream'):
        """
        Purpose:    Decodes byte string and stores data into relevant locations.
        Passed:     Byte string for decoding and data storage.
        """


        # Reads byte data and saves to appropriate locations
        if self.protocol is 'TCP':
            dataParsed = json.loads(byteData.decode("utf-8"))

            if state == 'init':
                self.refTime = time.time()
                self.timeOffset = dataParsed['Timestamp']/1000

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

        # Reads byte data and saves to appropriate locations
        elif self.protocol is 'UDP':
            dataParsed = byteData[:-2].split(sep=b',')
            dataParsed = [float(data) for data in dataParsed]

            if state == 'init':
                self.refTime = time.time()
                self.timeOffset = dataParsed[0]/1000

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

        if self.protocol is 'TCP':
            self.fnClearInitial()


        for i in range(500):
            dataReceived = self.fnReceivePhoneData()
            self.fnParsePhoneData(dataReceived, state = 'wait')

            dataReceived = self.fnReceivePhoneData()
            self.fnParsePhoneData(dataReceived, state = 'init')

        while True:
        # for i in range(1000):
            dataReceived = self.fnReceivePhoneData()
            self.fnParsePhoneData(dataReceived)

        self.fnSaveData(Phone)


    def fnSaveData(self, dataSource):
        """
        Purpose:    Method for saving data to CSV file.
        Passed:     dataSource dictionary that contains save path.
        """

        AccData = np.transpose(np.array([self.timeReceived, self.timeStamp, self.xData, self.yData, self.zData]))
        GyroData = np.transpose(np.array([self.timeReceived, self.timeStamp, self.xGyro, self.yGyro, self.zGyro]))
        np.savetxt(dataSource['AccPath'], AccData.astype(float), delimiter=",")
        np.savetxt(dataSource['GyroPath'], GyroData.astype(float), delimiter=",")


if __name__ == "__main__":

    if not os.path.exists(os.path.join(dir_path, 'IMU Data')):
        os.mkdir(os.path.join(dir_path, 'IMU Data'))

    instDataPhoneParsing = ClPhoneDataParsing(Phone)
    instDataPhoneParsing.fnRun()

    pass