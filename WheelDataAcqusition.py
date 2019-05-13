"""
Author:         Kevin Ta
Date:           2019 May 7th
Purpose:        To provide a visualization for the data acquisition process.
"""

# IMPORTED LIBRARIES

import os
import datetime
import time
import numpy as np
import csv
import struct
import bluetooth
import pyqtgraph as pg
from cobs import cobs
from libraries.imumsg import imumsg_pb2 as imuMsg
from pyqtgraph.Qt import QtGui
from PyQt5 import QtCore
from multiprocessing import Process

import threading

# DEFINITIONS

dir_path = os.path.dirname(os.path.realpath(__file__))

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)

# displayDataLeft = np.zeros((6, 1000)).astype(np.float32)

RaspberryPi = {'Source': 'Pi', 'Address': 'B8:27:EB:6B:15:7F'}
Left = {'Name': 'Left', 'Address': '98:D3:51:FD:AD:F5',
        'DataPath': os.path.join('IMU Data', '{} rightIMUMessage.csv'.format(datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
        'DisplayData': np.zeros((6, 3000)).astype(np.float32)}
Right = {'Name': 'Right', 'Address': '98:D3:81:FD:48:C9',
         'DataPath': os.path.join('IMU Data', '{} leftIMUMessage.csv'.format(datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
         'DisplayData': np.zeros((6, 3000)).astype(np.float32)}

# CLASSES

class ClUIWrapper():

    def __init__(self, sources):

        self.sources = sources
        self.dataPath = {}
        self.loop = {}

        for dataSource in self.sources:
            self.dataPath[dataSource['Name']] = dataSource['DataPath']
            self.loop[dataSource['Name']] = ClWheelDataParsing(dataSource)

        ### START QtApp #####
        self.app = QtGui.QApplication([])  # you MUST do this once (initialize things)
        ####################

        self.canvas = ClDisplayDataQT(self.sources)

    def fnStart(self):

        thread = {}

        for dataSource in self.sources:
            thread[dataSource['Name']] = threading.Thread(target=self.loop[dataSource['Name']].fnRun)
            thread[dataSource['Name']].start()

        ### END QtApp ####
        self.app.exec_()  # you MUST put this at the end

        for dataSource in self.sources:
            np.savetxt(self.dataPath[dataSource['Name']], np.transpose([self.loop[dataSource['Name']].xData, self.loop[dataSource['Name']].yData,
                                                            self.loop[dataSource['Name']].zData, self.loop[dataSource['Name']].xGyro,
                                                            self.loop[dataSource['Name']].yGyro, self.loop[dataSource['Name']].zGyro]),
                       delimiter=",")


class ClDisplayDataQT:
    """
    Displays data using QT interface
    """
    def __init__(self, sources):

        self.sources = sources
        self.win = pg.GraphicsWindow(title="Received Signal(s)")  # creates a window
        self.win.resize(1000, 600)
        self.plotData = {}
        self.plot = {}

        # Enable antialiasing for prettier plots
        pg.setConfigOptions(antialias=True)

        for dataSource in self.sources:
            dataName = dataSource['Name']
            # self.sources[dataName] = dataSource

            self.plotData[dataName] = {}
            self.plot[dataName] = {}

            # for item in ['AccX', 'AccY', 'AccZ', 'AngX', 'AngY', 'AngZ']:
            for item in ['AccX', 'AccY', 'AngZ']:
                self.plotData[dataName][item] = self.win.addPlot(title="{} {} Acceleration".format(dataName, item))
                self.plot[dataName][item] = self.plotData[dataName][item].plot(pen=(255, 0, 0))

            self.win.nextRow()

        self.timer = QtCore.QTimer()
        self.timer.setInterval(10) # in milliseconds
        self.timer.start()
        self.timer.timeout.connect(self.fnUpdate)

    # Realtime data plot. Each time this function is called, the data display is updated
    def fnUpdate(self):
        for dataSource in self.sources:
            # for i, item in enumerate(['AccX', 'AccY', 'AccZ', 'AngX', 'AngY', 'AngZ']):
            for i, item in enumerate(['AccX', 'AccY', 'AngZ']):
                self.plot[dataSource['Name']][item].setData(dataSource['DisplayData'][i, :])  # set the curve with this data


class ClWheelDataParsing:
    """
    Connects to BT and parses data from Teensy wheel module.
    Stores data into csv file.
    Displays data using pyQTgraph.
    """

    def __init__(self, dataSource):
        """
        Initializes variables and CSV data storage.
        """

        self.displayData = dataSource['DisplayData']

        self.IMU = ClBluetoothConnect(dataSource['Address'])

        self.xData = []
        self.yData = []
        self.zData = []
        self.xGyro = []
        self.yGyro = []
        self.zGyro = []

        self.IMUStorage = dataSource['DataPath']

    def fnRun(self):
        """
        Main program that continuously runs.
        Decodes messages from BT signal.

        """

        status = 'Active.'

        self.IMU.COBSIntialClear()

        while status != 'Disconnected.':
            status = self.IMU.fnRetieveIMUMessage()
            # print("Message retrieval complete.")
            self.fnReceiveData(self.IMU.cobsMessage)
            # print('Running wheel module.')

        self.IMU.sock.close()

    def fnReceiveData(self, msg):
        """
        Unpacks data doming from Teensy and stores stores data into CSV.
        """

        dataSizeArray = msg[:4]
        dataSize = struct.unpack("<L", dataSizeArray)[0]
        # print(len(msg))
        # print(dataSize)
        data = msg[4:]
        imuMsgBT = imuMsg.IMUInfo()
        imuMsgBT.ParseFromString(data)

        # print("Value In onRecieveData: %f" % imuMsgBT.acc_x)
        # print("Value: %f" % imuMsgBT.acc_y)
        # print("Data from sensor " + imuMsgBT.sensorID)

        self.fnStoreData(imuMsgBT.acc_x, imuMsgBT.acc_y, imuMsgBT.acc_z, imuMsgBT.angular_x, imuMsgBT.angular_y, imuMsgBT.angular_z)

    def fnStoreData(self, xAcc, yAcc, zAcc, xGyro, yGyro, zGyro):
        """
        Stores data into CSV storage.
        """
        self.displayData[:,:] = np.roll(self.displayData, -1)
        self.displayData[0:6, -1] = [xAcc, yAcc, zAcc, xGyro, yGyro, zGyro]
        self.xData.append(xAcc)
        self.yData.append(yAcc)
        self.zData.append(zAcc)
        self.xGyro.append(xGyro)
        self.yGyro.append(yGyro)
        self.zGyro.append(zGyro)


class ClBluetoothConnect:

    def __init__(self, BTAddress):

        self.cobsMessage = ''
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

        print ("%s: Began connection" % datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

        self.sock.connect((BTAddress, 1))

        print ("%s: Established connection" % datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))


    def fnRetieveIMUMessage(self):

        data = []
        c = self.sock.recv(1)

        while c != b'\x00':
            if c == b'':
                self.onDisconnect()
                return "Disconnected."
            data.append(c)
            c = self.sock.recv(1)
        data = b''.join(data)
        try:
            self.cobsMessage = self.decodeCOBS(data)
            return "Received."
        except Exception as e:
            print("Failed to decode message due to {}".format(e))

    def onDisconnect(self):
        print("Disconnected from server.")
        self.shutDown()

    def decodeCOBS(self, encodedCobsMsg):
        # print("Encoded message length "+ str(len(encodedCobsMsg)))
        return cobs.decode(encodedCobsMsg)

    def shutDown(self):
        self.sock.close()
        self._running = False

    def COBSIntialClear(self):
        byte = self.receive(1)
        #Keep looping while byte recieved is not 0, i.e. the end/start of a cobs message.
        while ord(byte) != 0:
            #Keep looping while not 0
            byte = self.receive(1)
            print("Not 0")
            #Clear out potential initial garbage
            pass

    def receive(self, MSGLEN):
        chunks = []
        bytes_recd = 0
        while bytes_recd < MSGLEN:
            print("Waiting for msg")
            chunk = self.sock.recv(1)
            print(chunk[0])
            print(ord(chunk))
            if chunk == '':

                #raise RuntimeError("socket connection broken")
                print("socket connection broken shutting down this thread")
                self.shutDown()
                return 0

            chunks.append(chunk)
            bytes_recd = bytes_recd + len(chunk)
        return b''.join(chunks)


if __name__ == "__main__":

    # UIWrapLeft = ClUIWrapper(Left)
    # UIWrapRight = ClUIWrapper(Right)
    #
    # worker1 = Process(target=UIWrapLeft.fnStart)
    # worker1.start()
    # worker2 = Process(target=UIWrapRight.fnStart)
    # worker2.start()
    #
    # worker1.join()
    # worker2.join()

    UIWrap = ClUIWrapper([Left, Right])
    UIWrap.fnStart()

    name = input('What is your name? \n')
    pass