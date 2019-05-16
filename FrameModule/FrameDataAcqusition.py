"""
Author:         Kevin Ta
Date:           2019 May 7th
Purpose:        This code aims to do perform two primary objectives:

                1. Establish Bluetooth connection with Teensy wheel modules for data acquisition.
                2. Receive IMU data from Teensy wheel module for storage and real-time display.

                To do so, the code utilizes pybluez for bluetooth connection, cobs for byte en/decoding, and Google's
                protobuf protocol for serializing the structured daya. The protobuf interpreter can be found as imuMsg.

                The data is displayed using the PyQTgraph library, updating at 100 ms intervals. When the display window
                is closed, the code will than dump the data into a file found in the IMUdata subdirectory.
"""


# IMPORTED LIBRARIES

import os
import time
import datetime
import numpy as np
import struct
import socket
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
from PyQt5 import QtCore
import threading
import frameUnitMsg_pb2 as frameUnitMsg


# DEFINITIONS

dir_path = os.path.dirname(os.path.realpath(__file__))  # Current file directory

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)

# Python dictionaries storing name of data source, bluetooth address, data storage path, and the recorded data
RaspberryPi = {'Name': 'Pi', 'Address': 'B8:27:EB:6B:15:7F',
               'AccPath6050': os.path.join('IMU Data', '{} Frame6050Acc.csv'.format(
                   datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
               'GyroPath6050': os.path.join('IMU Data', '{} Frame6050Gyro.csv'.format(
                   datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
               'AccPath9250': os.path.join('IMU Data', '{} Frame9250Acc.csv'.format(
                   datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
               'GyroPath9250': os.path.join('IMU Data', '{} Frame9250Gyro.csv'.format(
                   datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
               'DisplayData6050': np.zeros((6, 1000)), 'DisplayData9250': np.zeros((6, 1000))}
Left = {'Name': 'Left', 'Address': '98:D3:51:FD:AD:F5',
        'AccPath': os.path.join('IMU Data', '{} rightAcc.csv'.format(
            datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
        'GyroPath': os.path.join('IMU Data', '{} rightGyro.csv'.format(
            datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
        'DisplayData': np.zeros((6, 1000))}
Right = {'Name': 'Right', 'Address': '98:D3:81:FD:48:C9',
         'AccPath': os.path.join('IMU Data', '{} leftAcc.csv'.format(
             datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
         'GyroPath': os.path.join('IMU Data', '{} leftGyro.csv'.format(
             datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
         'DisplayData': np.zeros((6, 1000))}

IMUDataDict = {'X Acceleration (G)': 0, 'Y Acceleration (G)': 1, 'Z Acceleration (G)': 2,
               'X Angular Velocity (rad/s)': 3, 'Y Angular Velocity (rad/s)': 4, 'Z Angular Velocity (rad/s)': 5}

# CLASSES

class ClUIWrapper():
    """
    Class for running wheel module data acquisition (ClWheelDataParsing) and real-time display (ClDisplayDataQT).
    """

    def __init__(self, sources):
        """
        Purpose:    Initialize class with sub-class structures and initial variables. Creates a parsing class
                    for every passed data source.
        Passed:     Sources of data (Left wheel and/or right wheel)
        """

        self.sources = sources  # Make globally set source dictionaries available to class
        self.DAQLoop = {}  # Initialize dictionary containing data acquisition

        for dataSource in self.sources:
            self.DAQLoop[dataSource['Name']] = ClFrameDataParsing(dataSource)

        self.app = QtGui.QApplication([])  # Initialize QT GUI, must only be called once

        self.canvas = ClDisplayDataQT(self.sources) # Initialize QT display class

    def fnStart(self):
        """
        Purpose:    Runs each specified wheel data acquisition loop in a separate thread.
                    Runs QT update display.
                    Dumps data in csv file when complete.
        Passed:     None
        TODO:       Look into switching from threading to multiprocessing.
        """
        threads = {}    # Initialize thread dictionary

        # Creates and starts each wheel module in a separate theead
        for dataSource in self.sources:
            threads[dataSource['Name']] = threading.Thread(target=self.DAQLoop[dataSource['Name']].fnRun)
            threads[dataSource['Name']].start()

        self.app.exec_()  # Executes QT display update code until window is closed, necessary for code to run

        # Stores data in IMUData folder, accelerometer and angular velocity stored in separate files
        for dataSource in self.sources:
            self.DAQLoop[dataSource['Name']].fnSaveData(dataSource)


class ClDisplayDataQT:
    """
    Class for displaying IMU data using QT interface.
    """

    def __init__(self, sources):
        """
        Purpose:    Initialize QT window and subplots with axes and titles.
                    Store source data based on which sources were passed. (Left and/or Right)
        Passsed:    Sources containing information on file storage path and stored value arrays.
        """

        self.sources = sources
        self.win = pg.GraphicsWindow(title="Received Signal(s)")  # creates a window
        self.win.resize(1200, 400 * len(sources)) # Resize window based on number of sources
        self.plot = {} # Create dictionary for subplots
        self.plotData = {} # Create dictionary for subplot data

        # Enable antialiasing for prettier plots
        pg.setConfigOptions(antialias=True)

        # Cycle through each data source and set-up plotting information
        for dataSource in self.sources:
            dataName = dataSource['Name']

            self.plotData[dataName] = {}
            self.plot[dataName] = {}

            # Cycle through relevant parameters and initialze subplots
            # for item in ['X Acceleration (G)', 'Y Acceleration (G)', , 'Z Acceleration (G)',
            # 'X Angular Velocity (rad/s)', 'Y Angular Velocity (rad/s)', 'Z Angular Velocity (rad/s)']:
            for item in ['X Acceleration (G)', 'Y Acceleration (G)', 'Z Angular Velocity (rad/s)']:
                self.plot[dataName][item] = self.win.addPlot(title="{} {}".format(dataName, item))
                self.plotData[dataName][item] = self.plot[dataName][item].plot(pen=(255, 0, 0))

            # Create new row for each source
            self.win.nextRow()

        # Set update period for display, lowering setInterval requires more processing and leads to more issues
        self.timer = QtCore.QTimer()
        self.timer.setInterval(100) # in milliseconds
        self.timer.start()
        self.timer.timeout.connect(self.fnUpdate) # Sets timer to trigger fnUpdate

    # Realtime data plot. Each time this function is called, the data display is updated
    def fnUpdate(self):
        """
        Purpose:    Access display data arrays and displays results in QT interface.
        Passed:     None
        """

        # Cycles through sources and update plots
        for dataSource in self.sources:
            # for item in ['X Acceleration (G)', 'Y Acceleration (G)', , 'Z Acceleration (G)',
            # 'X Angular Velocity (rad/s)', 'Y Angular Velocity (rad/s)', 'Z Angular Velocity (rad/s)']:
            for item in ['X Acceleration (G)', 'Y Acceleration (G)', 'Z Angular Velocity (rad/s)']:
                self.plotData[dataSource['Name']][item].setData(dataSource['DisplayData'][IMUDataDict[item], :])


class ClFrameDataParsing:
    """
    Class that instantiates ClTCPServer to connect to Pi, receiving MPU6050/9250 data.
    """

    def __init__(self, dataSource):
        """
        Purpose:
        Passed:
        """

        self.displayData6050 = dataSource['DisplayData6050'] # Make shared display data variable accessible
        self.displayData9250 = dataSource['DisplayData9250']  # Make shared display data variable accessible

        self.FrameUnit = ClTCPServer() # Create TCP Server

        # Create class storage variables
        self.timeStamp = []
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

    def fnSaveData(self, dataSource):

        AccData6050 = np.transpose([self.timeStamp, self.xData6050, self.yData6050, self.zData6050])
        GyroData6050 = np.transpose([self.timeStamp, self.xGyro6050, self.yGyro6050, self.zGyro6050])
        AccData9250 = np.transpose([self.timeStamp, self.xData9250, self.yData9250, self.zData9250])
        GyroData9250 = np.transpose([self.timeStamp, self.xGyro9250, self.yGyro9250, self.zGyro9250])
        np.savetxt(dataSource['AccPath6050'], AccData6050, delimiter=",")
        np.savetxt(dataSource['GyroPath6050'], GyroData6050, delimiter=",")
        np.savetxt(dataSource['AccPath9250'], AccData9250, delimiter=",")
        np.savetxt(dataSource['GyroPath9250'], GyroData9250, delimiter=",")

    def fnRun(self):
        """
        Purpose:
        Passed:
        """

        timeStart = time.time()

        while (time.time() < timeStart + 30):
            self.fnReceiveData()

        self.FrameUnit.fnShutDown()

    def fnReceiveData(self):
        """
        Purpose:
        Passed:
        """

        print(self.FrameUnit.fnRetievePiData())

    def fnStoreData(self, timeStamp, xAcc, yAcc, zAcc, xGyro, yGyro, zGyro):
        """
        Purpose:    Store data into display data and class variables.
        Passed:     Teensy time values, (x, y, z) acceleration in Gs, (x, y, z) angular velocity in rad/s.
        TODO:       Look at efficiency of roll and if using indexing would be faster.
        """
        self.displayData6050[:,:] = np.roll(self.displayData6050, -1)
        self.displayData6050[0:6, -1] = [xAcc, yAcc, zAcc, xGyro, yGyro, zGyro]
        self.timeStamp.append(timeStamp)
        self.xData6050.append(xAcc)
        self.yData6050.append(yAcc)
        self.zData6050.append(zAcc)
        self.xGyro6050.append(xGyro)
        self.yGyro6050.append(yGyro)
        self.zGyro6050.append(zGyro)


class ClTCPServer:
    """
    Class
    """

    def __init__(self):
        """
        Purpose:
        Passed:
        """
        self.TCPSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        print ("{}: Began connection".format(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))

        self.TCPSocket.bind((HOST, PORT))
        self.TCPSocket.listen(1)
        self.conn, self.addr = self.TCPSocket.accept()

        print ("{}: Established connection".format(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))


    def fnRetievePiData(self):
        """
        Purpose:
        Passed:
        Return:
        """
        return self.conn.recv(20)


    def fnShutDown(self):
        """
        Purpose:    Close socket connections on shutdown.
        """

        print("Disconnecting server.")
        self.TCPSocket.close()


if __name__ == "__main__":

    # Run user interface for data collection
    UIWrap = ClUIWrapper([RaspberryPi])
    UIWrap.fnStart()
    print(input('What is your name? \n'))
    pass