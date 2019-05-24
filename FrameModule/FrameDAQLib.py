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
import struct
import socket
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
from PyQt5 import QtCore
import threading
import frameUnitMsg_pb2 as frameUnitMsg
from cobs import cobs


# DEFINITIONS

dir_path = os.path.dirname(os.path.realpath(__file__))  # Current file directory

HOST = ''           # Accept all connections
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
        Passed:     Sources of data (Raspberry Pi)
        """

        self.sources = sources  # Make globally set source dictionaries available to class
        self.DAQLoop = {}  # Initialize dictionary containing data acquisition

        # Initialize every passed data module
        for dataSource in self.sources:
            self.DAQLoop[dataSource['Name']] = ClFrameDataParsing(dataSource)

        self.app = QtGui.QApplication([])  # Initialize QT GUI, must only be called once

        self.canvas = ClDisplayDataQT(self.sources) # Initialize QT display class

    def fnStart(self):
        """
        Purpose:    Runs each data acquisition loop in a separate thread.
                    Runs QT update display.
                    Dumps data in csv file when complete.
        Passed:     None
        TODO:       Look into switching from threading to multiprocessing.
        """
        threads = {}    # Initialize thread dictionary

        # Creates and starts each module in a separate thread
        for dataSource in self.sources:
            threads[dataSource['Name']] = threading.Thread(target=self.DAQLoop[dataSource['Name']].fnRun)
            threads[dataSource['Name']].start()

        self.app.exec_()  # Executes QT display update code until window is closed, necessary for code to run

        # Stores data in IMU Data folder, accelerometer and angular velocity stored in separate files
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
            dataName6050 = dataSource['Name'] + ' 6050'
            dataName9250 = dataSource['Name'] + ' 9250'

            self.plotData[dataName6050] = {}
            self.plot[dataName6050] = {}
            self.plotData[dataName9250] = {}
            self.plot[dataName9250] = {}

            # Cycle through relevant parameters and initialize subplots
            # for item in ['X Acceleration (G)', 'Y Acceleration (G)', , 'Z Acceleration (G)',
            # 'X Angular Velocity (rad/s)', 'Y Angular Velocity (rad/s)', 'Z Angular Velocity (rad/s)']:
            for item in ['Y Acceleration (G)', 'Z Acceleration (G)', 'X Angular Velocity (rad/s)']:
                self.plot[dataName6050][item] = self.win.addPlot(title="{} {}".format(dataName6050, item))
                self.plotData[dataName6050][item] = self.plot[dataName6050][item].plot(pen=(255, 0, 0))

            # Create new row for each source
            self.win.nextRow()

            for item in ['Y Acceleration (G)', 'Z Acceleration (G)', 'X Angular Velocity (rad/s)']:
                self.plot[dataName9250][item] = self.win.addPlot(title="{} {}".format(dataName9250, item))
                self.plotData[dataName9250][item] = self.plot[dataName9250][item].plot(pen=(255, 0, 0))

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
            for item in ['Y Acceleration (G)', 'Z Acceleration (G)', 'X Angular Velocity (rad/s)']:
                self.plotData[dataSource['Name'] + ' 6050'][item].setData(dataSource['DisplayData6050'][IMUDataDict[item], :])
                self.plotData[dataSource['Name'] + ' 9250'][item].setData(dataSource['DisplayData9250'][IMUDataDict[item], :])


class ClFrameDataParsing:
    """
    Class that instantiates ClTCPServer to connect to Pi, receiving MPU6050/9250 data.
    """

    def __init__(self, dataSource):
        """
        Purpose:
        Passed:
        """

        self.displayData = dataSource['DisplayData'] # Make shared display data variable accessible

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

    def fnRun(self):
        """
        Purpose:    Main program that continuously runs.
                    Decodes messages from TCP and stores data.
        Passed:     None.
        """

        status = 'Active.' # Set marker to active

        self.FrameUnit.fnCOBSIntialClear() # Wait until message received starts at the correct location

        # Cycle through data retrieval until bluetooth disconnects
        while status != 'Disconnected.':
            status = self.FrameUnit.fnRetievePiMessage()
            self.fnReceiveData(self.FrameUnit.cobsMessage)

        # Close socket connection
        # TODO: Make socket terminate / escape from loop above when exiting display window.
        self.FrameUnit.fnShutDown()

    def fnSaveData(self, dataSource):

        AccData6050 = np.transpose([self.timeStamp, self.xData6050, self.yData6050, self.zData6050])
        GyroData6050 = np.transpose([self.timeStamp, self.xGyro6050, self.yGyro6050, self.zGyro6050])
        AccData9250 = np.transpose([self.timeStamp, self.xData9250, self.yData9250, self.zData9250])
        GyroData9250 = np.transpose([self.timeStamp, self.xGyro9250, self.yGyro9250, self.zGyro9250])
        np.savetxt(dataSource['AccPath6050'], AccData6050, delimiter=",")
        np.savetxt(dataSource['GyroPath6050'], GyroData6050, delimiter=",")
        np.savetxt(dataSource['AccPath9250'], AccData9250, delimiter=",")
        np.savetxt(dataSource['GyroPath9250'], GyroData9250, delimiter=",")

    def fnReceiveData(self, msg):
        """
        Purpose:    Unpack data coming from Teensy wheel module and calls fnStoreData to store data.
        Passed:     Cobs deciphered byte string message.
        """

        # Try to decipher message based on preset protobuf specifications
        try:

            # Pass msg to frameUnitMsg to parse into float values stored in imuMsg instance
            data = msg
            frameUnitMsgTCP = frameUnitMsg.frameUnit()
            frameUnitMsgTCP.ParseFromString(data)
            # Append data to display data and class variables
            self.fnStoreData(frameUnitMsgTCP)

        # Returns exceptions as e to avoid code crash but still allow for debugging
        except Exception as e:
            print (e)

    def fnStoreData(self, frameUnitPB):
        """
        Purpose:    Store data into display data and class variables.
        Passed:     Teensy time values, (x, y, z) acceleration in Gs, (x, y, z) angular velocity in rad/s.
        TODO:       Look at efficiency of roll and if using indexing would be faster.
        """
        self.displayData[:,:] = np.roll(self.displayData, -1)
        self.displayData[0:7, -1] = [frameUnitPB.time_stamp, frameUnitPB.acc_x_9250, frameUnitPB.acc_y_9250, frameUnitPB.acc_z_9250,
                                         frameUnitPB.angular_x_9250, frameUnitPB.angular_y_9250,
                                         frameUnitPB.angular_z_9250]
        self.displayData[7:12, -1] = [frameUnitPB.acc_x_6050, frameUnitPB.acc_y_6050, frameUnitPB.acc_z_6050,
                                         frameUnitPB.angular_x_6050, frameUnitPB.angular_y_6050,
                                         frameUnitPB.angular_z_6050]

        self.timeStamp.append(frameUnitPB.time_stamp)
        self.xData6050.append(frameUnitPB.acc_x_6050)
        self.yData6050.append(frameUnitPB.acc_y_6050)
        self.zData6050.append(frameUnitPB.acc_z_6050)
        self.xGyro6050.append(frameUnitPB.angular_x_6050)
        self.yGyro6050.append(frameUnitPB.angular_y_6050)
        self.zGyro6050.append(frameUnitPB.angular_z_6050)

        self.xData9250.append(frameUnitPB.acc_x_9250)
        self.yData9250.append(frameUnitPB.acc_y_9250)
        self.zData9250.append(frameUnitPB.acc_z_9250)
        self.xGyro9250.append(frameUnitPB.angular_x_9250)
        self.yGyro9250.append(frameUnitPB.angular_y_9250)
        self.zGyro9250.append(frameUnitPB.angular_z_9250)

        def fnSaveData(self, dataSource):

            AccData = np.transpose([self.timeStamp, self.xData9250, self.yData9250, self.zData9250])
            GyroData = np.transpose([self.timeStamp, self.xGyro9250, self.yGyro9250, self.zGyro9250])
            np.savetxt(dataSource['AccPath9250'], AccData, delimiter=",")
            np.savetxt(dataSource['GyroPath9250'], GyroData, delimiter=",")

class ClTCPServer:
    """
    Class
    """

    def __init__(self):
        """
        Purpose:
        Passed:
        """
        self.cobsMessage = ''  # Create variable for storing COBS decoded message
        self.TCP = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        print ("{}: Began connection".format(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))

        self.TCP.bind((HOST, PORT))
        self.TCP.listen(1)
        self.TCPSocket, self.addr = self.TCP.accept()

        print ("{}: Established connection".format(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))

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
        self.TCPSocket.close()

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
            chunk = self.TCPSocket.recv(1)
            print(chunk[0])
            print(ord(chunk))

            if chunk == '':
                print("socket connection broken shutting down this thread")
                self.TCPSocket.close()
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
        data = []  # List containing characters of byte string
        c = self.TCPSocket.recv(1)  # Receive 1 byte of information

        # Continue acquiring bytes of data until end point is reached. Combine into byte string.
        while c != b'\x00':
            if c == b'':
                self.TCPSocket.close()
                return "Disconnected."
            data.append(c)
            c = self.TCPSocket.recv(1)
        data = b''.join(data)

        # Try to decode message and returnes exception to avoid closing the program
        try:
            self.cobsMessage = cobs.decode(data)
            return 'Received.'
        except Exception as e:
            print("Failed to decode message due to {}".format(e))

if __name__ == "__main__":

    # Run user interface for data collection
    UIWrap = ClUIWrapper([RaspberryPi])
    UIWrap.fnStart()
    print(input('What is your name? \n'))
    pass