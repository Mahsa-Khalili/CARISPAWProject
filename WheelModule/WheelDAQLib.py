"""
Author:         Kevin Ta
Date:           2019 May 7th
Purpose:        This library aims to do perform two primary objectives:

                1. Establish Bluetooth connection with Teensy wheel modules for data acquisition.
                2. Parse and store data.

                To do so, the code utilizes pybluez for bluetooth connection, cobs for byte en/decoding, and Google's
                protobuf protocol for serializing the structured data. The protobuf interpreter can be found as imuMsg.

                The data is displayed using the PyQTgraph library, updating at 100 ms intervals. When the display window
                is closed, the code will than dump the data into a file found in the IMU Data subdirectory.
"""


# IMPORTED LIBRARIES

import os
import datetime
import numpy as np
import struct
import bluetooth
import pyqtgraph as pg
from cobs import cobs
from libraries.imumsg import imumsg_pb2 as imuMsg
from pyqtgraph.Qt import QtGui
from PyQt5 import QtCore
import threading
import math
import time


# DEFINITIONS

dir_path = os.path.dirname(os.path.realpath(__file__))  # Current file directory

# Python dictionaries storing name of data source, bluetooth address, data storage path, and the recorded data
Left = {'Name': 'Left', 'Address': '98:D3:51:FD:AD:F5',
        'AccPath': os.path.join('IMU Data', '{} leftAcc.csv'.format(
            datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
        'GyroPath': os.path.join('IMU Data', '{} leftGyro.csv'.format(
            datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
        'DisplayData': np.zeros((7, 1000))}
Right = {'Name': 'Right', 'Address': '98:D3:81:FD:48:C9',
         'AccPath': os.path.join('IMU Data', '{} rightAcc.csv'.format(
             datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
         'GyroPath': os.path.join('IMU Data', '{} rightGyro.csv'.format(
             datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
         'DisplayData': np.zeros((7, 1000))}

# Dictionary containing
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

        self.sources = sources  # Make passed list of sources available to class
        self.wheelDAQLoop = {}  # Initialize dictionary containing wheel data acquisition loops

        for dataSource in self.sources:
            self.wheelDAQLoop[dataSource['Name']] = ClWheelDataParsing(dataSource)

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
            threads[dataSource['Name']] = threading.Thread(target=self.wheelDAQLoop[dataSource['Name']].fnRun)
            threads[dataSource['Name']].start()

        self.app.exec_()  # Executes QT display update code until window is closed, necessary for code to run

        # Stores data in IMUData folder, accelerometer and angular velocity stored in separate files
        for dataSource in self.sources:
            AccData = np.transpose([self.wheelDAQLoop[dataSource['Name']].timeStamp,
                                    self.wheelDAQLoop[dataSource['Name']].xData,
                                    self.wheelDAQLoop[dataSource['Name']].yData,
                                    self.wheelDAQLoop[dataSource['Name']].zData])
            GyroData = np.transpose([self.wheelDAQLoop[dataSource['Name']].timeStamp,
                                     self.wheelDAQLoop[dataSource['Name']].xGyro,
                                     self.wheelDAQLoop[dataSource['Name']].yGyro,
                                     self.wheelDAQLoop[dataSource['Name']].zGyro])
            np.savetxt(dataSource['AccPath'], AccData, delimiter=",")
            np.savetxt(dataSource['GyroPath'], GyroData, delimiter=",")


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

        self.displayData = dataSource['DisplayData'] # Make shared display data variable accessible

        self.IMU = ClBluetoothConnect(dataSource['Address']) # Creates bluetooth connection instance with wheel module

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

        status = 'Active.' # Set marker to active

        self.IMU.fnCOBSIntialClear() # Wait until message received starts at the correct location

        # Cycle through data retrieval until bluetooth disconnects
        for i in range(1000):
            if status != 'Disconnected.':
                status = self.IMU.fnRetieveIMUMessage()
                self.fnReceiveData(self.IMU.cobsMessage, 'wait')

        if status != 'Disconnected.':
            status = self.IMU.fnRetieveIMUMessage()
            self.fnReceiveData(self.IMU.cobsMessage, 'init')

        # Cycle through data retrieval until bluetooth disconnects
        while status != 'Disconnected.':
            status = self.IMU.fnRetieveIMUMessage()
            self.fnReceiveData(self.IMU.cobsMessage)

        # Close socket connection
        # TODO: Make socket terminate / escape from loop above when exiting display window.
        self.IMU.sock.close()


    def fnReceiveData(self, msg, state = 'stream'):
        """
        Purpose:    Unpack data coming from Teensy wheel module and calls fnStoreData to store data.
        Passed:     Cobs deciphered byte string message.
        """

        # Try to decipher message based on preset protobuf specifications
        try:
            # Get data dize
            # dataSizeArray = msg[:4]
            # dataSize = struct.unpack("<L", dataSizeArray)[0]

            # Pass msg to imuMsg to parse into float values stored in imuMsg instance
            data = msg[4:]
            imuMsgBT = imuMsg.IMUInfo()
            imuMsgBT.ParseFromString(data)

            if state == 'init':
                self.refTime = time.time()
                self.timeOffset = imuMsgBT.time_stamp / 1000000
            elif state == 'stream':
                self.timeReceived.append(time.time())
                self.fnStoreData(self.refTime + imuMsgBT.time_stamp / 1000000 - self.timeOffset, imuMsgBT.acc_x * 9.8065, imuMsgBT.acc_y * 9.8065, imuMsgBT.acc_z * 9.8065,
                                 imuMsgBT.angular_x * math.pi / 180, imuMsgBT.angular_y * math.pi / 180, imuMsgBT.angular_z * math.pi / 180)
            # self.fnStoreData(imuMsgBT.time_stamp / 1000000, imuMsgBT.acc_x * 9.8065, imuMsgBT.acc_y * 9.8065, imuMsgBT.acc_z * 9.8065,
            #                  imuMsgBT.angular_x * math.pi / 180, imuMsgBT.angular_y * math.pi / 180, imuMsgBT.angular_z * math.pi / 180)

        # Returns exceptions as e to avoid code crash but still allow for debugging
        except Exception as e:
            print(e)

    def fnStoreData(self, timeStamp, xAcc, yAcc, zAcc, xGyro, yGyro, zGyro):
        """
        Purpose:    Store data into display data and class variables.
        Passed:     Teensy time values, (x, y, z) acceleration in Gs, (x, y, z) angular velocity in rad/s.
        TODO:       Look at efficiency of roll and if using indexing would be faster.
        """
        self.displayData[:,:] = np.roll(self.displayData, -1)
        self.displayData[0:7, -1] = [time.time(), xAcc, yAcc, zAcc, xGyro, yGyro, zGyro]
        self.timeStamp.append(timeStamp)
        self.xData.append(xAcc)
        self.yData.append(yAcc)
        self.zData.append(zAcc)
        self.xGyro.append(xGyro)
        self.yGyro.append(yGyro)
        self.zGyro.append(zGyro)

    def fnSaveData(self, dataSource):

        AccData = np.transpose(np.array([self.timeReceived, self.timeStamp, self.xData, self.yData, self.zData]))
        GyroData = np.transpose(np.array([self.timeReceived, self.timeStamp, self.xGyro, self.yGyro, self.zGyro]))
        np.savetxt(dataSource['AccPath'], AccData.astype(float), delimiter = ",")
        np.savetxt(dataSource['GyroPath'], GyroData.astype(float), delimiter = ",")


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

    # Run user interface for data collection
    UIWrap = ClUIWrapper([Left])
    UIWrap.fnStart()
    print(input('What is your name? \n'))
    pass