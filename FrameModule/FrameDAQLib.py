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
import math
import socket
import bluetooth
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

# CLASSES

class ClFrameDataParsing:
    """
    Class that instantiates ClTCPServer to connect to Pi, receiving MPU6050/9250 data.
    """

    def __init__(self, dataSource, commPort = PORT, protocol = 'UDP'):
        """
        Purpose:
        Passed:
        """

        self.protocol = protocol

        # Initialize class variables
        self.refTime = 0
        self.timeOffset = 0

        self.displayData = dataSource['DisplayData'] # Make shared display data variable accessible

        self.FrameUnit = ClWifiServer(dataSource, self.protocol) # Create TCP Server

        # Create class storage variables
        self.timeReceived = []
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

        if self.protocol == 'TCP':
            self.FrameUnit.fnCOBSIntialClear() # Wait until message received starts at the correct location

        for i in range(500):
            if status != 'Disconnected.':
                status = self.FrameUnit.fnRetievePiMessage()
                self.fnReceiveData(self.FrameUnit.cobsMessage, state = 'wait')

        if status != 'Disconnected.':
            status = self.FrameUnit.fnRetievePiMessage()
            self.fnReceiveData(self.FrameUnit.cobsMessage, state='init')

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

    def fnReceiveData(self, msg, state = 'stream'):
        """
        Purpose:    Unpack data coming from Teensy wheel module and calls fnStoreData to store data.
        Passed:     Cobs deciphered byte string message.
        """

        # Try to decipher message based on preset protobuf specifications
        try:

            # Pass msg to frameUnitMsg to parse into float values stored in imuMsg instance
            data = msg
            frameUnitMsgRcv = frameUnitMsg.frameUnit()
            frameUnitMsgRcv.ParseFromString(data)

            if state == 'init':
                self.refTime = time.time()
                self.timeOffset = frameUnitMsgRcv.time_stamp

            elif state == 'stream':
                self.timeReceived.append(time.time())
                # Append data to display data and class variables
                self.fnStoreData(frameUnitMsgRcv)

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
        self.displayData[0:7, -1] = [self.refTime + frameUnitPB.time_stamp - self.timeOffset, frameUnitPB.acc_x_9250, frameUnitPB.acc_y_9250, frameUnitPB.acc_z_9250,
                                         frameUnitPB.angular_x_9250 * math.pi / 180, frameUnitPB.angular_y_9250 * math.pi / 180,
                                         frameUnitPB.angular_z_9250 * math.pi / 180]
        self.displayData[7:13, -1] = [frameUnitPB.acc_x_6050, frameUnitPB.acc_y_6050, frameUnitPB.acc_z_6050,
                                         frameUnitPB.angular_x_6050 * math.pi / 180, frameUnitPB.angular_y_6050 * math.pi / 180,
                                         frameUnitPB.angular_z_6050 * math.pi / 180]

        self.timeStamp.append(self.refTime + frameUnitPB.time_stamp - self.timeOffset)
        self.xData6050.append(frameUnitPB.acc_x_6050)
        self.yData6050.append(frameUnitPB.acc_y_6050)
        self.zData6050.append(frameUnitPB.acc_z_6050)
        self.xGyro6050.append(frameUnitPB.angular_x_6050 * math.pi / 180)
        self.yGyro6050.append(frameUnitPB.angular_y_6050 * math.pi / 180)
        self.zGyro6050.append(frameUnitPB.angular_z_6050 * math.pi / 180)

        self.xData9250.append(frameUnitPB.acc_x_9250)
        self.yData9250.append(frameUnitPB.acc_y_9250)
        self.zData9250.append(frameUnitPB.acc_z_9250)
        self.xGyro9250.append(frameUnitPB.angular_x_9250 * math.pi / 180)
        self.yGyro9250.append(frameUnitPB.angular_y_9250 * math.pi / 180)
        self.zGyro9250.append(frameUnitPB.angular_z_9250 * math.pi / 180)

        def fnSaveData(self, dataSource):

            AccData = np.transpose([np.array(self.timeStamp), np.array(self.xData9250), np.array(self.yData9250), np.array(self.zData9250)])
            GyroData = np.transpose([np.array(self.timeStamp), np.array(self.xGyro9250), np.array(self.yGyro9250), np.array(self.zGyro9250)])
            np.savetxt(dataSource['AccPath9250'], AccData, delimiter=",")
            np.savetxt(dataSource['GyroPath9250'], GyroData, delimiter=",")

class ClWifiServer:
    """
    Class
    """

    def __init__(self, dataSource, protocol):
        """
        Purpose:
        Passed:
        """
        self.protocol = protocol
        self.cobsMessage = ''  # Create variable for storing COBS decoded message

        if self.protocol == 'TCP':
            self.TCPSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            print ("{}: Began connection".format(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))

            self.TCPSocket.bind((HOST, dataSource['Port']))
            self.TCPSocket.listen(1)
            self.socket, self.addr = self.TCPSocket.accept()

            print ("{}: Established connection".format(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))

        elif self.protocol == 'UDP':

            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print('Initialized.')
            self.socket.bind((HOST, dataSource['Port']))

        elif self.protocol =='BT':
            self.socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            print('Initialized.')
            self.socket.connect((dataSource['Address'], 3))
            # self.sock.bind(('', 3))
            # self.sock.listen(1)
            # self.socket, self.clienttInfo = self.sock.accept()

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

        if self.protocol == 'TCP' or 'BT':
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

    # Run user interface for data collection
    print(input('What is your name? \n'))
    pass