"""
Author:         Kevin Ta
Date:           2019 May 7th
Purpose:        To provide a visualization for the data acquisition process.
"""

# IMPORTED LIBRARIES

import os
import datetime
import time
import imp
import csv
import struct
import bluetooth
from cobs import cobs

# DEFINITIONS

dir_path = os.path.dirname(os.path.realpath(__file__))
rightIMUDataFilePath = os.path.join(dir_path, 'IMU Data', '%s_rightIMUMessage.csv' % datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
leftIMUDataFilePath = os.path.join(dir_path, 'IMU Data', '%s_leftIMUMessage.csv' % datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

imuMsgPath = os.path.join(dir_path, 'release', 'imumsg_pb2.py')
print(imuMsgPath)
imuMsg = imp.load_source("imumsg_pb2",imuMsgPath)

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)
bluetoothHost = 1
bluetoothPort = 2
RaspberryPiAddress = "B8:27:EB:6B:15:7F"

Left = {'Address': "98:D3:51:FD:AD:F5", "DataPath": leftIMUDataFilePath}
Right = {'Address': "98:D3:81:FD:48:C9", "DataPath": rightIMUDataFilePath}

# CLASSES

class ClWheelDataParsing:
    """
    Connects to BT and parses data from Teensy wheel module.
    Stores data into csv file.
    Displays data using pyQTgraph.
    """

    def __init__(self, source):
        """
        Initializes variables and CSV data storage.
        """

        self.IMU = ClBluetoothConnect(source['Address'])

        self.xData = []
        self.yData = []
        self.zData = []
        self.xGyro = []
        self.yGyro = []
        self.zGyro = []

        self.IMUStorage = open(source['DataPath'], mode='w')
        self.IMUCSV = csv.writer(self.IMUStorage)


    def fnRun(self):
        """
        Main program that continuously runs.
        Decodes messages from BT signal.

        """

        self.IMU.COBSIntialClear()

        while True:
            data = []
            c = self.IMU.sock.recv(1)
            if c == b'':
                self.IMU.onDisconnect()
                break
            while c != b'\x00' and c != b'':
                data.append(c)
                c = self.IMU.sock.recv(1)
            data = b''.join(data)
            try:
                IMUMessage = self.IMU.decodeCOBS(data)
            except Exception as e:
                print("Failed to decode message.")

            self.fnReceiveData(IMUMessage)

        self.IMU.sock.close()

    def fnReceiveData(self, msg):
        """
        Unpacks data doming from Teensy and stores stores data into CSV.
        """

        dataSizeArray = msg[:4]
        dataSize = struct.unpack("<L", dataSizeArray)[0]
        print(len(msg))
        print(dataSize)
        data = msg[4:]
        imuMsgBT = imuMsg.IMUInfo()
        imuMsgBT.ParseFromString(data)

        print("Value In onRecieveData: %f" % imuMsgBT.acc_x)
        print("Value: %f" % imuMsgBT.acc_y)
        print("Data from sensor " + imuMsgBT.sensorID)

        self.fnStoreData(imuMsgBT.acc_x, imuMsgBT.acc_y, imuMsgBT.acc_z, imuMsgBT.angular_x, imuMsgBT.angular_y, imuMsgBT.angular_z)

    def fnStoreData(self, xAcc, yAcc, zAcc, xGyro, yGyro, zGyro):
        """
        Stores data into CSV storage.
        """

        self.IMUCSV.writerow([xAcc, yAcc, zAcc, xGyro, yGyro, zGyro])
        self.xData.append(xAcc)
        self.yData.append(yAcc)
        self.zData.append(zAcc)
        self.xGyro.append(xGyro)
        self.yGyro.append(yGyro)
        self.zGyro.append(zGyro)


class ClPAWDataStream:
    """
    Connect to HC05 Bluetooth module.
    """

    def __init__(self):
        """
        Creates data parsing instances.
        Feeds data parsing instances to data client.
        """
        self.IMUInstance = ClWheelDataParsing()
        self.dataClientRight = ClDataTransferClient(HC05RightAddress)
        self.dataClientLeft = ClDataTransferClient(HC05LeftAddress)

    def fnRun(self):

        while True:
            self.dataClientRight.fnRun()
            self.dataClientLeft.fnRun()
            self.fnDisplayData("right")
            self.fnDisplayData("left")
            time.sleep(0.5)

    def fnDisplayData(self,  sensorType):

        if sensorType == "right":
            self.rightHC05PlotXAxis(self.IMUInstance.xRightData, self.IMUInstance.yRightData, self.IMUInstance.zRightData,
                                    self.IMUInstance.xRightGyro, self.IMUInstance.yRightGyro, self.IMUInstance.zRightGyro,'r-')

        elif sensorType == "left":
            self.leftHC05PlotXAxis(self.IMUInstance.xLeftData, self.IMUInstance.yLeftData, self.IMUInstance.zLeftData,
                                   self.IMUInstance.xLeftGyro, self.IMUInstance.yLeftGyro, self.IMUInstance.zLeftGyro,'r-')

class ClBluetoothConnect:

    def __init__(self, BTAddress):

        self.cobsmsg = ''
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

        print ("%s: Began connection" % datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

        self.sock.connect((BTAddress, 1))

        print ("%s: Established connection" % datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

    def fnRetieveIMUMessage(self):

        data = []
        c = self.sock.recv(1)
        if c == b'':
            self.IMU.onDisconnect()
            break
        while c != b'\x00' and c != b'':
            data.append(c)
            c = self.IMU.sock.recv(1)
        data = b''.join(data)
        try:
            IMUMessage = self.IMU.decodeCOBS(data)
        except Exception as e:
            print("Failed to decode message (SocketHandler)")

    def onDisconnect(self):
        print("Disconnected from server")
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
        return ''.join(chunks)


if __name__ == "__main__":
    loop = ClWheelDataParsing()
    name = input('What is your name? \n')
    pass