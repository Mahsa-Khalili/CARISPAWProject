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
from cobs import cobs
from libraries.imumsg import imumsg_pb2 as imuMsg
from libraries import realtimelib
from vispy import gloo
from vispy import app
import math
import threading

# DEFINITIONS

dir_path = os.path.dirname(os.path.realpath(__file__))
rightIMUDataFilePath = os.path.join('IMU Data', '{} rightIMUMessage.csv'.format(datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S")))
leftIMUDataFilePath = os.path.join('IMU Data', '{} leftIMUMessage.csv'.format(datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S")))


# imuMsgPath = os.path.join(dir_path, 'libraries', 'imumsg_pb2.py')
# print(imuMsgPath)
# imuMsg = importlib.import_module('imumsg_pb2', imuMsgPath)

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)
RaspberryPiAddress = "B8:27:EB:6B:15:7F"

displayData = np.zeros((6, 1000)).astype(np.float32)

Left = {'Address': "98:D3:51:FD:AD:F5", "DataPath": leftIMUDataFilePath}
Right = {'Address': "98:D3:81:FD:48:C9", "DataPath": rightIMUDataFilePath}

# CLASSES

class ClUIWrapper():

    def __init__(self, Source):
        self.loop = ClWheelDataParsing(Source)
        self.canvas = Canvas()
        self.source = Source

    def fnStart(self):

        threads = []

        # p2 = threading.Thread(target=app.run)
        # threads.append(p2)
        # p2.start()
        p1 = threading.Thread(target=self.loop.fnRun)
        threads.append(p1)
        p1.start()

        # self.loop.fnRun()
        app.run()

        np.savetxt(self.source['DataPath'], np.transpose([self.loop.xData, self.loop.yData, self.loop.zData, self.loop.xGyro,
                                             self.loop.yGyro, self.loop.zGyro]), delimiter=",")


class Canvas(app.Canvas):
    def __init__(self):
        app.Canvas.__init__(self, title='Use your wheel to zoom!',
                            keys='interactive')
        self.program = gloo.Program(realtimelib.VERT_SHADER, realtimelib.FRAG_SHADER)
        self.program['a_position'] = displayData.reshape(-1, 1)
        self.program['a_color'] = realtimelib.color
        self.program['a_index'] = realtimelib.index
        self.program['u_scale'] = (1., 1.)
        self.program['u_size'] = (realtimelib.nrows, realtimelib.ncols)
        self.program['u_n'] = realtimelib.n

        gloo.set_viewport(0, 0, *self.physical_size)

        self._timer = app.Timer('auto', connect=self.on_timer, start=True)

        gloo.set_state(clear_color='black', blend=True,
                       blend_func=('src_alpha', 'one_minus_src_alpha'))

        self.show()

    def on_resize(self, event):
        gloo.set_viewport(0, 0, *event.physical_size)

    def on_mouse_wheel(self, event):
        dx = np.sign(event.delta[1]) * .05
        scale_x, scale_y = self.program['u_scale']
        scale_x_new, scale_y_new = (scale_x * math.exp(2.5*dx),
                                    scale_y * math.exp(0.0*dx))
        self.program['u_scale'] = (max(1, scale_x_new), max(1, scale_y_new))
        self.update()

    def on_timer(self, event):
        """Add some data at the end of each signal (real-time signals)."""
        k = 10

        print('Running canvas.')

        self.program['a_position'].set_data(displayData.ravel().astype(np.float32))
        self.update()

    def on_draw(self, event):
        gloo.clear()
        self.program.draw('line_strip')


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

        self.IMUStorage = source['DataPath']

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
        # with open(self.IMUStorage, mode='a') as csvfile:
        #     writer = csv.writer(csvfile)
        #     writer.writerow([xAcc, yAcc, zAcc, xGyro, yGyro, zGyro])
        displayData[:,:] = np.roll(displayData, -1)
        displayData[0:6, -1] = [xAcc, yAcc, zAcc, xGyro, yGyro, zGyro]
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
    # loop = ClWheelDataParsing(Left)
    # loop.fnRun()
    UIWrap = ClUIWrapper(Right)
    UIWrap.fnStart()

    name = input('What is your name? \n')
    pass