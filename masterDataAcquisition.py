"""
Author:         Kevin Ta
Date:           2019 May 24th
Purpose:        This Python code imports
"""


# IMPORTED LIBRARIES

import os
import datetime
import threading
import time
import sys

import numpy as np
import pyqtgraph as pg

from pyqtgraph.Qt import QtGui
from PyQt5 import QtCore

# DEFINITIONS

dir_path = os.path.dirname(os.path.realpath(__file__))  # Current file directory

PI_HOST = ''           # Accept all connections from Pi
PI_PORT = 65432        # Port to listen on from Pi (non-privileged ports are > 1023)

PHONE_HOST = ''        # Accept all connections from Phone

# CUSTOM LIBRARIES

sys.path.insert(0, os.path.join(dir_path, 'WheelModule'))
sys.path.insert(0, os.path.join(dir_path, 'FrameModule'))
sys.path.insert(0, os.path.join(dir_path, 'PhoneVerification'))

from WheelDAQLib import ClWheelDataParsing
from FrameDAQLib import ClFrameDataParsing
from PhoneDAQLib import ClPhoneDataParsing

# DICTIONARIES

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

RaspberryPi = {'Name': 'Pi', 'Address': 'B8:27:EB:6B:15:7F',
               'AccPath6050': os.path.join('IMU Data', '{} Frame6050Acc.csv'.format(
                   datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
               'GyroPath6050': os.path.join('IMU Data', '{} Frame6050Gyro.csv'.format(
                   datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
               'AccPath9250': os.path.join('IMU Data', '{} Frame9250Acc.csv'.format(
                   datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
               'GyroPath9250': os.path.join('IMU Data', '{} Frame9250Gyro.csv'.format(
                   datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
               'DisplayData': np.zeros((13, 1000))} # 9250 then 6050

LeftPhone = {'Name': 'Left Phone', 'Port': 5555,
         'AccPath': os.path.join('IMU Data', '{} LeftphoneAcc.csv'.format(
             datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
         'GyroPath': os.path.join('IMU Data', '{} LeftphoneGyro.csv'.format(
             datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
         'DisplayData': np.zeros((7, 1000))}

RightPhone = {'Name': 'Right Phone', 'Port': 6666,
         'AccPath': os.path.join('IMU Data', '{} RightPhoneAcc.csv'.format(
             datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
         'GyroPath': os.path.join('IMU Data', '{} RightPhoneGyro.csv'.format(
             datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
         'DisplayData': np.zeros((7, 1000))}

# Dictionary associating measurement descriptions to array space
IMUDataDict = {'X Acceleration (G)': 1, 'Y Acceleration (G)': 2, 'Z Acceleration (G)': 3,
               'X Angular Velocity (rad/s)': 4, 'Y Angular Velocity (rad/s)': 5, 'Z Angular Velocity (rad/s)': 6}

# Default Tableau color set for plotting
PenColors = [(31, 119, 180), (255, 127, 14), (44, 160, 44), (214, 39, 40), (148, 103, 189), (140, 66, 75),
             (227, 119, 194), (127, 127, 127), (188, 189, 34), (23, 190, 207)]

# CLASSES

class ClUIWrapper():
    """
    Class for running wheel module data acquisition (ClWheelDataParsing) and real-time display (ClDisplayDataQT).
    """

    def __init__(self, sources):
        """
        Purpose:    Initialize class with sub-class structures and initial variables. Creates a parsing class
                    for every passed data source.
        Passed:     Sources of data (Left Wheel, Right Wheel, Raspberry Pi, Left Phone, Right Phone)
        """

        self.sources = sources  # Make globally set source dictionaries available to class
        self.instDAQLoop = {}  # Initialize dictionary containing data acquisition

        # Initialize every passed data module
        for dataSource in self.sources:
            if dataSource['Name'] in ['Left', 'Right']:
                self.instDAQLoop[dataSource['Name']] = ClWheelDataParsing(dataSource)
            if dataSource['Name'] in ['Pi']:
                self.instDAQLoop[dataSource['Name']] = ClFrameDataParsing(dataSource)
            if dataSource['Name'] in ['Left Phone', 'Right Phone']:
                self.instDAQLoop[dataSource['Name']] = ClPhoneDataParsing(dataSource, dataSource['Port'])
            dataSource['DisplayData'][0,:] = time.time()

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
            threads[dataSource['Name']] = threading.Thread(target=self.instDAQLoop[dataSource['Name']].fnRun)
            threads[dataSource['Name']].start()

        self.app.exec_()  # Executes QT display update code until window is closed, necessary for code to run

        # Stores data in IMU Data folder, accelerometer and angular velocity stored in separate files
        for dataSource in self.sources:
            self.instDAQLoop[dataSource['Name']].fnSaveData(dataSource)


class ClDisplayDataQT:
    """
    Class for displaying IMU data using QT interface.
    """

    def __init__(self, sources):
        """
        Purpose:    Initialize QT window and subplots with axes and titles.
                    Store source data based on which sources were passed.
        Passsed:    Sources containing information on file storage path and stored value arrays.
        """

        self.sources = sources

        self.win = pg.GraphicsWindow(title="Received Signal(s)")  # creates a window
        self.win.resize(1200, 400)# * len(sources)) # Resize window based on number of sources
        self.plot = {} # Create dictionary for subplots
        self.plotData = {} # Create dictionary for subplot data

        for item in ['X Acceleration (G)', 'Y Acceleration (G)', 'Z Angular Velocity (rad/s)']:
            self.plot[item] = self.win.addPlot(title="{}".format(item), clipToView=True)

        # Cycle through each data source and set-up plotting information
        for i, dataSource in enumerate(self.sources):

            dataName = dataSource['Name']

            self.plotData[dataName] = {}
            self.plot[dataName] = {}

            for item in ['X Acceleration (G)', 'Y Acceleration (G)', 'Z Angular Velocity (rad/s)']:
                self.plotData[dataName][item] = self.plot[item].plot(pen=PenColors[i])

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
            for item in ['X Acceleration (G)', 'Y Acceleration (G)', 'Z Angular Velocity (rad/s)']:
                self.plotData[dataSource['Name']][item].setData(dataSource['DisplayData'][0, :],
                                                                dataSource['DisplayData'][IMUDataDict[item], :])


if __name__ == "__main__":

    status = 'Active'
    sources = []

    print('Please input the letter corresponding to which sources you would like to include: ')
    print('l - Left Wheel Module')
    print('r - Right Wheel Module')
    print('f - Frame Module')
    print('p - Phone')
    print('None - End input or enter defaults')

    # while status == 'Active':
    #     source = input('Input: ')
    #
    #     if source == 'l' and Left not in sources:
    #         sources.append(Left)
    #         print('Sources: {}'.format([source['Name'] for source in sources]))
    #     elif source == 'r' and Right not in sources:
    #         sources.append(Right)
    #         print('Sources: {}'.format([source['Name'] for source in sources]))
    #     elif source == 'f' and RaspberryPi not in sources:
    #         sources.append(RaspberryPi)
    #         print('Sources: {}'.format([source['Name'] for source in sources]))
    #     elif source == 'p' and Phone not in sources:
    #         sources.append(Phone)
    #         print('Sources: {}'.format([source['Name'] for source in sources]))
    #     elif not sources:
    #         status = 'Inactive'
    #         sources = [Phone]
    #         print('Sources: {}'.format([source['Name'] for source in sources]))
    #     else:
    #         status = 'Inactive'
    #         print('Sources: {}'.format([source['Name'] for source in sources]))

    sources = [Left, LeftPhone]

    instUIWrapper = ClUIWrapper(sources)
    instUIWrapper.fnStart()
