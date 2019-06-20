"""
Author:         Kevin Ta
Date:           2019 May 24th
Purpose:        This Python code utilizes various custom data collection libraries to interface with the Wheelchair
                data acquisition hardware. These include connections to the phones, the Teensy Wheel Modules, and the
                Raspberry Pi Frame Modules.
"""


# IMPORTED LIBRARIES

import os
import datetime
import sys

import numpy as np
import pyqtgraph as pg

from pyqtgraph.Qt import QtGui
from PyQt5 import QtCore
from multiprocessing import Process, Queue
from scipy.signal import butter, lfilter

# DEFINITIONS

dir_path = os.path.dirname(os.path.realpath(__file__))  # Current file directory

# CUSTOM LIBRARIES

# Include subdirectories for libraries
# TODO: Reorganize file structure to have all libraries in shared folder
sys.path.insert(0, os.path.join(dir_path, 'WheelModule'))
sys.path.insert(0, os.path.join(dir_path, 'FrameModule'))
sys.path.insert(0, os.path.join(dir_path, 'PhoneVerification'))

from WheelDAQLib import ClWheelDataParsing
from FrameDAQLib import ClFrameDataParsing
from PhoneDAQLib import ClPhoneDataParsing

# DICTIONARIES

# Python dictionaries storing name of data source, bluetooth address, data storage path, and the recorded data
Left = {'Name': 'Left', 'Address': '98:D3:51:FD:AD:F5', 'Placement': 'Left', 'Device': 'Module',
        'AccPath': os.path.join('IMU Data', '{} leftAcc.csv'.format(
            datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
        'GyroPath': os.path.join('IMU Data', '{} leftGyro.csv'.format(
            datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
        'Path': '',
        'DisplayData': np.zeros((7, 1000)),
        'Queue': Queue(),
        'RunMarker': Queue()
        }

Right = {'Name': 'Right', 'Address': '98:D3:81:FD:48:C9', 'Placement': 'Right', 'Device': 'Module',
         'AccPath': os.path.join('IMU Data', '{} rightAcc.csv'.format(
             datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
         'GyroPath': os.path.join('IMU Data', '{} rightGyro.csv'.format(
             datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
         'Path': '',
         'DisplayData': np.zeros((7, 1000)),
         'Queue': Queue(),
         'RunMarker': Queue()
         }

RaspberryPi = {'Name': 'Frame', 'Address': 'B8:27:EB:A3:ED:6F', 'Host': '', 'Port': 65432, 'Placement': 'Middle', 'Device': 'Module',
               'AccPath6050': os.path.join('IMU Data', '{} Frame6050Acc.csv'.format(
                   datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
               'GyroPath6050': os.path.join('IMU Data', '{} Frame6050Gyro.csv'.format(
                   datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
               'AccPath9250': os.path.join('IMU Data', '{} Frame9250Acc.csv'.format(
                   datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
               'GyroPath9250': os.path.join('IMU Data', '{} Frame9250Gyro.csv'.format(
                   datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
                'Path': '',
               'DisplayData': np.zeros((13, 1000)), # 9250 then 6050
               'Queue': Queue(),
               'RunMarker': Queue()
               }

LeftPhone = {'Name': 'Left Phone', 'Port': 5555, 'Placement': 'Left', 'Device': 'Phone',
            'AccPath': os.path.join('IMU Data', '{} LeftphoneAcc.csv'.format(
                datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
            'GyroPath': os.path.join('IMU Data', '{} LeftphoneGyro.csv'.format(
                datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
            'Path': '',
            'DisplayData': np.zeros((10, 1000)),
            'Queue': Queue(),
            'RunMarker': Queue()
            }

RightPhone = {'Name': 'Right Phone', 'Port': 6666, 'Placement': 'Right', 'Device': 'Phone',
              'AccPath': os.path.join('IMU Data', '{} RightPhoneAcc.csv'.format(
                  datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
              'GyroPath': os.path.join('IMU Data', '{} RightPhoneGyro.csv'.format(
                  datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
              'Path': '',
              'DisplayData': np.zeros((10, 1000)),
              'Queue': Queue(),
              'RunMarker': Queue()
              }

FramePhone = {'Name': 'Frame Phone', 'Port': 7777,'Placement': 'Middle', 'Device': 'Phone',
              'AccPath': os.path.join('IMU Data', '{} RightPhoneAcc.csv'.format(
                  datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
              'GyroPath': os.path.join('IMU Data', '{} RightPhoneGyro.csv'.format(
                  datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S"))),
               'Path': '',
              'DisplayData': np.zeros((10, 1000)),
              'Queue': Queue(),
              'RunMarker': Queue()
              }

# Dictionary associating measurement descriptions to array space
IMUDataDict = {'X Acceleration (m/s^2)': 1, 'Y Acceleration (m/s^2)': 2, 'Z Acceleration (m/s^2)': 3,
               'X Angular Velocity (rad/s)': 4, 'Y Angular Velocity (rad/s)': 5, 'Z Angular Velocity (rad/s)': 6,
               'heading (deg) C': 7, 'pitch (deg) C': 8, 'roll (deg) C': 9,
               'heading (deg)': 10, 'pitch (deg)': 11, 'roll (deg)': 12}

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
        Passed:     Sources of data (Left Wheel, Right Wheel, Raspberry Pi, Left Phone, Right Phone, Frame Phone)
        """

        self.sources = sources  # Make globally set source dictionaries available to class
        self.instDAQLoop = {}  # Initialize dictionary containing data acquisition

        # Initialize every passed data module
        for dataSource in self.sources:
            if dataSource['Name'] in ['Left', 'Right']:
                self.instDAQLoop[dataSource['Name']] = ClWheelDataParsing(dataSource)
            if dataSource['Name'] in ['Frame']:
                self.instDAQLoop[dataSource['Name']] = ClFrameDataParsing(dataSource, protocol = 'TCP')
            if dataSource['Name'] in ['Left Phone', 'Right Phone', 'Frame Phone']:
                self.instDAQLoop[dataSource['Name']] = ClPhoneDataParsing(dataSource, dataSource['Port'])

        self.app = QtGui.QApplication([])  # Initialize QT GUI, must only be called once

        self.canvas = ClDisplayDataQT(self.sources) # Initialize QT display class

    def fnStart(self):
        """
        Purpose:    Runs each data acquisition loop in a separate thread.
                    Runs QT update display.
                    Dumps data in csv file when complete.
        Passed:     None
        """

        processes = {}

        # Creates and starts each module in a separate process
        for dataSource in self.sources:
            processes[dataSource['Name']] = Process(target=self.instDAQLoop[dataSource['Name']].fnRun)
            processes[dataSource['Name']].start()

        self.app.exec_()  # Executes QT display update code until window is closed, necessary for code to run

        # Send end signal to process and wait for processes to join
        for dataSource in self.sources:
            dataSource['RunMarker'].put(False)
        for dataSource in self.sources:
            processes[dataSource['Name']].join()

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
        self.win.resize(1200, 400)# Sets window size TODO: Look into dynamic resizing
        self.plot = {} # Create dictionary for subplots
        self.plotData = {} # Create dictionary for subplot data

        # Set which parameters you want to plot, 3 is a good number for the window size
        # self.graphSet = ['heading (deg)', 'pitch (deg)', 'roll (deg)']
        # self.graphSet = ['heading (deg) C', 'pitch (deg) C', 'roll (deg) C']
        # self.graphSet = ['X Angular Velocity (rad/s)', 'Y Angular Velocity (rad/s)', 'Z Angular Velocity (rad/s)']
        self.graphSet = ['X Acceleration (m/s^2)', 'Y Acceleration (m/s^2)', 'Z Acceleration (m/s^2)']
        # Create plots
        for item in self.graphSet:
            self.plot[item] = self.win.addPlot(title="{}".format(item), clipToView=True)

        # Cycle through each data source and set-up plotting information
        for i, dataSource in enumerate(self.sources):

            dataName = dataSource['Name']

            # Create nested dictionaries containing individual acceleration data
            self.plotData[dataName] = {}
            self.plot[dataName] = {}

            # Initialize each source of data with specific Tableau color set
            for item in self.graphSet:
                self.plotData[dataName][item] = self.plot[item].plot(pen=PenColors[i])
                # self.plotData[dataName][item + ' C'] = self.plot[item].plot(pen=PenColors[i + 1])

            # Create new row for each source
            self.win.nextRow()



        # Set update period for display, lowering setInterval requires more processing and may lead to more issues
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

        # Cycles through source queues and update plots
        # TODO: Check efficiency of roll and if indexing would be more efficient
        for dataSource in self.sources:
            qSize = dataSource['Queue'].qsize()
            for i in range(qSize):
                buffer = dataSource['Queue'].get()
                dataSource['DisplayData'][:, :] = np.roll(dataSource['DisplayData'], -1)
                dataSource['DisplayData'][:, -1] = buffer
            # if (len(dataSource['DisplayData'])) > 7:
            #     dataSource['DisplayData'][7, -qSize:0] = butter_lowpass_filter(dataSource['DisplayData'][7, -qSize:0], 0.5, 200)
            #     dataSource['DisplayData'][8, -qSize:0] = butter_lowpass_filter(dataSource['DisplayData'][8, -qSize:0], 0.5, 200)
            #     dataSource['DisplayData'][9, -qSize:0] = butter_lowpass_filter(dataSource['DisplayData'][9, -qSize:0], 0.5, 200)

            for item in self.graphSet:
                self.plotData[dataSource['Name']][item].setData(dataSource['DisplayData'][0],
                                                                dataSource['DisplayData'][IMUDataDict[item]])
                # self.plotData[dataSource['Name']][item + ' C'].setData(dataSource['DisplayData'][0],
                #                                                 dataSource['DisplayData'][IMUDataDict[item + ' C']])


if __name__ == "__main__":

    status = 'Active'
    sources = []

    # Prompts user for inputs
    print('Please input the letter corresponding to which sources you would like to include: ')
    print('l - Left Wheel Module')
    print('r - Right Wheel Module')
    print('f - Frame Module')
    print('L -  Left Phone')
    print('M -  Middle Phone')
    print('R -  Right Phone')
    print('None / Wrong - End input or enter defaults')

    while status == 'Active':
        source = input('Input: ')

        if source == 'l' and Left not in sources:
            sources.append(Left)
            print('Sources: {}'.format([source['Name'] for source in sources]))
        elif source == 'r' and Right not in sources:
            sources.append(Right)
            print('Sources: {}'.format([source['Name'] for source in sources]))
        elif source == 'f' and RaspberryPi not in sources:
            sources.append(RaspberryPi)
            print('Sources: {}'.format([source['Name'] for source in sources]))
        elif source == 'L' and LeftPhone not in sources:
            sources.append(LeftPhone)
            print('Sources: {}'.format([source['Name'] for source in sources]))
        elif source == 'M' and FramePhone not in sources:
            sources.append(FramePhone)
            print('Sources: {}'.format([source['Name'] for source in sources]))
        elif source == 'R' and RightPhone not in sources:
            sources.append(RightPhone)
            print('Sources: {}'.format([source['Name'] for source in sources]))
        elif not sources:
            status = 'Inactive'
            sources = [Left, Right, RaspberryPi]
            print('Sources: {}'.format([source['Name'] for source in sources]))
        else:
            status = 'Inactive'
            print('Sources: {}'.format([source['Name'] for source in sources]))

    # Create custom paths for different terrain tests
    terrain = input('Input terrain type: ')

    if not terrain:
        terrain = datetime.datetime.now().strftime("%Y-%m-%d %H.%M.%S")

    print('Terrain: {}'.format(terrain))

    # Stores terrain path name for data storage
    for dataSource in sources:
        dataSource['Path'] = os.path.join('IMU Data', '{}_{}_{}.csv'.format(dataSource['Placement'], terrain, dataSource['Device']))

    # Begin data collection
    instUIWrapper = ClUIWrapper(sources)
    instUIWrapper.fnStart()
