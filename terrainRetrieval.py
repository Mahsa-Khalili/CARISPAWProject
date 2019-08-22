"""
Author:         Kevin Ta
Date:           2019 May 24th
Purpose:        This Python code utilizes various custom data collection libraries to interface with the Wheelchair
                data acquisition hardware. These include connections to the phones, the Teensy Wheel Modules, and the
                Raspberry Pi Frame Modules.
"""


# IMPORTED LIBRARIES

import os
import datetime, time
import sys

import numpy as np
import pandas as pd

import socket

# DEFINITIONS

dir_path = os.path.dirname(os.path.realpath(__file__))  # Current file directory
HOST = ''
PORT = 65432

# CLASSES

class ClTerrainRetrieval:
    """
    Class for running wheel module data acquisition (ClWheelDataParsing) and real-time display (ClDisplayDataQT).
    """

    def __init__(self):
        """
        Purpose:
        Passed:
        """

        self.TCPSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.TCPSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        print ("{}: Began connection".format(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))

        self.TCPSocket.bind((HOST, PORT))
        self.TCPSocket.listen(1)
        self.socket, self.addr = self.TCPSocket.accept()

        print ("{}: Established connection".format(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))

    def fnStart(self):
        """
        Purpose:
        Passed:
        """

        while(True):
            try:
                terrain = self.socket.recv(128).decode()
                print(terrain)
            except Exception as e:
                print(e)
                break

if __name__ == "__main__":
    while True:
        try:
            instTerrainRetrieval = ClTerrainRetrieval()
            instTerrainRetrieval.fnStart()
            instTerrainRetrieval.TCPsocket.close()
            instTerrainRetrieval.socket.close()
        except Exception as e:
            print(e)
            try:
                instTerrainRetrieval.TCPsocket.close()
                instTerrainRetrieval.socket.close()
            except:
                pass
