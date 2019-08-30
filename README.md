# CARISPAWProject

**Author:**			Kevin Ta  
**Date Started:**	2019 May 3rd  
**Github Repository:** https://github.com/kev-in-ta/CARISPAWProject

***

## 1.0 Introduction

Mahsa Khalili's work involves developing smarter control systems for pushrim-activated power-assisted wheelchairs (PAPAWs). 
One aspect of this project involves terrain classification through IMU(inertial measurement unit) sensors (acceleration and angular velocity).
The code in this repository relates to the hardware running real-time data collection and real-time terrain classification.

***
## 2.0 Hardware Set-up

The hardware set-up consists of three separate modules that capture various sensor information. 

These modules are:
1. Left Wheel Module
1. Right Wheel Module
1. Frame Module

### 2.1 Left Wheel Module and Right Wheel Module

The left and right wheel modules are equipt with an MPU6050 6-axis IMU to capture accelerometer and gyroscope data. 
This information is fed into a Teensy 3.6 development board which does some preporcessing in the form of removing offsets.
The data is eventually packaged and sent via the Bluetooth module (HC-05) to some connecting device, which currently might be the laptop or Raspberry Pi.

#### Current MPU6050 Specifications:

**Frequency Cut-off:** 184/188 Hz  
**Accelerometer Sensitivity:** +-4g  
**Gyroscope Sensitivity:** +-500 degrees per second

*Values are set in the \CARISPAWProject\WheelModule\TeensyWheelModule\libraries\MPU6050Library\MPU6050.cpp*  
*This file must be edited on the local Arduino library, the libraries included must be moved to the Arduino library folder*

#### HC-05 Bluetooth Modules

On each wheel module there is an HC-05 bluetooth module. These bluetooth modules have the following parameters:

| Module | Name        | Bluetooth Address | Pin  |
| :----: | :---------: | :---------------: | :--: |
| Left   | HC-05 Left  | 98:D3:51:FD:AD:F5 | 0303 |
| Right  | HC-05       | 98:D3:81:FD:48:C9 | 1234 |

The baudrate is set to 230400 bit/s.

The name, pin, and baudrate can be changed using the AT commands, where command mode can be activated by holding down the button on the HC-05 when first powered. 

#### Information Pipeline

6-axis IMU data is polled at 333.3 Hz (3ms) and packaged into a serialized data structure created using Google's Protocol Buffer (Proto Buffers). 
This message is then encoded using constant overhead byte stuffing (COBS) to remove the x00 bytes, allowing the x00 bytes to be used as delimiters.
The message is then written to Serial1 which is connected to the bluetooth module.

IMU -> Proto Buffer -> COBS -> HC-05

### 2.2 Frame Module

The frame module is currently run off a Raspberry Pi 3B+ connected to a number of sensors.

These sensors include:

1. MPU6050 6-axis IMU
1. MPU9250 9-axis IMU
1. AJ-SR04M (JSN-SR04T-2.0) Ultrasonic Sensor (Down)
1. AJ-SR04M (JSN-SR04T-2.0) Ultrasonic Sensor (Forward)
1. Sony IMX219 - RaspberryPi Camera Module v2

#### IMU Configuration

**Frequency Cut-off:** 184/188 Hz  
**Accelerometer Sensitivity:** +-4g  
**Gyroscope Sensitivity:** +-500 degrees per second  
**Magnetometer:** Sampling at 8 Hz

*Values can be set in \CARISPAWProject\FrameModule\FrameClient\libraries\mpu6050.py or \CARISPAWProject\FrameModule\FrameClient\libraries\mpu9250.py*

#### Raspberry Pi Bluetooth Configuration

The Raspberry Pi comes with built-in bluetooth and wi-fi capabilities. 

| Module | Name          | Bluetooth Address | Pin  |
| :----: | :-----------: | :---------------: | :--: |
| Frame  | Raspberry Pi  | B8:27:EB:A3:ED:6F | None |

#### Information Pipeline

The Raspberry Pi collects data from various sensors, serializing them into procol buffer data structures, and places them into a multiprocessing queue to send to a remote laptop.
The messages are also processed using COBS before sending to utilize x00 as a delimiter. This data packaged data is sent via wi-fi through a tp-link AC750 portable router.

The 6-axis IMU is set to run at 300 Hz.  
The 9-axis IMU is set to run at 100 Hz. (1/3 of 6-axis IMU)  
The proximity sensors are set to run at 37.5 Hz (1/8 of 6-axis IMU)  
The Pi Camera is set to run at 1 Hz. (1/300 of 6-axis IMU)

**Wi-fi Name**: CarisWheelchair5g  
**Wi-fi Password**: (default CARIS x2)   
**Server**: Laptop  
**Client**: Raspberry Pi  
**Port Number**: 65432  
**IP Address**: 192.168.0.100  

***
## 3.0 Code

### 3.1 Laptop Data Acquisition

The laptop data acquisition is wrapped by the masterDataAcqusition.py. 
Here, the program prompts the user to input which modules they will receive data from.

    Please input the letter corresponding to which sources you would like to include: 
    l - Left Wheel Module
    r - Right Wheel Module
    f - Frame Module
    L -  Left Phone
    M -  Middle Phone
    R -  Right Phone
    None / Wrong - End input or enter defaults

It will then prompt you for the terrain. 
If no terrain is specified, the terrain is set to the date time in [YYYY-MM-DD HH.MM.SS] format.
This generates save paths in the form of:  
	
	[placement]_[terrain]_[Device].csv
	middle_concrete_module.csv

The program then will instantiate instances of custom data acquisition libraries located in FrameDAQLib.py and WheelDAQLib.py.
Many of the parameters necessary ot instantiate these classes are located in the header dictionaries.

Once these classes have been instantiated the main data acquisition tool will instnatiate a plotting class which utilizes pyQTgraph.
The program utilizes an updating loop within the pyQT library to retrieve data from a queue and display that data.
The data acquisition instances for each module feed data back through individual multiprocessing queues which is retrieved in the update loop.
When the left and right wheel are instantiated, the loop will also perform sensor fusion to calulate linear and angular velocity,
 referenced as x velocity and z angular velocity in the frame's local reference frame. 
 When the display is terminated (closing the window), the main program sends the runMarker, implemented as a queue, to terminate each of the data acqusition instances.
 
 *Note: Data is stored in memory of each individual data acquisition instance and saves when the display is terminated.*

### 3.2 Frame Data Acquisition

The code located for this is within the /FrameModule/FrameClient/ folder.

The frame data acqusition code use a similar library system and multiprocessing to poll information from its different sensors. 
There are three separate libraries for data aqusition, being IMUSensorLib.py, USSSensorLib.py, and PiCamSensorLib.py.
To activate sensors, you have to set the header variable ACTIVE_SENSORS to include the numbers corresponsing to each sensor.

	IMU-9 - 0
	IMU-6 - 1
	USS-D - 2
	USS-F - 3 (Not implemented as of 2019 August)
	PiCam - 4 (Not implemented as of 2019 August)

The program will then run all active sensors, trasmitting data via wifi to the remote laptop.

### 3.3 Laptop Terrain Retrieval

The laptop terrain retrieval code is currently quite barebones. 
The laptop simply retrieves a string message containing the clasiffication type and the terrain classification.
This is done thorugh a socket connection with the Raspberry Pi.

### 3.4 Frame Terrain Classification

The frame terrain classification is currently only working with the middle frame, although it can be adjusted for individual wheel modules quite easily.
Reusing much of the code for the data acquisition, a similar queue implementation is used.

The machine learning algorithms and methods are the major points for the terrain classification.
The completed / trained classifiers are provided in the form of joblib files, which are more efficient than pickle files for storing Python objects.
Currently, support vector machines and random forests are implemented for time features, frequency features, and log PSDs (power spectrum densities).

	IMU-9 - 0
	IMU-6 - 1
	USS-D - 2
	USS-F - 3 (Not implemented as of 2019 August)
	PiCam - 4 (Not implemented as of 2019 August)
	RiWhl - 5 (Not implemented as of 2019 August)
	LeWhl - 6 (Not implemented as of 2019 August)

*Note that due to quirks of the random forest algorithm, the machine leaning must occur on a 32-bit system*

***

## 4.0 Adjusting Start-up Python Program (Frame)

The Raspberry Pi is set to autorun a service which calls a python script after connecting to wi-fi.
To start or stop this script when you're troublehooting, utilize the following commands:

    sudo systemctl stop dataTransfer
	sudo systemctl start dataTransfer
	
To change the which script runs between the terrain classifier and the data acqusition, you must edit the service.

To do so, go into the console editor with the following command:

    sudo nano /lib/systemd/system/dataTransfer.service
	
and change the script:

	FrameDataTransferClient.py from/to terrainClassifier.py
	
When you reboot the Pi, it will then start whichever progam of your choice.

***

## 5.0 Router Set-up

The router should be set to dynamic in the LAN settings for running frmae module code. 

Currently with the laptop set-up as the server, you will have assign your laptop with a the IP address of 192.168.0.100.
This can be changed in the router settings under DHCP > address reservation.

***

## 6.0 Future Work

### 6.1 High Priority Work
1. Add data collection / storage for terrain classification retrieval.
1. Get left and right wheel modules communicating with the frame module and calculate synthesis data.

### 6.2 Low Priority Work
1. Switch queue structures for value structures for run markers and ensure all sub libraries utilize the run marker.
1. Switch the server and client relationship of the Pi and laptop so any laptop works.
1. Implement forward ultrasonic sensors.
1. Implement pi camera to take photos and feed back using protobuffer structures.
1. Test multiple sensors working conurrently for stability.