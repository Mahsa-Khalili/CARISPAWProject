#ifndef IMUMSG_h
#define IMUMSG_h
#include <carisPAWBuffers.pb.h>
#include <string.h>

class pbSensorMsgGenerator
{
	
	public:
		pbSensorMsgGenerator();
		void addIMUData(float time, float x_accel, float y_accel, float z_accel, float x_gyro, float y_gyro, float z_gyro);
		//void addSensorID(string sensorID);
		//Generates a proto buffer message in the buffer and returns the length on the resulting message.
		int generatePBMessage();	
		uint8_t* getPBMessage();

	private:
		uint8_t sensorMsgBuffer[2000];
		wheelUnit _msg;

};


#endif
