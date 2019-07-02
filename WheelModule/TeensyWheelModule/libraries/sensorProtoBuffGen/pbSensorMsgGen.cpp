
#include "pbSensorMsgGen.h"
#include <pb.h>
#include <pb_common.h>
#include <pb_decode.h>
#include <pb_encode.h>

pbSensorMsgGenerator::pbSensorMsgGenerator(){

	_msg = wheelUnit_init_zero;
}
void pbSensorMsgGenerator::addIMUData(float time, float x_accel, float y_accel, float z_accel, float x_gyro, float y_gyro, float z_gyro){
	_msg.has_time_stamp = true;
	_msg.time_stamp = time;

	_msg.has_acc_x = true;
	_msg.acc_x = x_accel;
	_msg.has_acc_y = true;
	_msg.acc_y = y_accel;
	_msg.has_acc_z = true;
	_msg.acc_z = z_accel;

	_msg.has_angular_x = true;
	_msg.angular_x = x_gyro;
	_msg.has_angular_y = true;
	_msg.angular_y = y_gyro;
	_msg.has_angular_z = true;
	_msg.angular_z = z_gyro;
};

/*void addSensorID(string sensorID){

	_msg.sensorID = sensorID;

}*/

int pbSensorMsgGenerator::generatePBMessage(){
	
	
	pb_ostream_t stream = pb_ostream_from_buffer(sensorMsgBuffer, sizeof(sensorMsgBuffer));
	// pb_ostream_t stream = pb_ostream_from_buffer(sensorMsgBuffer, sizeof(sensorMsgBuffer));

	bool _status = pb_encode(&stream, wheelUnit_fields, &_msg);  

	if(!_status){
	 return 21;
	}

	int msgLength = stream.bytes_written;

	return msgLength;
};


uint8_t* pbSensorMsgGenerator::getPBMessage(){
	return sensorMsgBuffer;
}