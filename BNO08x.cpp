#include "BNO08x.h"
#include <cmath>

bool BNO085::beginSPI(uint8_t user_CSPin, uint8_t user_WAKPin, uint8_t user_INTPin, uint8_t user_RSTPin)
{
    //TINTIN
}

float BNO085::qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
	float qFloat = fixedPointValue;
	qFloat *= pow(2, qPoint * -1);
	return (qFloat);
}

bool BNO085::receivePacket(void)
{
     //TINTIN
}

bool BNO085::enableRotationVector(uint16_t timeBetweenReports)
{
	bool status = setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports);
	return status;
}

bool BNO085::enableAccelerometer(uint16_t timeBetweenReports)
{
	bool status = setFeatureCommand(SENSOR_REPORTID_ACCELEROMETER, timeBetweenReports);
	return status;
}

bool BNO085::enableLinearAccelerometer(uint16_t timeBetweenReports)
{
	bool status = setFeatureCommand(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports);
	return status;
}

bool BNO085::enableGravity(uint16_t timeBetweenReports)
{
	bool status = setFeatureCommand(SENSOR_REPORTID_GRAVITY, timeBetweenReports);
	return status;
}

bool BNO085::enableGyro(uint16_t timeBetweenReports)
{
	bool status = setFeatureCommand(SENSOR_REPORTID_GYROSCOPE, timeBetweenReports);
	return status;
}

bool BNO085::enableMagnetometer(uint16_t timeBetweenReports)
{
	bool status = setFeatureCommand(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports);
	return status;
}

bool BNO085::enableRawAccelerometer(uint16_t timeBetweenReports)
{
	bool status = setFeatureCommand(SENSOR_REPORTID_RAW_ACCELEROMETER, timeBetweenReports);
	return status;
}

bool BNO085::enableRawGyro(uint16_t timeBetweenReports)
{
	bool status = setFeatureCommand(SENSOR_REPORTID_RAW_GYROSCOPE, timeBetweenReports);
	return status;
}

bool BNO085::enableRawMagnetometer(uint16_t timeBetweenReports)
{
	bool status = setFeatureCommand(SENSOR_REPORTID_RAW_MAGNETOMETER, timeBetweenReports);
	return status;
}

bool BNO085::enableGyroIntegratedRotationVector(uint16_t timeBetweenReports)
{
	bool status = setFeatureCommand(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, timeBetweenReports);
	return status;
}

bool BNO085::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports)
{
	bool status = setFeatureCommand(reportID, timeBetweenReports, 0); //No specific config
	return status;
}

bool BNO085::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig)
{
	long microsBetweenReports = (long)timeBetweenReports * 1000L;

	shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;	 //Set feature command. Reference page 55
	shtpData[1] = reportID;							   //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
	shtpData[2] = 0;								   //Feature flags
	shtpData[3] = 0;								   //Change sensitivity (LSB)
	shtpData[4] = 0;								   //Change sensitivity (MSB)
	shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  //Report interval (LSB) in microseconds. 0x7A120 = 500ms
	shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  //Report interval
	shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
	shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
	shtpData[9] = 0;								   //Batch Interval (LSB)
	shtpData[10] = 0;								   //Batch Interval
	shtpData[11] = 0;								   //Batch Interval
	shtpData[12] = 0;								   //Batch Interval (MSB)
	shtpData[13] = (specificConfig >> 0) & 0xFF;	   //Sensor-specific config (LSB)
	shtpData[14] = (specificConfig >> 8) & 0xFF;	   //Sensor-specific config
	shtpData[15] = (specificConfig >> 16) & 0xFF;	  //Sensor-specific config
	shtpData[16] = (specificConfig >> 24) & 0xFF;	  //Sensor-specific config (MSB)

	//Transmit packet on channel 2, 17 bytes
	bool status = sendPacket(CHANNEL_CONTROL, 17);
	return status;
}




