#include "BNO08x.h"
#include <cmath>

bool BNO085::beginSPI(uint8_t user_CSPin, uint8_t user_WAKPin, uint8_t user_INTPin, uint8_t user_RSTPin)
{
    // ADD SPI 
}

void BNO085::softReset(void)
{
	shtpData[0] = 1; 
	sendPacket(CHANNEL_EXECUTABLE, 1); 
	HAL_Delay(50);
	while (receivePacket() == true)
		; 
	HAL_Delay(50);
	while (receivePacket() == true)
		; 
}

float BNO085::qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
	float qFloat = fixedPointValue;
	qFloat *= pow(2, qPoint * -1);
	return (qFloat);
}

bool BNO085::receivePacket(void)
{
    // ADD SPI
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

bool BNO085::dataAvailable(void)
{
	return (getReadings() != 0);
}

uint16_t BNO085::getReadings(void)
{
    if (receivePacket() == true)
	{
		
		if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
		{
			return parseInputReport(); 
		}
		else if (shtpHeader[2] == CHANNEL_CONTROL)
		{
			return parseCommandReport(); 
		}
    else if(shtpHeader[2] == CHANNEL_GYRO)
    {
      return parseInputReport(); 
    }
	}
	return 0;
}

uint16_t BNO085::parseInputReport(void)
{
	int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
	dataLength &= ~(1 << 15); 
	dataLength -= 4; 
	timeStamp = ((uint32_t)shtpData[4] << (8 * 3)) | ((uint32_t)shtpData[3] << (8 * 2)) | ((uint32_t)shtpData[2] << (8 * 1)) | ((uint32_t)shtpData[1] << (8 * 0));
	if(shtpHeader[2] == CHANNEL_GYRO) {
		rawQuatI = (uint16_t)shtpData[1] << 8 | shtpData[0];
		rawQuatJ = (uint16_t)shtpData[3] << 8 | shtpData[2];
		rawQuatK = (uint16_t)shtpData[5] << 8 | shtpData[4];
		rawQuatReal = (uint16_t)shtpData[7] << 8 | shtpData[6];
		rawFastGyroX = (uint16_t)shtpData[9] << 8 | shtpData[8];
		rawFastGyroY = (uint16_t)shtpData[11] << 8 | shtpData[10];
		rawFastGyroZ = (uint16_t)shtpData[13] << 8 | shtpData[12];

		return SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR;
	}

	uint8_t status = shtpData[5 + 2] & 0x03;
	uint16_t data1 = (uint16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
	uint16_t data2 = (uint16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
	uint16_t data3 = (uint16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];
	uint16_t data4 = 0;
	uint16_t data5 = 0; 

	if (dataLength - 5 > 9)
	{
		data4 = (uint16_t)shtpData[5 + 11] << 8 | shtpData[5 + 10];
	}
	if (dataLength - 5 > 11)
	{
		data5 = (uint16_t)shtpData[5 + 13] << 8 | shtpData[5 + 12];
	}

	if (shtpData[5] == SENSOR_REPORTID_ACCELEROMETER)
	{
		accelAccuracy = status;
		rawAccelX = data1;
		rawAccelY = data2;
		rawAccelZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_LINEAR_ACCELERATION)
	{
		accelLinAccuracy = status;
		rawLinAccelX = data1;
		rawLinAccelY = data2;
		rawLinAccelZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_GYROSCOPE)
	{
		gyroAccuracy = status;
		rawGyroX = data1;
		rawGyroY = data2;
		rawGyroZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_MAGNETIC_FIELD)
	{
		magAccuracy = status;
		rawMagX = data1;
		rawMagY = data2;
		rawMagZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR ||
		shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR ||
		shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR ||
		shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR)
	{
		quatAccuracy = status;
		rawQuatI = data1;
		rawQuatJ = data2;
		rawQuatK = data3;
		rawQuatReal = data4;
		rawQuatRadianAccuracy = data5;
	}
	else if (shtpData[5] == SENSOR_REPORTID_TAP_DETECTOR)
	{
		tapDetector = shtpData[5 + 4]; 
	}
	else if (shtpData[5] == SENSOR_REPORTID_STEP_COUNTER)
	{
		stepCount = data3; 
	}
	else if (shtpData[5] == SENSOR_REPORTID_STABILITY_CLASSIFIER)
	{
		stabilityClassifier = shtpData[5 + 4]; 
	}
	else if (shtpData[5] == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER)
	{
		activityClassifier = shtpData[5 + 5]; 

	
		for (uint8_t x = 0; x < 9; x++)					   
			_activityConfidences[x] = shtpData[5 + 6 + x]; 
	}
	else if (shtpData[5] == SENSOR_REPORTID_RAW_ACCELEROMETER)
	{
		memsRawAccelX = data1;
		memsRawAccelY = data2;
		memsRawAccelZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_RAW_GYROSCOPE)
	{
		memsRawGyroX = data1;
		memsRawGyroY = data2;
		memsRawGyroZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_RAW_MAGNETOMETER)
	{
		memsRawMagX = data1;
		memsRawMagY = data2;
		memsRawMagZ = data3;
	}
	else if (shtpData[5] == SHTP_REPORT_COMMAND_RESPONSE)  
	{
		
		uint8_t command = shtpData[5 + 2]; 

		if (command == COMMAND_ME_CALIBRATE)
		{
			calibrationStatus = shtpData[5 + 5]; 
		}
	}
	else if(shtpData[5] == SENSOR_REPORTID_GRAVITY)
	{
		gravityAccuracy = status;
		gravityX = data1;
		gravityY = data2;
		gravityZ = data3;
	}
	else
	{
		
		return 0;
	}

	return shtpData[5];
}

uint16_t BNO085::parseCommandReport(void)
{
	if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
	{
		
		uint8_t command = shtpData[2]; 

		if (command == COMMAND_ME_CALIBRATE)
		{
			calibrationStatus = shtpData[5 + 0]; 
		}
		return shtpData[0];
	}
	else
	{
		
	}

	return 0;
}

void BNO085::getQuat(float &i, float &j, float &k, float &real, float &radAccuracy, uint8_t &accuracy)
{
	i = qToFloat(rawQuatI, rotationVector_Q1);
	j = qToFloat(rawQuatJ, rotationVector_Q1);
	k = qToFloat(rawQuatK, rotationVector_Q1);
	real = qToFloat(rawQuatReal, rotationVector_Q1);
	radAccuracy = qToFloat(rawQuatRadianAccuracy, rotationVector_Q1);
	accuracy = quatAccuracy;
}

float BNO085::getQuatI()
{
	float quat = qToFloat(rawQuatI, rotationVector_Q1);
	return (quat);
}

float BNO085::getQuatJ()
{
	float quat = qToFloat(rawQuatJ, rotationVector_Q1);
	return (quat);
}

float BNO085::getQuatK()
{
	float quat = qToFloat(rawQuatK, rotationVector_Q1);
	return (quat);
}

float BNO085::getQuatReal()
{
	float quat = qToFloat(rawQuatReal, rotationVector_Q1);
	return (quat);
}

float BNO085::getQuatRadianAccuracy()
{
	float quat = qToFloat(rawQuatRadianAccuracy, rotationVectorAccuracy_Q1);
	return (quat);
}

uint8_t BNO085::getQuatAccuracy()
{
	return (quatAccuracy);
}

void BNO085::getAccel(float &x, float &y, float &z, uint8_t &accuracy)
{
	x = qToFloat(rawAccelX, accelerometer_Q1);
	y = qToFloat(rawAccelY, accelerometer_Q1);
	z = qToFloat(rawAccelZ, accelerometer_Q1);
	accuracy = accelAccuracy;
}

float BNO085::getAccelX()
{
	float accel = qToFloat(rawAccelX, accelerometer_Q1);
	return (accel);
}

float BNO085::getAccelY()
{
	float accel = qToFloat(rawAccelY, accelerometer_Q1);
	return (accel);
}

float BNO085::getAccelZ()
{
	float accel = qToFloat(rawAccelZ, accelerometer_Q1);
	return (accel);
}

uint8_t BNO085::getAccelAccuracy()
{
	return (accelAccuracy);
}

void BNO085::getLinAccel(float &x, float &y, float &z, uint8_t &accuracy)
{
	x = qToFloat(rawLinAccelX, linear_accelerometer_Q1);
	y = qToFloat(rawLinAccelY, linear_accelerometer_Q1);
	z = qToFloat(rawLinAccelZ, linear_accelerometer_Q1);
	accuracy = accelLinAccuracy;
}


float BNO085::getLinAccelX()
{
	float accel = qToFloat(rawLinAccelX, linear_accelerometer_Q1);
	return (accel);
}


float BNO085::getLinAccelY()
{
	float accel = qToFloat(rawLinAccelY, linear_accelerometer_Q1);
	return (accel);
}


float BNO085::getLinAccelZ()
{
	float accel = qToFloat(rawLinAccelZ, linear_accelerometer_Q1);
	return (accel);
}

uint8_t BNO085::getLinAccelAccuracy()
{
	return (accelLinAccuracy);
}

void BNO085::getGyro(float &x, float &y, float &z, uint8_t &accuracy)
{
	x = qToFloat(rawGyroX, gyro_Q1);
	y = qToFloat(rawGyroY, gyro_Q1);
	z = qToFloat(rawGyroZ, gyro_Q1);
	accuracy = gyroAccuracy;
}


float BNO085::getGyroX()
{
	float gyro = qToFloat(rawGyroX, gyro_Q1);
	return (gyro);
}


float BNO085::getGyroY()
{
	float gyro = qToFloat(rawGyroY, gyro_Q1);
	return (gyro);
}


float BNO085::getGyroZ()
{
	float gyro = qToFloat(rawGyroZ, gyro_Q1);
	return (gyro);
}


uint8_t BNO085::getGyroAccuracy()
{
	return (gyroAccuracy);
}

void BNO085::getFastGyro(float &x, float &y, float &z)
{
	x = qToFloat(rawFastGyroX, angular_velocity_Q1);
	y = qToFloat(rawFastGyroY, angular_velocity_Q1);
	z = qToFloat(rawFastGyroZ, angular_velocity_Q1);
}


float BNO085::getFastGyroX()
{
	float gyro = qToFloat(rawFastGyroX, angular_velocity_Q1);
	return (gyro);
}


float BNO085::getFastGyroY()
{
	float gyro = qToFloat(rawFastGyroY, angular_velocity_Q1);
	return (gyro);
}


float BNO085::getFastGyroZ()
{
	float gyro = qToFloat(rawFastGyroZ, angular_velocity_Q1);
	return (gyro);
}

void BNO085::getMag(float &x, float &y, float &z, uint8_t &accuracy)
{
	x = qToFloat(rawMagX, magnetometer_Q1);
	y = qToFloat(rawMagY, magnetometer_Q1);
	z = qToFloat(rawMagZ, magnetometer_Q1);
	accuracy = magAccuracy;
}

float BNO085::getMagX()
{
	float mag = qToFloat(rawMagX, magnetometer_Q1);
	return (mag);
}

float BNO085::getMagY()
{
	float mag = qToFloat(rawMagY, magnetometer_Q1);
	return (mag);
}


float BNO085::getMagZ()
{
	float mag = qToFloat(rawMagZ, magnetometer_Q1);
	return (mag);
}


uint8_t BNO085::getMagAccuracy()
{
	return (magAccuracy);
}

void BNO085::getGravity(float &x, float &y, float &z, uint8_t &accuracy)
{
	x = qToFloat(gravityX, gravity_Q1);
	y = qToFloat(gravityX, gravity_Q1);
	z = qToFloat(gravityX, gravity_Q1);
	accuracy = gravityAccuracy;
}

float BNO085::getGravityX()
{
	float x = qToFloat(gravityX, gravity_Q1);
	return x;
}


float BNO085::getGravityY()
{
	float y = qToFloat(gravityY, gravity_Q1);
	return y;
}

float BNO085::getGravityZ()
{
	float z = qToFloat(gravityZ, gravity_Q1);
	return z;
}

uint8_t BNO085::getGravityAccuracy()
{
	return (gravityAccuracy);
}

void BNO085::calibrateAccelerometer()
{
	sendCalibrateCommand(CALIBRATE_ACCEL);
}

void BNO085::calibrateGyro()
{
	sendCalibrateCommand(CALIBRATE_GYRO);
}


void BNO085::calibrateMagnetometer()
{
	sendCalibrateCommand(CALIBRATE_MAG);
}


void BNO085::calibratePlanarAccelerometer()
{
	sendCalibrateCommand(CALIBRATE_PLANAR_ACCEL);
}


void BNO085::calibrateAll()
{
	sendCalibrateCommand(CALIBRATE_ACCEL_GYRO_MAG);
}

int16_t BNO085::getRawAccelX()
{
	return (memsRawAccelX);
}

int16_t BNO085::getRawAccelY()
{
	return (memsRawAccelY);
}

int16_t BNO085::getRawAccelZ()
{
	return (memsRawAccelZ);
}

int16_t BNO085::getRawGyroX()
{
	return (memsRawGyroX);
}
int16_t BNO085::getRawGyroY()
{
	return (memsRawGyroY);
}
int16_t BNO085::getRawGyroZ()
{
	return (memsRawGyroZ);
}


int16_t BNO085::getRawMagX()
{
	return (memsRawMagX);
}
int16_t BNO085::getRawMagY()
{
	return (memsRawMagY);
}
int16_t BNO085::getRawMagZ()
{
	return (memsRawMagZ);
}

int16_t BNO085::getQ1(uint16_t recordID)
{
	
	uint16_t q = readFRSword(recordID, 7) & 0xFFFF; 
	return (q);
}


int16_t BNO085::getQ2(uint16_t recordID)
{
	
	uint16_t q = readFRSword(recordID, 7) >> 16; 
	return (q);
}


int16_t BNO085::getQ3(uint16_t recordID)
{
	
	uint16_t q = readFRSword(recordID, 8) >> 16; 
	return (q);
}


float BNO085::getResolution(uint16_t recordID)
{
	
	int16_t Q = getQ1(recordID);

	uint32_t value = readFRSword(recordID, 2); 

	float resolution = qToFloat(value, Q);

	return (resolution);
}


float BNO085::getRange(uint16_t recordID)
{
	
	int16_t Q = getQ1(recordID);

	//Range is always word 1
	uint32_t value = readFRSword(recordID, 1); 

	float range = qToFloat(value, Q);

	return (range);
}


uint32_t BNO085::readFRSword(uint16_t recordID, uint8_t wordNumber)
{
	if (readFRSdata(recordID, wordNumber, 1) == true) 
		return (metaData[0]);						  

	return (0); 
}


void BNO085::frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize)
{
	shtpData[0] = SHTP_REPORT_FRS_READ_REQUEST; 
	shtpData[1] = 0;							
	shtpData[2] = (readOffset >> 0) & 0xFF;		
	shtpData[3] = (readOffset >> 8) & 0xFF;
	shtpData[4] = (recordID >> 0) & 0xFF;		
	shtpData[5] = (recordID >> 8) & 0xFF;	
	shtpData[6] = (blockSize >> 0) & 0xFF;		
	shtpData[7] = (blockSize >> 8) & 0xFF;		

	
	sendPacket(CHANNEL_CONTROL, 8);
}


bool BNO085::readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead)
{
	uint8_t spot = 0;

	
	frsReadRequest(recordID, startLocation, wordsToRead);
	while (1)
	{
	
		while (1)
		{
			uint8_t counter = 0;
			while (receivePacket() == false)
			{
				if (counter++ > 100)
					return (false); //Give up
				HAL_Delay(1);   
			}

			
			if (shtpData[0] == SHTP_REPORT_FRS_READ_RESPONSE)
				if (((((uint16_t)shtpData[13]) << 8) | shtpData[12]) == recordID)
					break;
		}

		uint8_t dataLength = shtpData[1] >> 4;
		uint8_t frsStatus = shtpData[1] & 0x0F;

		uint32_t data0 = (uint32_t)shtpData[7] << 24 | (uint32_t)shtpData[6] << 16 | (uint32_t)shtpData[5] << 8 | (uint32_t)shtpData[4];
		uint32_t data1 = (uint32_t)shtpData[11] << 24 | (uint32_t)shtpData[10] << 16 | (uint32_t)shtpData[9] << 8 | (uint32_t)shtpData[8];


		if (dataLength > 0)
		{
			metaData[spot++] = data0;
		}
		if (dataLength > 1)
		{
			metaData[spot++] = data1;
		}

		if (spot >= MAX_METADATA_SIZE)
		{
			return (true);
		}

		if (frsStatus == 3 || frsStatus == 6 || frsStatus == 7)
		{
			return (true); 
		}
	}
}

void BNO085::endCalibration()
{
	sendCalibrateCommand(CALIBRATE_STOP); 
}

bool BNO085::calibrationComplete()
{
	if (calibrationStatus == 0)
		return (true);
	return (false);
}

void BNO085::saveCalibration()
{
	for (uint8_t x = 3; x < 12; x++) 
		shtpData[x] = 0;
	sendCommand(COMMAND_DCD); 
}

void BNO085::requestCalibrationStatus()
{
	for (uint8_t x = 3; x < 12; x++) 
		shtpData[x] = 0;

	shtpData[6] = 0x01; 
	sendCommand(COMMAND_ME_CALIBRATE);
}

void BNO085::sendCalibrateCommand(uint8_t thingToCalibrate)
{

	for (uint8_t x = 3; x < 12; x++) 
		shtpData[x] = 0;

	if (thingToCalibrate == CALIBRATE_ACCEL)
		shtpData[3] = 1;
	else if (thingToCalibrate == CALIBRATE_GYRO)
		shtpData[4] = 1;
	else if (thingToCalibrate == CALIBRATE_MAG)
		shtpData[5] = 1;
	else if (thingToCalibrate == CALIBRATE_PLANAR_ACCEL)
		shtpData[7] = 1;
	else if (thingToCalibrate == CALIBRATE_ACCEL_GYRO_MAG)
	{
		shtpData[3] = 1;
		shtpData[4] = 1;
		shtpData[5] = 1;
	}
	else if (thingToCalibrate == CALIBRATE_STOP)
		; 
	calibrationStatus = 1;

	
	sendCommand(COMMAND_ME_CALIBRATE);
}

bool BNO085::sendCommand(uint8_t command)
{
	shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; 
	shtpData[1] = commandSequenceNumber++;	 
	shtpData[2] = command;					   
	bool status = sendPacket(CHANNEL_CONTROL, 12);
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

	shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;	 
	shtpData[1] = reportID;							   
	shtpData[2] = 0;								 
	shtpData[3] = 0;								   
	shtpData[4] = 0;								  
	shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  
	shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  
	shtpData[7] = (microsBetweenReports >> 16) & 0xFF; 
	shtpData[8] = (microsBetweenReports >> 24) & 0xFF; 
	shtpData[9] = 0;								   
	shtpData[10] = 0;								  
	shtpData[11] = 0;
	shtpData[12] = 0;								  
	shtpData[13] = (specificConfig >> 0) & 0xFF;	   
	shtpData[14] = (specificConfig >> 8) & 0xFF;	   
	shtpData[15] = (specificConfig >> 16) & 0xFF;	  
	shtpData[16] = (specificConfig >> 24) & 0xFF;	 

	
	bool status = sendPacket(CHANNEL_CONTROL, 17);
	return status;
}

bool BNO085::sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
	uint8_t packetLength = dataLength + 4; //Add four bytes for the header
		//digitalWrite(_cs, LOW);

		//Send the 4 byte packet header
		// _spiPort->transfer(packetLength & 0xFF);			 //Packet length LSB
		// _spiPort->transfer(packetLength >> 8);				 //Packet length MSB
		// _spiPort->transfer(channelNumber);					 //Channel number
		// _spiPort->transfer(sequenceNumber[channelNumber]++); //Send the sequence number, increments with each packet sent, different counter for each channel

		// //Send the user's data packet
		// for (uint8_t i = 0; i < dataLength; i++)
		// {
		// 	_spiPort->transfer(shtpData[i]);
		// }

		// digitalWrite(_cs, HIGH);
		// _spiPort->endTransaction();

	return (true);
}

bool BNO085::waitForSPI()
{
	for (uint8_t counter = 0; counter < 125; counter++) //Don't got more than 255
	{
		// if (digitalRead(_int) == LOW)
		// 	return (true);
		// delay(1);
	}
	return (false);
}






