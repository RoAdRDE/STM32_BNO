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

bool BNO085::dataAvailable(void)
{
	return (getReadings() != 0);
}

uint16_t BNO085::getReadings(void)
{
    if (receivePacket() == true)
	{
		//Check to see if this packet is a sensor reporting its data to us
		if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
		{
			return parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
		}
		else if (shtpHeader[2] == CHANNEL_CONTROL)
		{
			return parseCommandReport(); //This will update responses to commands, calibrationStatus, etc.
		}
    else if(shtpHeader[2] == CHANNEL_GYRO)
    {
      return parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
    }
	}
	return 0;
}

uint16_t BNO085::parseInputReport(void)
{
	//Calculate the number of data bytes in this packet
	int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
	dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last.
	//Ignore it for now. TODO catch this as an error and exit

	dataLength -= 4; //Remove the header bytes from the data count

	timeStamp = ((uint32_t)shtpData[4] << (8 * 3)) | ((uint32_t)shtpData[3] << (8 * 2)) | ((uint32_t)shtpData[2] << (8 * 1)) | ((uint32_t)shtpData[1] << (8 * 0));

	// The gyro-integrated input reports are sent via the special gyro channel and do no include the usual ID, sequence, and status fields
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

	uint8_t status = shtpData[5 + 2] & 0x03; //Get status bits
	uint16_t data1 = (uint16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
	uint16_t data2 = (uint16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
	uint16_t data3 = (uint16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];
	uint16_t data4 = 0;
	uint16_t data5 = 0; //We would need to change this to uin32_t to capture time stamp value on Raw Accel/Gyro/Mag reports

	if (dataLength - 5 > 9)
	{
		data4 = (uint16_t)shtpData[5 + 11] << 8 | shtpData[5 + 10];
	}
	if (dataLength - 5 > 11)
	{
		data5 = (uint16_t)shtpData[5 + 13] << 8 | shtpData[5 + 12];
	}

	//Store these generic values to their proper global variable
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

		//Only available on rotation vector and ar/vr stabilized rotation vector,
		// not game rot vector and not ar/vr stabilized rotation vector
		rawQuatRadianAccuracy = data5;
	}
	else if (shtpData[5] == SENSOR_REPORTID_TAP_DETECTOR)
	{
		tapDetector = shtpData[5 + 4]; //Byte 4 only
	}
	else if (shtpData[5] == SENSOR_REPORTID_STEP_COUNTER)
	{
		stepCount = data3; //Bytes 8/9
	}
	else if (shtpData[5] == SENSOR_REPORTID_STABILITY_CLASSIFIER)
	{
		stabilityClassifier = shtpData[5 + 4]; //Byte 4 only
	}
	else if (shtpData[5] == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER)
	{
		activityClassifier = shtpData[5 + 5]; //Most likely state

		//Load activity classification confidences into the array
		for (uint8_t x = 0; x < 9; x++)					   //Hardcoded to max of 9. TODO - bring in array size
			_activityConfidences[x] = shtpData[5 + 6 + x]; //5 bytes of timestamp, byte 6 is first confidence byte
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
		//The BNO085 responds with this report to command requests. It's up to use to remember which command we issued.
		uint8_t command = shtpData[5 + 2]; //This is the Command byte of the response

		if (command == COMMAND_ME_CALIBRATE)
		{
			calibrationStatus = shtpData[5 + 5]; //R0 - Status (0 = success, non-zero = fail)
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
		//This sensor report ID is unhandled.
		//See reference manual to add additional feature reports as needed
		return 0;
	}

	//TODO additional feature reports may be strung together. Parse them all.
	return shtpData[5];
}

uint16_t BNO085::parseCommandReport(void)
{
	if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
	{
		//The BNO085 responds with this report to command requests. It's up to use to remember which command we issued.
		uint8_t command = shtpData[2]; //This is the Command byte of the response

		if (command == COMMAND_ME_CALIBRATE)
		{
			calibrationStatus = shtpData[5 + 0]; //R0 - Status (0 = success, non-zero = fail)
		}
		return shtpData[0];
	}
	else
	{
		//This sensor report ID is unhandled.
		//See reference manual to add additional feature reports as needed
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

//Return the acceleration component
float BNO085::getLinAccelX()
{
	float accel = qToFloat(rawLinAccelX, linear_accelerometer_Q1);
	return (accel);
}

//Return the acceleration component
float BNO085::getLinAccelY()
{
	float accel = qToFloat(rawLinAccelY, linear_accelerometer_Q1);
	return (accel);
}

//Return the acceleration component
float BNO085::getLinAccelZ()
{
	float accel = qToFloat(rawLinAccelZ, linear_accelerometer_Q1);
	return (accel);
}

//Return the acceleration component
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

//Return the gyro component
float BNO085::getGyroX()
{
	float gyro = qToFloat(rawGyroX, gyro_Q1);
	return (gyro);
}

//Return the gyro component
float BNO085::getGyroY()
{
	float gyro = qToFloat(rawGyroY, gyro_Q1);
	return (gyro);
}

//Return the gyro component
float BNO085::getGyroZ()
{
	float gyro = qToFloat(rawGyroZ, gyro_Q1);
	return (gyro);
}

//Return the gyro component
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

// Return the high refresh rate gyro component
float BNO085::getFastGyroX()
{
	float gyro = qToFloat(rawFastGyroX, angular_velocity_Q1);
	return (gyro);
}

// Return the high refresh rate gyro component
float BNO085::getFastGyroY()
{
	float gyro = qToFloat(rawFastGyroY, angular_velocity_Q1);
	return (gyro);
}

// Return the high refresh rate gyro component
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

//Return the magnetometer component
float BNO085::getMagX()
{
	float mag = qToFloat(rawMagX, magnetometer_Q1);
	return (mag);
}

//Return the magnetometer component
float BNO085::getMagY()
{
	float mag = qToFloat(rawMagY, magnetometer_Q1);
	return (mag);
}

//Return the magnetometer component
float BNO085::getMagZ()
{
	float mag = qToFloat(rawMagZ, magnetometer_Q1);
	return (mag);
}

//Return the mag component
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

//Return the gravity component
float BNO085::getGravityY()
{
	float y = qToFloat(gravityY, gravity_Q1);
	return y;
}

//Return the gravity component
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

//Sends the commands to begin calibration of the magnetometer
void BNO085::calibrateMagnetometer()
{
	sendCalibrateCommand(CALIBRATE_MAG);
}

//Sends the commands to begin calibration of the planar accelerometer
void BNO085::calibratePlanarAccelerometer()
{
	sendCalibrateCommand(CALIBRATE_PLANAR_ACCEL);
}

//See 2.2 of the Calibration Procedure document 1000-4044
void BNO085::calibrateAll()
{
	sendCalibrateCommand(CALIBRATE_ACCEL_GYRO_MAG);
}

int16_t BNO085::getRawAccelX()
{
	return (memsRawAccelX);
}
//Return raw mems value for the accel
int16_t BNO085::getRawAccelY()
{
	return (memsRawAccelY);
}
//Return raw mems value for the accel
int16_t BNO085::getRawAccelZ()
{
	return (memsRawAccelZ);
}

//Return raw mems value for the gyro
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

//Return raw mems value for the mag
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
	//Q1 is always the lower 16 bits of word 7
	uint16_t q = readFRSword(recordID, 7) & 0xFFFF; //Get word 7, lower 16 bits
	return (q);
}

//Given a record ID, read the Q2 value from the metaData record in the FRS
//Q2 is used in sensor bias
int16_t BNO085::getQ2(uint16_t recordID)
{
	//Q2 is always the upper 16 bits of word 7
	uint16_t q = readFRSword(recordID, 7) >> 16; //Get word 7, upper 16 bits
	return (q);
}

//Given a record ID, read the Q3 value from the metaData record in the FRS
//Q3 is used in sensor change sensitivity
int16_t BNO085::getQ3(uint16_t recordID)
{
	//Q3 is always the upper 16 bits of word 8
	uint16_t q = readFRSword(recordID, 8) >> 16; //Get word 8, upper 16 bits
	return (q);
}

//Given a record ID, read the resolution value from the metaData record in the FRS for a given sensor
float BNO085::getResolution(uint16_t recordID)
{
	//The resolution Q value are 'the same as those used in the sensor's input report'
	//This should be Q1.
	int16_t Q = getQ1(recordID);

	//Resolution is always word 2
	uint32_t value = readFRSword(recordID, 2); //Get word 2

	float resolution = qToFloat(value, Q);

	return (resolution);
}

//Given a record ID, read the range value from the metaData record in the FRS for a given sensor
float BNO085::getRange(uint16_t recordID)
{
	//The resolution Q value are 'the same as those used in the sensor's input report'
	//This should be Q1.
	int16_t Q = getQ1(recordID);

	//Range is always word 1
	uint32_t value = readFRSword(recordID, 1); //Get word 1

	float range = qToFloat(value, Q);

	return (range);
}

//Given a record ID and a word number, look up the word data
//Helpful for pulling out a Q value, range, etc.
//Use readFRSdata for pulling out multi-word objects for a sensor (Vendor data for example)
uint32_t BNO085::readFRSword(uint16_t recordID, uint8_t wordNumber)
{
	if (readFRSdata(recordID, wordNumber, 1) == true) //Get word number, just one word in length from FRS
		return (metaData[0]);						  //Return this one word

	return (0); //Error
}

//Ask the sensor for data from the Flash Record System
//See 6.3.6 page 40, FRS Read Request
void BNO085::frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize)
{
	shtpData[0] = SHTP_REPORT_FRS_READ_REQUEST; //FRS Read Request
	shtpData[1] = 0;							//Reserved
	shtpData[2] = (readOffset >> 0) & 0xFF;		//Read Offset LSB
	shtpData[3] = (readOffset >> 8) & 0xFF;		//Read Offset MSB
	shtpData[4] = (recordID >> 0) & 0xFF;		//FRS Type LSB
	shtpData[5] = (recordID >> 8) & 0xFF;		//FRS Type MSB
	shtpData[6] = (blockSize >> 0) & 0xFF;		//Block size LSB
	shtpData[7] = (blockSize >> 8) & 0xFF;		//Block size MSB

	//Transmit packet on channel 2, 8 bytes
	sendPacket(CHANNEL_CONTROL, 8);
}

//Given a sensor or record ID, and a given start/stop bytes, read the data from the Flash Record System (FRS) for this sensor
//Returns true if metaData array is loaded successfully
//Returns false if failure
bool BNO085::readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead)
{
	uint8_t spot = 0;

	//First we send a Flash Record System (FRS) request
	frsReadRequest(recordID, startLocation, wordsToRead); //From startLocation of record, read a # of words

	//Read bytes until FRS reports that the read is complete
	while (1)
	{
		//Now we wait for response
		while (1)
		{
			uint8_t counter = 0;
			while (receivePacket() == false)
			{
				if (counter++ > 100)
					return (false); //Give up
				delay(1);   // Add HAL DELAY
			}

			//We have the packet, inspect it for the right contents
			//See page 40. Report ID should be 0xF3 and the FRS types should match the thing we requested
			if (shtpData[0] == SHTP_REPORT_FRS_READ_RESPONSE)
				if (((((uint16_t)shtpData[13]) << 8) | shtpData[12]) == recordID)
					break; //This packet is one we are looking for
		}

		uint8_t dataLength = shtpData[1] >> 4;
		uint8_t frsStatus = shtpData[1] & 0x0F;

		uint32_t data0 = (uint32_t)shtpData[7] << 24 | (uint32_t)shtpData[6] << 16 | (uint32_t)shtpData[5] << 8 | (uint32_t)shtpData[4];
		uint32_t data1 = (uint32_t)shtpData[11] << 24 | (uint32_t)shtpData[10] << 16 | (uint32_t)shtpData[9] << 8 | (uint32_t)shtpData[8];

		//Record these words to the metaData array
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
			return (true); //We have run out of space in our array. Bail.
		}

		if (frsStatus == 3 || frsStatus == 6 || frsStatus == 7)
		{
			return (true); //FRS status is read completed! We're done!
		}
	}
}

void BNO085::endCalibration()
{
	sendCalibrateCommand(CALIBRATE_STOP); //Disables all calibrations
}

//See page 51 of reference manual - ME Calibration Response
//Byte 5 is parsed during the readPacket and stored in calibrationStatus
bool BNO085::calibrationComplete()
{
	if (calibrationStatus == 0)
		return (true);
	return (false);
}

void BNO085::saveCalibration()
{
	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;
	sendCommand(COMMAND_DCD); //Save DCD command
}

void BNO085::requestCalibrationStatus()
{
	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	shtpData[6] = 0x01; //P3 - 0x01 - Subcommand: Get ME Calibration
	sendCommand(COMMAND_ME_CALIBRATE);
}

void BNO085::sendCalibrateCommand(uint8_t thingToCalibrate)
{

	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
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
		; //Do nothing, bytes are set to zero

	//Make the internal calStatus variable non-zero (operation failed) so that user can test while we wait
	calibrationStatus = 1;

	//Using this shtpData packet, send a command
	sendCommand(COMMAND_ME_CALIBRATE);
}

bool BNO085::sendCommand(uint8_t command)
{
	shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; //Command Request
	shtpData[1] = commandSequenceNumber++;	 //Increments automatically each function call
	shtpData[2] = command;					   //Command
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




