#include <stdint.h>

const uint8_t CHANNEL_COMMAND = 0;
const uint8_t CHANNEL_EXECUTABLE = 1;
const uint8_t CHANNEL_CONTROL = 2;
const uint8_t CHANNEL_REPORTS = 3;
const uint8_t CHANNEL_WAKE_REPORTS = 4;
const uint8_t CHANNEL_GYRO = 5;

#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29

#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

#define EXECUTABLE_RESET_COMPLETE 0x1

#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define MAX_PACKET_SIZE 128 
#define MAX_METADATA_SIZE 9

uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
uint8_t shtpData[MAX_PACKET_SIZE];
uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
uint8_t commandSequenceNumber = 0;				//Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
uint32_t metaData[MAX_METADATA_SIZE];

class BNO085
{
    public:
        	bool beginSPI(uint8_t user_CSPin, uint8_t user_WAKPin, uint8_t user_INTPin, uint8_t user_RSTPin); //TINTIN
             // Reset Functions to be added 
            float qToFloat(int16_t fixedPointValue, uint8_t qPoint);
            // Wait for SPI
            bool receivePacket(void); //TINTIN
            bool sendPacket(uint8_t channelNumber, uint8_t dataLength); //TINTIN

            bool setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports);
	        bool setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig);

            bool enableRotationVector(uint16_t timeBetweenReports);
            bool enableAccelerometer(uint16_t timeBetweenReports);
            bool enableLinearAccelerometer(uint16_t timeBetweenReports);
            bool enableGravity(uint16_t timeBetweenReports);
            bool enableGyro(uint16_t timeBetweenReports);
            bool enableMagnetometer(uint16_t timeBetweenReports);
            bool enableRawAccelerometer(uint16_t timeBetweenReports);
            bool enableRawGyro(uint16_t timeBetweenReports);
            bool enableRawMagnetometer(uint16_t timeBetweenReports);
            bool enableGyroIntegratedRotationVector(uint16_t timeBetweenReports);

            bool dataAvailable(void);
            uint16_t getReadings(void);
            uint16_t parseInputReport(void);   //Parse sensor readings out of report
	        uint16_t parseCommandReport(void); //Parse command responses out of report

            void getQuat(float &i, float &j, float &k, float &real, float &radAccuracy, uint8_t &accuracy);
            float getQuatI();
            float getQuatJ();
            float getQuatK();
            float getQuatReal();
            float getQuatRadianAccuracy();
            uint8_t getQuatAccuracy();

            void getAccel(float &x, float &y, float &z, uint8_t &accuracy);
            float getAccelX();
            float getAccelY();
            float getAccelZ();
            uint8_t getAccelAccuracy();

            void getLinAccel(float &x, float &y, float &z, uint8_t &accuracy);
            float getLinAccelX();
            float getLinAccelY();
            float getLinAccelZ();
            uint8_t getLinAccelAccuracy();

            void getGyro(float &x, float &y, float &z, uint8_t &accuracy);
            float getGyroX();
            float getGyroY();
            float getGyroZ();
            uint8_t getGyroAccuracy();

            void getFastGyro(float &x, float &y, float &z);
            float getFastGyroX();
            float getFastGyroY();
            float getFastGyroZ();

            void getMag(float &x, float &y, float &z, uint8_t &accuracy);
            float getMagX();
            float getMagY();
            float getMagZ();
            uint8_t getMagAccuracy();

            void getGravity(float &x, float &y, float &z, uint8_t &accuracy);
            float getGravityX();
            float getGravityY();
            float getGravityZ();
            uint8_t getGravityAccuracy();

            void sendCalibrateCommand(uint8_t thingToCalibrate);
            bool sendCommand(uint8_t command);

            void calibrateAccelerometer();
            void calibrateGyro();
            void calibrateMagnetometer();
            void calibratePlanarAccelerometer();
            void calibrateAll();
            void endCalibration();
            void saveCalibration();
            void requestCalibrationStatus(); //Sends command to get status
            bool calibrationComplete();

            int16_t getRawAccelX();
            int16_t getRawAccelY();
            int16_t getRawAccelZ();

            int16_t getRawGyroX();
            int16_t getRawGyroY();
            int16_t getRawGyroZ();

            int16_t getRawMagX();
            int16_t getRawMagY();
            int16_t getRawMagZ();

            int16_t getQ1(uint16_t recordID);
            int16_t getQ2(uint16_t recordID);
            int16_t getQ3(uint16_t recordID);
            float getResolution(uint16_t recordID);
            float getRange(uint16_t recordID);
            uint32_t readFRSword(uint16_t recordID, uint8_t wordNumber);
            void frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize);
            bool readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead);






    private:
	uint8_t _cs;				 //Pins needed for SPI
	uint8_t _wake;
	uint8_t _int;
	uint8_t _rst;

	bool _hasReset = false;		// Keeps track of any Reset Complete packets we receive. 

	//These are the raw sensor values (without Q applied) pulled from the user requested Input Report
	uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
	uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
	uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
	uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
	uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
	uint16_t rawFastGyroX, rawFastGyroY, rawFastGyroZ;
	uint16_t gravityX, gravityY, gravityZ, gravityAccuracy;
	uint8_t tapDetector;
	uint16_t stepCount;
	uint32_t timeStamp;
	uint8_t stabilityClassifier;
	uint8_t activityClassifier;
	uint8_t *_activityConfidences;						  //Array that store the confidences of the 9 possible activities
	uint8_t calibrationStatus;							  //Byte R0 of ME Calibration Response
	uint16_t memsRawAccelX, memsRawAccelY, memsRawAccelZ; //Raw readings from MEMS sensor
	uint16_t memsRawGyroX, memsRawGyroY, memsRawGyroZ;	//Raw readings from MEMS sensor
	uint16_t memsRawMagX, memsRawMagY, memsRawMagZ;		  //Raw readings from MEMS sensor
	int16_t rotationVector_Q1 = 14;
	int16_t rotationVectorAccuracy_Q1 = 12; //Heading accuracy estimate in radians. The Q point is 12.
	int16_t accelerometer_Q1 = 8;
	int16_t linear_accelerometer_Q1 = 8;
	int16_t gyro_Q1 = 9;
	int16_t magnetometer_Q1 = 4;
	int16_t angular_velocity_Q1 = 10;
	int16_t gravity_Q1 = 8;


};