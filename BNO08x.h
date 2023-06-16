void HAL_Delay(int);  /// SHOULD BE REMOVED WHEN PORTING TO STM32 CUBE IDE
//sck-> PA5
//MOSI-> pa7 
//MISO-> pa6
//SS-> PB0  // For one bno 


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

uint8_t shtpHeader[4]; 
uint8_t shtpData[MAX_PACKET_SIZE];
uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; 
uint8_t commandSequenceNumber = 0;				
uint32_t metaData[MAX_METADATA_SIZE];

class BNO085
{
    public:
        	bool beginSPI(uint8_t user_CSPin, uint8_t user_WAKPin, uint8_t user_INTPin, uint8_t user_RSTPin); //Needs SPI Function
            bool waitForSPI(); //Needs HAL Function
            float qToFloat(int16_t fixedPointValue, uint8_t qPoint); // Verified
            void softReset(); // Needs SPI Function
            bool receivePacket(void); //Needs SPI Function 
            bool sendPacket(uint8_t channelNumber, uint8_t dataLength); //Needs SPI Function

            bool setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports); //Needs SPI Function
	        bool setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig);  //Needs SPI Function

            bool enableRotationVector(uint16_t timeBetweenReports); //Verified
            bool enableAccelerometer(uint16_t timeBetweenReports); //Verified
            bool enableLinearAccelerometer(uint16_t timeBetweenReports); //Verified
            bool enableGravity(uint16_t timeBetweenReports); //Verified
            bool enableGyro(uint16_t timeBetweenReports); //Verified
            bool enableMagnetometer(uint16_t timeBetweenReports); //Verified
            bool enableRawAccelerometer(uint16_t timeBetweenReports); //Verified
            bool enableRawGyro(uint16_t timeBetweenReports); //Verified
            bool enableRawMagnetometer(uint16_t timeBetweenReports); //Verified
            bool enableGyroIntegratedRotationVector(uint16_t timeBetweenReports); //Verified

            bool dataAvailable(void);  //Verified
            uint16_t getReadings(void);  //Verified
            uint16_t parseInputReport(void);   //Verified
	        uint16_t parseCommandReport(void); //Verified

            void getQuat(float &i, float &j, float &k, float &real, float &radAccuracy, uint8_t &accuracy);    //Verified
            float getQuatI();   //Verified
            float getQuatJ();   //Verified
            float getQuatK();   //Verified
            float getQuatReal();   //Verified
            float getQuatRadianAccuracy();   //Verified
            uint8_t getQuatAccuracy();   //Verified

            void getAccel(float &x, float &y, float &z, uint8_t &accuracy);   //Verified
            float getAccelX();   //Verified
            float getAccelY();   //Verified
            float getAccelZ();   //Verified
            uint8_t getAccelAccuracy();   //Verified

            void getLinAccel(float &x, float &y, float &z, uint8_t &accuracy);   //Verified
            float getLinAccelX();   //Verified
            float getLinAccelY();   //Verified
            float getLinAccelZ();   //Verified
            uint8_t getLinAccelAccuracy();   //Verified

            void getGyro(float &x, float &y, float &z, uint8_t &accuracy);   //Verified
            float getGyroX();   //Verified
            float getGyroY();   //Verified
            float getGyroZ();   //Verified   
            uint8_t getGyroAccuracy();   //Verified

            void getFastGyro(float &x, float &y, float &z);   //Verified
            float getFastGyroX();   //Verified
            float getFastGyroY();   //Verified  
            float getFastGyroZ();   //Verified

            void getMag(float &x, float &y, float &z, uint8_t &accuracy);   //Verified
            float getMagX();   //Verified
            float getMagY();   //Verified
            float getMagZ();   //Verified
            uint8_t getMagAccuracy();   //Verified

            void getGravity(float &x, float &y, float &z, uint8_t &accuracy);   //Verified
            float getGravityX();   //Verified
            float getGravityY();   //Verified
            float getGravityZ();   //Verified
            uint8_t getGravityAccuracy();   //Verified

            void sendCalibrateCommand(uint8_t thingToCalibrate);   //Verified
            bool sendCommand(uint8_t command);   //Verified

            void calibrateAccelerometer();   //Verified
            void calibrateGyro();   //Verified
            void calibrateMagnetometer();   //Verified
            void calibratePlanarAccelerometer();   //Verified
            void calibrateAll();   //Verified
            void endCalibration();   //Verified
            void saveCalibration();   //Verified
            void requestCalibrationStatus();    //Verified
            bool calibrationComplete();   //Verified

            int16_t getRawAccelX();   //Verified
            int16_t getRawAccelY();   //Verified
            int16_t getRawAccelZ();   //Verified

            int16_t getRawGyroX();   //Verified
            int16_t getRawGyroY();   //Verified
            int16_t getRawGyroZ();   //Verified

            int16_t getRawMagX();   //Verified
            int16_t getRawMagY();   //Verified
            int16_t getRawMagZ();   //Verified

            int16_t getQ1(uint16_t recordID);   //Verified
            int16_t getQ2(uint16_t recordID);   //Verified
            int16_t getQ3(uint16_t recordID);   //Verified
            float getResolution(uint16_t recordID);   //Verified
            float getRange(uint16_t recordID);   //Verified
            uint32_t readFRSword(uint16_t recordID, uint8_t wordNumber);   //Verified
            void frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize);   //Verified
            bool readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead);   //Verified






    private:
	uint8_t _cs;				 
	uint8_t _wake;
	uint8_t _int;
	uint8_t _rst;

	bool _hasReset = false;		


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
	uint8_t *_activityConfidences;						
	uint8_t calibrationStatus;							  
	uint16_t memsRawAccelX, memsRawAccelY, memsRawAccelZ; 
	uint16_t memsRawGyroX, memsRawGyroY, memsRawGyroZ;	
	uint16_t memsRawMagX, memsRawMagY, memsRawMagZ;		  
	int16_t rotationVector_Q1 = 14;
	int16_t rotationVectorAccuracy_Q1 = 12; 
	int16_t accelerometer_Q1 = 8;
	int16_t linear_accelerometer_Q1 = 8;
	int16_t gyro_Q1 = 9;
	int16_t magnetometer_Q1 = 4;
	int16_t angular_velocity_Q1 = 10;
	int16_t gravity_Q1 = 8;


};