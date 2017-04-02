/*
 * attitude_estimation.h
 *
 *  Created on: Apr 2, 2017
 *      Author: Brandon Klefman
 */

#ifndef ATTITUDE_ESTIMATION_H_
#define ATTITUDE_ESTIMATION_H_

#define DT 0.01
#define DEG2RAD 0.01745329251
#define RAD2DEG 57.2957795131

#define M_PI 3.14159265358979323846

//****************************************************************************************
//
// Structure to hold all the sensor data.
//
//****************************************************************************************
typedef struct {
	float fAccelX;
	float fAccelY;
	float fAccelZ;
	float fGyroX;
	float fGyroY;
	float fGyroZ;
	float fMagX;
	float fMagY;
	float fMagZ;
	float fPitch;
	float fRoll;
	float fYaw;
	float fGyroWeight;
} sAttitudeData;

//****************************************************************************************
//
// Function Prototypes.
//
//****************************************************************************************
void InitAttitude(sAttitudeData *sAttData, float fGyroWeight);
void UpdateAccel(sAttitudeData *sAttData, float fAccelX, float fAccelY, float fAccelZ);
void UpdateGyro(sAttitudeData *sAttData, float fGyroX, float fGyroY, float fGyroZ);
void UpdateMag(sAttitudeData *sAttData, float fMagX, float fMagY, float fMagZ);
void InitHeading(sAttitudeData *sAttData);
void UpdateYaw(sAttitudeData *sAttData);
void UpdateRoll(sAttitudeData *sAttData);
void UpdatePitch(sAttitudeData *sAttData);
void UpdateAttitude(sAttitudeData *sAttData);

#endif /* ATTITUDE_ESTIMATION_H_ */
