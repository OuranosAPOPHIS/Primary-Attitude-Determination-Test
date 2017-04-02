/*
 * attitude_estimation.c
 *
 *  Created on: Apr 2, 2017
 *      Author: Brandon Klefman
 *
 *      Purpose: Custom attitude estimation algorithm
 *
 *      Note: These functions assume a time step of DT
 *      	  which is defined in the associated header
 *      	  file. The default setting for DT is 0.01.
 *
 *      Portions of this code were derived from TI's
 *      sensorlib compDCM functions. Additionally, a
 *      portion of this code was derived from github user Skadi15 - Christian Moncada.
 *      https://github.com/Skadi15/Overlord_System/blob/master/sensors/windrose_module.c
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "sensorlib/vector.h"

#include "attitude_estimation.h"

/*
 * Initialize the attitude.
 * Parameter(s):
 *  *sAttData - pointer to the attitude data structure as defined above.
 * 	*fAccel - pointer to the accel data.
 * 	*fGyro - pointer to the gyro data.
 * 	*fMag - pointer to the mag data.
 * 	*fEuler - pointer to the Euler angles (RPY).
 * 	fGyroWeight - weighting of the gyro.
 */
void InitAttitude(sAttitudeData *sAttData, float fGyroWeight)
{
	//
	// Set the structure values.
	sAttData->fGyroWeight = fGyroWeight;
}

/*
 * Update the accelerometer values.
 * Parameter(s):
 *  *sAttData - pointer to the attitude data structure as defined above.
 *  fAccelX,Y,Z - current x,y,z values for the accelerometer.
 */
void UpdateAccel(sAttitudeData *sAttData, float fAccelX, float fAccelY, float fAccelZ)
{
	//
	// Update the accelerometer values in the struct.
	sAttData->fAccelX = fAccelX;
	sAttData->fAccelY = fAccelY;
	sAttData->fAccelZ = fAccelZ;
}

/*
 * Update the gyroscope values.
 * Parameter(s):
 *  *sAttData - pointer to the attitude data structure as defined above.
 *  fGyroX,Y,Z - current x,y,z values for the gyroscope.
 */
void UpdateGyro(sAttitudeData *sAttData, float fGyroX, float fGyroY, float fGyroZ)
{
	//
	// Update the gyroscope values in the struct.
	sAttData->fGyroX = fGyroX;
	sAttData->fGyroY = fGyroY;
	sAttData->fGyroZ = fGyroZ;
}

/*
 * Update the magnetometer values.
 * Parameter(s):
 *  *sAttData - pointer to the attitude data structure as defined above.
 *  fMagX,Y,Z - current x,y,z values for the magnetometer.
 */
void UpdateMag(sAttitudeData *sAttData, float fMagX, float fMagY, float fMagZ)
{
	//
	// Update the magnetometer values in the struct.
	sAttData->fMagX = fMagX;
	sAttData->fMagY = fMagY;
	sAttData->fMagZ = fMagZ;
}

/*
 * InitHeading
 * Parameter(s):
 * *sAttData - Pointer to the sAttitudeData struct defined above.
 * Purpose:
 * 	Calculates the absolute 2D heading based on the magnetometer reading.
 */
void InitHeading(sAttitudeData *sAttData)
{
	sAttData->fYaw = atan2(-sAttData->fMagY, sAttData->fMagX);
}

/*
 * UpdateYaw
 * Parameter(s):
 * 	*sAttData - Pointer to the sAttitudeData struct defined above.
 * Purpose:
 * 	Updates the current heading based on magetometer and gyroscope readings.
 *
 * Note: This function must use calibrated gyro and mag data.
 */
void UpdateYaw(sAttitudeData *sAttData)
{
	float fGyroHeading;
	float fMagHeading;

	//
	// Check what reading the gyro is giving and convert to radians.
	fGyroHeading = sAttData->fYaw * DEG2RAD;

	//
	// Get the mag heading in radians.
	fMagHeading = atan2(-sAttData->fMagY, sAttData->fMagX);

	//
	// Check for negative mag values.
	if (fMagHeading < 0)
		fMagHeading += 2 * M_PI;

	//
	// Keep the values between +M_PI and -M_PI.
	if ((fMagHeading - fGyroHeading) < -M_PI)
		fMagHeading += 2 * M_PI;
	else if ((fMagHeading - fGyroHeading) > M_PI)
		fGyroHeading += 2 * M_PI;

	//
	// Update the yaw.
	sAttData->fYaw = (fMagHeading * (1 - (sAttData->fGyroWeight)) +
			fGyroHeading * (sAttData->fGyroWeight)) * RAD2DEG;

	//
	// Keep yaw between 0 and 360 degrees.
	if (sAttData->fYaw > 360.0f)
		sAttData->fYaw -= 360.0f;
}

/*
 * UpdateRoll
 * Parameter(s):
 * 	*sAttData - Pointer to the sAttitudeData struct defined above.
 * Purpose:
 * 	Updates the current roll based on accelerometer	and gyroscope readings.
 *
 * Note: This function must use calibrated accel and gyro data.
 */
void UpdateRoll(sAttitudeData *sAttData)
{
	// TODO: update roll function.
}

/*
 * UpdatePitch
 * Parameter(s):
 * 	*sAttData - Pointer to the sAttitudeData struct defined above.
 * Purpose:
 * 	Updates the current pitch based on accelerometer and gyroscope readings.
 *
 * Note: This function must use calibrated accel and gyro data.
 */
void UpdatePitch(sAttitudeData *sAttData)
{
	// TODO: update pitch function.
}

/*
 * UpdateAttitude
 * Parameter(s):
 * 	*sAttData - Pointer to the sAttitudeData struct defined above.
 * Purpose:
 * 	Updates the current attitude based on accelerometer, magetometer
 * 	and gyroscope readings.
 *
 * Note: This function must use calibrated accel, gyro and mag data.
 */
void UpdateAttitude(sAttitudeData *sAttData)
{
	//
	// Get the heading to update yaw.
	UpdateYaw(sAttData);

	//
	// Get the roll.
	UpdateRoll(sAttData);

	//
	// Get the pitch.
	UpdatePitch(sAttData);
}

/*
 * UpdateDCM
 * Parameter(s):
 * 	*sAttData - Pointer to the sAttitudeData struct defined above.
 * Purpose:
 * 	Updates the current DCM based on gyroscope readings.
 *
 * Note: This function must use calibrated accel, gyro and mag data.
 * This function was adapted from Dr. Stephen Bruder's MATLAB code
 * available on his website.
 * http://mercury.pr.erau.edu/~bruders/teaching/2017_spring/EE440/lectures/lecture%2016_Inertial_Nav_ECEF.pdf
 */
void UpdateDCM(sAttitutdeData *sAttData)
{
	//
	// Skew symmetric matrix.
	K[3][3] = { 0 };

	//
	// From Dr. Bruder's MATLAB code, adapted to C.
	Identity[3][3] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

	//
	// Compute the skew-symmetric matrix based on the gyroscope data.
	ComputeSkewMatrix(sAttData, &K);

	//C = I + Sin(theta) K + (1-Cos(theta)) K^2  = Rodrigues formula
	C = Identity + sin(theta) * K + (1 - cos(theta)) * pow(K, 2);
}

// K assumed to be a 3x3
void ComputeSkewMatrix(sAttitudeData *sAttData, float *K);
{
	sAttData->fGyroX;

}

void MatrixMultiply3x3(float Out[3][3], float firstThing[3][3], float secondThing[3][3])
{
	// do stuff here.
}
