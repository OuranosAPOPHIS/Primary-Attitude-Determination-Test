/*
 * messingwithAtitutde.c
 *
 *  Created on: Mar 29, 2017
 *      Author: Brandon Klefman
 */


#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "sensorlib/vector.h"



void InitOrientation(float *accelData, float *magData, float *gyroData, float *EulerAngles)
{
	float down[3] = { 0 };
	float east[3] = { 0 };
	float north[3] = { 0 };
	float RotationMatrix[3][3] = { 0 };
	int i;

	//
	// Compute initial down vector.
	down[0] = accelData[0];
	down[1] = accelData[1];
	down[2] = accelData[2];

	//
	// Compute initial north vector.
	VectorCrossProduct(east, down, magData);

	//
	// Compute East vector.
	VectorCrossProduct(north, east, down);

	//
	// normalize the vectors.
    VectorScale(down, down, 1 / sqrtf(VectorDotProduct(down, down)));
    VectorScale(east, east, 1 / sqrtf(VectorDotProduct(east, east)));
    VectorScale(north, north, 1 / sqrtf(VectorDotProduct(north, north)));

    //
    // Build the DCM.
    for (i = 0; i < 2; i++)
		RotationMatrix[i][0] = north[i];
    for (i = 0; i < 2; i++)
    	RotationMatrix[i][1] = east[i];
    for (i = 0; i < 2; i++)
    	RotationMatrix[i][2] = down[i];

    //
    // Calculate Quaternion
    EulerAngles[0] = atan2f(RotationMatrix[2][1], RotationMatrix[2][2]);
    EulerAngles[1] = -asinf(RotationMatrix[2][0]);
    EulerAngles[2] = atan2f(RotationMatrix[1][0], RotationMatrix[0][0]);
}


