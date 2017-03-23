/*
 * our_dcm.c
 *
 *  Created on: Mar 22, 2017
 *      Author: Brandon Klefman
 *      		Abby Couto
 *
 *     	Late one night...
 */

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "sensorlib/comp_dcm.h"
#include "sensorlib/vector.h"

#include "driverlib/debug.h"

CustomCompDCMStart(tCompDCM *psDCM)
{
	float pfI[3], pfJ[3], pfK[3];

	//
	// The magnetometer reading forms the initial I vector, pointing north.
	//
    pfI[0] = psDCM->pfMagneto[0];
    pfI[1] = psDCM->pfMagneto[1];
    pfI[2] = psDCM->pfMagneto[2];

	//
	// The accelerometer reading forms the initial K vector, pointing down.
	//
	pfK[0] = psDCM->pfAccel[0];
	pfK[1] = psDCM->pfAccel[1];
	pfK[2] = psDCM->pfAccel[2];

	//
	// Compute the initial J vector, which is the cross product of the K and I
	// vectors.
	//
	VectorCrossProduct(pfJ, pfK, pfI);

	//
	// Recompute the I vector from the cross product of the J and K vectors.
	// This makes it fully orthogonal, which it wasn't before since magnetic
	// north points inside the Earth in many places.
	//
	VectorCrossProduct(pfI, pfJ, pfK);

	//
	// Normalize the I, J, and K vectors.
	//
	VectorScale(pfI, pfI, 1 / sqrtf(VectorDotProduct(pfI, pfI)));
	VectorScale(pfJ, pfJ, 1 / sqrtf(VectorDotProduct(pfJ, pfJ)));
	VectorScale(pfK, pfK, 1 / sqrtf(VectorDotProduct(pfK, pfK)));

	//
	// Initialize the DCM matrix from the I, J, and K vectors.
	//
	psDCM->ppfDCM[0][0] = pfI[0];
	psDCM->ppfDCM[0][1] = pfI[1];
	psDCM->ppfDCM[0][2] = pfI[2];
	psDCM->ppfDCM[1][0] = pfJ[0];
	psDCM->ppfDCM[1][1] = pfJ[1];
	psDCM->ppfDCM[1][2] = pfJ[2];
	psDCM->ppfDCM[2][0] = pfK[0];
	psDCM->ppfDCM[2][1] = pfK[1];
	psDCM->ppfDCM[2][2] = pfK[2];
}

CustomCompDCMUpdate(tCompDCM *psDCM)
{
    float pfI[3], pfJ[3], pfK[3], pfDelta[3], pfTemp[3], fError;
    bool bNAN;

    //
    // The magnetometer reading forms the new Im vector, pointing north.
    //
    pfI[0] = psDCM->pfMagneto[0];
    pfI[1] = psDCM->pfMagneto[1];
    pfI[2] = psDCM->pfMagneto[2];

    //
    // The accelerometer reading forms the new Ka vector, pointing down.
    //
    pfK[0] = psDCM->pfAccel[0];
    pfK[1] = psDCM->pfAccel[1];
    pfK[2] = psDCM->pfAccel[2];


    //
    // Compute the new J vector, which is the cross product of the Ka and Im
    // vectors.
    //
    VectorCrossProduct(pfJ, pfK, pfI);

    //
    // Recompute the Im vector from the cross product of the J and Ka vectors.
    // This makes it fully orthogonal, which it wasn't before since magnetic
    // north points inside the Earth in many places.
    //
    VectorCrossProduct(pfI, pfJ, pfK);

    //
    // Normalize the Im and Ka vectors.
    //
    VectorScale(pfI, pfI, 1 / sqrtf(VectorDotProduct(pfI, pfI)));
    VectorScale(pfK, pfK, 1 / sqrtf(VectorDotProduct(pfK, pfK)));

    //
    // Compute and scale the rotation as inferred from the accelerometer,
    // storing it in the rotation accumulator.
    //
    VectorCrossProduct(pfTemp, psDCM->ppfDCM[2], pfK);
    VectorScale(pfDelta, pfTemp, psDCM->fScaleA);

    //
    // Compute and scale the rotation as measured by the gyroscope, adding it
    // to the rotation accumulator.
    //
    pfTemp[0] = psDCM->pfGyro[0] * psDCM->fDeltaT * psDCM->fScaleG;
    pfTemp[1] = psDCM->pfGyro[1] * psDCM->fDeltaT * psDCM->fScaleG;
    pfTemp[2] = psDCM->pfGyro[2] * psDCM->fDeltaT * psDCM->fScaleG;
    VectorAdd(pfDelta, pfDelta, pfTemp);

    //
    // Compute and scale the rotation as inferred from the magnetometer, adding
    // it to the rotation accumulator.
    //
    VectorCrossProduct(pfTemp, psDCM->ppfDCM[0], pfI);
    VectorScale(pfTemp, pfTemp, psDCM->fScaleM);
    VectorAdd(pfDelta, pfDelta, pfTemp);

    //
    // Rotate the I vector from the DCM matrix by the scaled rotation.
    //
    VectorCrossProduct(pfI, pfDelta, psDCM->ppfDCM[0]);
    VectorAdd(psDCM->ppfDCM[0], psDCM->ppfDCM[0], pfI);

    //
    // Rotate the K vector from the DCM matrix by the scaled rotation.
    //
    VectorCrossProduct(pfK, pfDelta, psDCM->ppfDCM[2]);
    VectorAdd(psDCM->ppfDCM[2], psDCM->ppfDCM[2], pfK);

    //
    // Compute the orthogonality error between the rotated I and K vectors and
    // adjust each by half the error, bringing them closer to orthogonality.
    //
    fError = VectorDotProduct(psDCM->ppfDCM[0], psDCM->ppfDCM[2]) / -2.0;
    VectorScale(pfI, psDCM->ppfDCM[0], fError);
    VectorScale(pfK, psDCM->ppfDCM[2], fError);
    VectorAdd(psDCM->ppfDCM[0], psDCM->ppfDCM[0], pfK);
    VectorAdd(psDCM->ppfDCM[2], psDCM->ppfDCM[2], pfI);

    //
    // Normalize the I and K vectors.
    //
    VectorScale(psDCM->ppfDCM[0], psDCM->ppfDCM[0],
                0.5 * (3.0 - VectorDotProduct(psDCM->ppfDCM[0],
                                              psDCM->ppfDCM[0])));
    VectorScale(psDCM->ppfDCM[2], psDCM->ppfDCM[2],
                0.5 * (3.0 - VectorDotProduct(psDCM->ppfDCM[2],
                                              psDCM->ppfDCM[2])));

    //
    // Compute the rotated J vector from the cross product of the rotated,
    // corrected K and I vectors.
    //
    VectorCrossProduct(psDCM->ppfDCM[1], psDCM->ppfDCM[2], psDCM->ppfDCM[0]);

    //
    // Determine if the newly updated DCM contains any invalid (in other words,
    // NaN) values.
    //
    bNAN = (isnan(psDCM->ppfDCM[0][0]) ||
            isnan(psDCM->ppfDCM[0][1]) ||
            isnan(psDCM->ppfDCM[0][2]) ||
            isnan(psDCM->ppfDCM[1][0]) ||
            isnan(psDCM->ppfDCM[1][1]) ||
            isnan(psDCM->ppfDCM[1][2]) ||
            isnan(psDCM->ppfDCM[2][0]) ||
            isnan(psDCM->ppfDCM[2][1]) ||
            isnan(psDCM->ppfDCM[2][2]));

    //
    // As a debug measure, we check for NaN in the DCM.  The user can trap
    // this event depending on their implementation of __error__.  Should they
    // choose to disable interrupts and loop forever then they will have
    // preserved the stack and can analyze how they arrived at NaN.
    //
    ASSERT(!bNAN);

    //
    // If any part of the matrix is not-a-number then reset the DCM back to the
    // identity matrix.
    //
    if(bNAN)
    {
        psDCM->ppfDCM[0][0] = 1.0;
        psDCM->ppfDCM[0][1] = 0.0;
        psDCM->ppfDCM[0][2] = 0.0;
        psDCM->ppfDCM[1][0] = 0.0;
        psDCM->ppfDCM[1][1] = 1.0;
        psDCM->ppfDCM[1][2] = 0.0;
        psDCM->ppfDCM[2][0] = 0.0;
        psDCM->ppfDCM[2][1] = 0.0;
        psDCM->ppfDCM[2][2] = 1.0;
    }
}

