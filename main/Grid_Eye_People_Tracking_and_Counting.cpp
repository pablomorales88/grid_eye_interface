/*
 * Grid_Eye_People_Tracking_and_Counting.c
 *
 * Created: 2017/2/09 16:20:50
 *  Author: Eric Paljan
 */ 
#include "grid_eye_config.h"
#include "Grid_Eye_People_Tracking_and_Counting.h"
#include <stdbool.h>
typedef enum {unproved, perhapsLeaved, matched, finished } ePersonStates;

// Algorithm Parameters
static USHORT g_usRectangleFilterLength = 3;
static USHORT g_usMedianFilterLength = 0;
//static float g_flIirFilterValue = 0.8;
static short g_shIirFilterValue = 205;
static eInterpolationType_enum g_eInterpolationType = tangential;
static float g_flTangentialInterpolationFactor = 0.35f;
static float g_flCornerThreshold = 0.45f;
static float g_flSideThreshold = 0.4f;
static float g_flCenterThreshold = 0.35f;
static BOOL g_bAutomaticPersonParameterControl = TRUE;
static BOOL g_bAutomaticParameterCalculation = TRUE;
static float g_flPersonRadius = 2;
static float g_flPersonArea = 10;
static float g_flPersonMaxSpeed = 2;
static float g_flAdaptionCoefficient = 0.995f;
static float g_flOverlappFactor = 0.0f;                      // Amount of pixels which are over lapping between two persons
static float g_flAreaFactor = 0.5f;                          // Amount of pixels which are needed to recognize a person
static float g_flTempFactor = 0.4f;                          //
static short g_shMaxMovementDistanceFactor = 5;
static short g_shMinPersonDistFactor = 1;
static USHORT g_usMaxEstimatedFrames = 5;
//static float g_flBackgroundIirFilterCoeff = 0.3f;
static short g_shBackgroundIirFilterCoeffNonObjPixel = 77;
static short g_shBackgroundIirFilterCoeffObjPixel = 25;

static float g_usHumanRadiusQuad = 4;
static short g_ucPeopleInMemory = 0;

// Initial value 77 is a threshold of 0.3�C
USHORT g_thresholdArray[] =
{
	77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77, 77
};

// Error detection variables
static short maxErrorFrames = 100;
static short errorFrameCounter = 0;
							
// Parameter Interface
static BOOL setRectangleFilterLength( USHORT );
static BOOL setRectangleFilterLength( USHORT value );
static BOOL setMedianFilterLength( USHORT value );
static BOOL setIirFilterValue( float value );
static BOOL setInterpolationType( eInterpolationType_enum value );
static BOOL setTangentialInterpolationFactor( float value );
static BOOL	setThresholdArray( float corner, float side, float center );
static BOOL setAutomaticPersonParameterControl( BOOL value );
static BOOL setAutomaticParameterCalculation( BOOL value );
static BOOL setPersonRadius( float value );
static BOOL setPersonArea( float value );
static BOOL setPersonMaxSpeed( float value );
static BOOL setAdaptionCoefficient( float value );
static BOOL setOverlappFactor( float value );
static BOOL setAreaFactor( float value );
static BOOL setTempFactor( float value );
static BOOL setMaxMovementDistanceFactor( short value);
static BOOL setMinPersonDistFactor( short value );
static BOOL setMaxEstimatedFrames( USHORT value );
static BOOL setBackgroundIirFilterCoeffNonObjPixel( float value );
static BOOL setBackgroundIirFilterCoeffObjPixel( float value );

// Common functions
static BOOL bAMG_PUB_PT_CalcAveTemp( USHORT usSize, UCHAR ucLabelNo, UCHAR* pucImg, short* pshImg, short* pshRet );
static BOOL bAMG_PUB_PT_CalcMinTemp( USHORT usSize, UCHAR ucLabelNo, UCHAR* pucImg, short* pshImg, short* pshRet );

// Image functions
static BOOL bAMG_PUB_PT_LinearInterpolationSQ15( short* pshInImg, short* pshOutImg );
static BOOL bAMG_PUB_PT_SpecialInterpolationSQ15( short* pshInImg, short* pshOutImg );

// Label algorithm sub functions
static void vAMG_PUB_ODT_ActualizeHumanRadiusAreaAndSpeed( float flRadius );
static USHORT findHighestTemperature( USHORT usImgSize, short* ashInputImg, BOOL* abWork );
static USHORT findLowestTemperature( USHORT usImgSize, short* ashInputImg );
static USHORT findHighestTempIndex( USHORT usImgSize, short* ashInputImg , BOOL* abWork );
static float estimateHumanSize( UCHAR ucImgZsY, UCHAR ucImgZsX, USHORT usHiTempIdx, short* ashInputImg, UCHAR* aucDetectImg );
static USHORT calculateObjectArea( UCHAR ucImgZsX, UCHAR ucImgZsY, UCHAR ucMark, float flHmnRadius, USHORT usHiTempIdx, short* ashInputImg, UCHAR* aucLabeledImg );
static void labelDetectedPerson( UCHAR ucImgZsX, UCHAR ucImgZsY, UCHAR  ucLabelNumber, float flHmnRadius, USHORT usHiTempIdx, short* ashInputImg, UCHAR* aucLabeledImg );

// Tracking algorithm sub functions
static void initializeArrays( UCHAR ucMaxPossiblePeople, UCHAR ucDetectNum, USHORT* ausMatchedPeople, USHORT* ausMatchedCOH );
static USHORT findHighestPeopleID( UCHAR ucMaxPossiblePeople, peopleStruct_t* astrPeoplePass );
static void markPotentialLeavedPersons( short shShortXSize, short shShortYSize, UCHAR ucMaxPossiblePeople, UCHAR ucPeopleInMemory, USHORT* ausMatchedPeople, peopleStruct_t* astrPeoplePass );
static void calculateDistances(UCHAR ucMaxPossiblePeople, UCHAR ucDetectNum, ULONG ulDistThreshold, coh_t* astrCenterOfHeat, peopleStruct_t* astrPeoplePass);
static void sortSpots(strFittingHeatSpot* strFittingHeatSpots);
static unsigned long calculateDistance( short x1, short y1, short x2, short y2 );
static void matchEstimatedPositionsWithCOH(UCHAR ucMaxPossiblePeople, ULONG ulDistThreshold, USHORT* ausMatchedPeople, USHORT* ausMatchedCOH, coh_t* astrLineOfHeat, peopleStruct_t* astrPeoplePass);
static void matchLastPositionsWithCOH( UCHAR ucMaxPossiblePeople, UCHAR ucPeopleInMemory, UCHAR ucDetectNum, ULONG ulDistThreshold, USHORT* ausMatchedPeople, USHORT* ausMatchedCOH, coh_t* astrCenterOfHeat, peopleStruct_t* astrPeoplePass );
static void matchMissingCOH( UCHAR ucImgSzX, UCHAR ucImgSzY, UCHAR ucMaxPossiblePeople, UCHAR ucDetectNum, ULONG ulDistThreshold, USHORT usNextID, USHORT* ausMatchedPeople, USHORT* ausMatchedCOH, coh_t* astrCenterOfHeat, peopleStruct_t* astrPeoplePass );
static UCHAR getNewID(peopleStruct_t* astrPeoplePass);
static void actualizePeopleData( UCHAR ucDetectNum, USHORT* ausMatchedCOH, USHORT* ausMatchedPeople, peopleStruct_t* astrPeoplePass, coh_t* astrCenterOfHeat );
static void setEstimatedPositionForTheRestOfPersons( UCHAR ucMaxPossiblePeople, ULONG ulDistRadiusThresh, USHORT* ausMatchedPeople, peopleStruct_t* astrPeoplePass, UCHAR* aucDetectImg );
static void deletePersons( UCHAR ucMaxPossiblePeople, USHORT* ausMatchedPeople, peopleStruct_t* astrPeoplePass );
static UCHAR getTrackedPersonNumber( peopleStruct_t* astrPeoplePass );

// Counting algorithm sub functions
static void actualizeCounterWithHysteresis( short personIndex, peopleStruct_t* g_astrPeoplePass, UCHAR ucNumberOfLines, line_t* strCountline, short* output );
static void actualizeCounter( short personIndex, peopleStruct_t* g_astrPeoplePass, UCHAR ucNumberOfLines, line_t* strCountline, short* output );

// Error detection private functions
static USHORT getAverage(short* inArray);

//*****************************************************************************
//
//! \addtogroup PTaC GridEye People Tracking and Counting
//! \brief Component for the realization of a people detecting, tracking and counting algorithm
//! @{
//
//*****************************************************************************
//*****************************************************************************
//
//! \addtogroup PTaC_func Sub functions
//! \brief The sub function reference for the People Tracking and Counting algorithm
//! @{
//
//*****************************************************************************
//*****************************************************************************
//
//! \brief Resets the algorithm to its initial state. Also the algorithm parameters will be set to initial values.
//
//*****************************************************************************
void vAMG_PUB_PT_Reset()
{
	g_flPersonRadius = 2;
	g_usHumanRadiusQuad = 4;
	g_flPersonArea = 10;
	g_flPersonMaxSpeed = 2;
	g_ucPeopleInMemory = 0;
	
	strAlgorithmParameters algoParams;
	algoParams.usRectangleFilterLength = 3;
	algoParams.usMedianFilterLength = 0;
	algoParams.flIirFilterValue = 0.8f;
	algoParams.eInterpolationType = tangential;
	algoParams.flTangentialInterpolationFactor = 0.35f;
	algoParams.flCornerThreshold = 0.45f;
	algoParams.flSideThreshold = 0.4f;
	algoParams.flCenterThreshold = 0.35f;
	algoParams.bAutomaticPersonParameterControl = TRUE;
	algoParams.bAutomaticParameterCalculation = TRUE;
	algoParams.flPersonRadius = 2;
	algoParams.flPersonArea = 10;
	algoParams.flPersonMaxSpeed = 2;
	algoParams.flAdaptionCoefficient = 0.995f;
	algoParams.flOverlappFactor = 0.0f;
	algoParams.flAreaFactor = 0.5f;
	algoParams.flTempFactor = 0.4f;
	algoParams.shMaxMovementDistanceFactor = 5;
	algoParams.shMinPersonDistFactor = 1;
	algoParams.usMaxEstimatedFrames = 5;
	algoParams.flBackgroundIirFilterCoeffNonObjPixel = 0.3f;
	algoParams.flBackgroundIirFilterCoeffObjPixel = 0.1f;
	vAMG_PUB_PC_SetParameters( algoParams );
}

//*****************************************************************************
//
//! \brief Sets the algorithm parameters
//! \param [in] strAlgoParams The input struct which stores the algorithm parameters.
//
//*****************************************************************************
void vAMG_PUB_PC_SetParameters( strAlgorithmParameters strAlgoParams )
{
	BOOL retVal = TRUE;
	if( !setRectangleFilterLength( strAlgoParams.usRectangleFilterLength ) ) { retVal = FALSE; };
	if( !setMedianFilterLength( strAlgoParams.usMedianFilterLength ) ) { retVal = FALSE; };
	if( !setIirFilterValue( strAlgoParams.flIirFilterValue ) ) { retVal = FALSE; };
	if( !setInterpolationType( strAlgoParams.eInterpolationType ) ) { retVal = FALSE; };
	if( !setTangentialInterpolationFactor( strAlgoParams.flTangentialInterpolationFactor ) ) { retVal = FALSE; };
	if( !setThresholdArray( strAlgoParams.flCornerThreshold, strAlgoParams.flSideThreshold, strAlgoParams.flCenterThreshold) ) { retVal = FALSE; };
	if( !setAutomaticPersonParameterControl( strAlgoParams.bAutomaticPersonParameterControl ) ) { retVal = FALSE; };
	if( !setAutomaticParameterCalculation( strAlgoParams.bAutomaticParameterCalculation ) ) { retVal = FALSE; };
	if( !setPersonRadius( strAlgoParams.flPersonRadius ) ) { retVal = FALSE; };
	if( !setPersonArea( strAlgoParams.flPersonArea ) ) { retVal = FALSE; };
	if( !setPersonMaxSpeed( strAlgoParams.flPersonMaxSpeed ) ) { retVal = FALSE; };
	if( !setAdaptionCoefficient( strAlgoParams.flAdaptionCoefficient ) ) { retVal = FALSE; };
	if( !setOverlappFactor( strAlgoParams.flOverlappFactor ) ) { retVal = FALSE; };
	if( !setAreaFactor( strAlgoParams.flAreaFactor ) ) { retVal = FALSE; };
	if( !setTempFactor( strAlgoParams.flTempFactor ) ) { retVal = FALSE; };
	if( !setMaxMovementDistanceFactor( strAlgoParams.shMaxMovementDistanceFactor) ) { retVal = FALSE; };
	if( !setMinPersonDistFactor( strAlgoParams.shMinPersonDistFactor ) ) { retVal = FALSE; };
	if( !setMaxEstimatedFrames( strAlgoParams.usMaxEstimatedFrames ) ) { retVal = FALSE; };
	if( !setBackgroundIirFilterCoeffNonObjPixel( strAlgoParams.flBackgroundIirFilterCoeffNonObjPixel ) ) { retVal = FALSE; };
	if( !setBackgroundIirFilterCoeffObjPixel( strAlgoParams.flBackgroundIirFilterCoeffObjPixel ) ) { retVal = FALSE; };
	//return retVal;
}
BOOL setRectangleFilterLength( USHORT value )
{
	if ( value < 1 )
	{
		g_usRectangleFilterLength = 1;
		setMedianFilterLength(g_usMedianFilterLength);
		return FALSE;
	}
	else if( value > 10 )
	{
		g_usRectangleFilterLength = 10;
		setMedianFilterLength(g_usMedianFilterLength);
		return FALSE;
	}
	else
	{
		g_usRectangleFilterLength = value;
		setMedianFilterLength(g_usMedianFilterLength);
		return TRUE;
	}
}
BOOL setMedianFilterLength( USHORT value )
{
	if ( g_usRectangleFilterLength > 0 )
	{
		if (g_usRectangleFilterLength % 2 == 0)
		{
			if ((value * 2) > g_usRectangleFilterLength - 1)
			{
				g_usMedianFilterLength = (UCHAR)((g_usRectangleFilterLength / 2) - 1);
				return FALSE;
			}
		}
		else
		{
			if ((value * 2) > g_usRectangleFilterLength)
			{
				g_usMedianFilterLength = (UCHAR)((g_usRectangleFilterLength / 2));
				return FALSE;
			}
		}
	}
	else if (value < 0)
	{
		g_usMedianFilterLength = 0;
		return FALSE;
	}
	
	g_usMedianFilterLength = value;
	return TRUE;
}
BOOL setIirFilterValue( float value )
{
	if ( value < 0 )
	{
		g_shIirFilterValue = 0;
		return FALSE;
	}
	else if( value > 1 )
	{
		g_shIirFilterValue = 1;
		return FALSE;
	}
	else
	{
		g_shIirFilterValue = shAMG_PUB_PT_ConvFtoS( value );
		return TRUE;
	}
}
BOOL setInterpolationType( eInterpolationType_enum value )
{
	g_eInterpolationType = value;
	return TRUE;
}
BOOL setTangentialInterpolationFactor( float value )
{
	if ( value < 0 )
	{
		g_flTangentialInterpolationFactor = 0;
		return FALSE;
	}
	else if( value > 1 )
	{
		g_flTangentialInterpolationFactor = 1;
		return FALSE;
	}
	else
	{
		g_flTangentialInterpolationFactor = value;
		return TRUE;
	}
}
BOOL setThresholdArray( float corner, float side, float center)
{
	USHORT i = 0;
	USHORT cornerThreshold = (USHORT)(corner * 256 + 0.5);
	USHORT sideThreshold = (USHORT)(side * 256 + 0.5);
	USHORT middleThreshold = (USHORT)(center * 256 + 0.5);

	USHORT thresholdArray[] =
	{
		cornerThreshold, cornerThreshold, cornerThreshold, sideThreshold, sideThreshold, sideThreshold, sideThreshold, sideThreshold, sideThreshold, sideThreshold, sideThreshold, sideThreshold, cornerThreshold, cornerThreshold, cornerThreshold,
		cornerThreshold, cornerThreshold, sideThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, sideThreshold, cornerThreshold, cornerThreshold,
		cornerThreshold, sideThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, sideThreshold, cornerThreshold,
		sideThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, sideThreshold,
		sideThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, sideThreshold,
		sideThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, sideThreshold,
		sideThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, sideThreshold,
		sideThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, sideThreshold,
		sideThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, sideThreshold,
		sideThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, sideThreshold,
		sideThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, sideThreshold,
		sideThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, sideThreshold,
		cornerThreshold, sideThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, sideThreshold, cornerThreshold,
		cornerThreshold, cornerThreshold, sideThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, middleThreshold, sideThreshold, cornerThreshold, cornerThreshold,
		cornerThreshold, cornerThreshold, cornerThreshold, sideThreshold, sideThreshold, sideThreshold, sideThreshold, sideThreshold, sideThreshold, sideThreshold, sideThreshold, sideThreshold, cornerThreshold, cornerThreshold, cornerThreshold
	};
	for ( i = 0; i < 225; i++ )
	{
		g_thresholdArray[i] = thresholdArray[i];
	}
}
BOOL setAutomaticPersonParameterControl( BOOL value )
{
	g_bAutomaticPersonParameterControl = value;
	return TRUE;
}
BOOL setAutomaticParameterCalculation( BOOL value )
{
	if (g_bAutomaticPersonParameterControl)
	{
		return FALSE;
	}
	else
	{
		g_bAutomaticParameterCalculation = value;
		return TRUE;
	}
}
BOOL setPersonRadius( float value )
{
	BOOL retValue = TRUE;
	if (!g_bAutomaticPersonParameterControl)
	{
		if ( value < 0 )
		{
			g_flPersonRadius = 0;
			retValue = FALSE;
		}
		else
		{
			g_flPersonRadius = value;
			retValue = TRUE;
		}
		if(g_bAutomaticParameterCalculation)
		{
			vAMG_PUB_ODT_ActualizeHumanRadiusAreaAndSpeed(value);
		}
		else
		{
			g_usHumanRadiusQuad = value * value;
		}
	}
	else
	{
		retValue = FALSE;
	}
	return retValue;
}
BOOL setPersonArea( float value )
{
	if(!g_bAutomaticPersonParameterControl && !g_bAutomaticParameterCalculation)
	{
		if ( value < 0 )
		{
			g_flPersonArea = 0;
			return FALSE;
		}
		else
		{
			g_flPersonArea = value;
			return TRUE;
		}
	}
	else
	{
		return FALSE;
	}
}
BOOL setPersonMaxSpeed( float value )
{
	if(!g_bAutomaticPersonParameterControl && !g_bAutomaticParameterCalculation)
	{
		if ( value < 0 )
		{
			g_flPersonMaxSpeed = 0;
			return FALSE;
		}
		else
		{
			g_flPersonMaxSpeed = value;
			return TRUE;
		}
	}
	else
	{
		return FALSE;
	}
}
BOOL setAdaptionCoefficient( float value )
{
	if ( value < 0 )
	{
		g_flAdaptionCoefficient = 0;
		return FALSE;
	}
	else if( value > 1 )
	{
		g_flAdaptionCoefficient = 1;
		return FALSE;
	}
	else
	{
		g_flAdaptionCoefficient = value;
		return TRUE;
	}
}
BOOL setOverlappFactor( float value )
{
	if ( value < 0 )
	{
		g_flOverlappFactor = 0;
		return FALSE;
	}
	else if( value > 1 )
	{
		g_flOverlappFactor = 1;
		return FALSE;
	}
	else
	{
		g_flOverlappFactor = value;
		return TRUE;
	}
}
BOOL setAreaFactor( float value )
{
	if ( value < 0 )
	{
		g_flAreaFactor = 0;
		return FALSE;
	}
	else if( value > 1 )
	{
		g_flAreaFactor = 1;
		return FALSE;
	}
	else
	{
		g_flAreaFactor = value;
		return TRUE;
	}
}
BOOL setTempFactor( float value )
{
	if ( value < 0 )
	{
		g_flTempFactor = 0;
		return FALSE;
	}
	else if( value > 1 )
	{
		g_flTempFactor = 1;
		return FALSE;
	}
	else
	{
		g_flTempFactor = value;
		return TRUE;
	}
}
BOOL setMaxMovementDistanceFactor( short value)
{
	if ( value < 0 )
	{
		g_shMaxMovementDistanceFactor = 0;
		return FALSE;
	}
	else
	{
		g_shMaxMovementDistanceFactor = value;
		return TRUE;
	}
}
BOOL setMinPersonDistFactor( short value )
{
	if ( value < 0 )
	{
		g_shMinPersonDistFactor = 0;
		return FALSE;
	}
	else
	{
		g_shMinPersonDistFactor = value;
		return TRUE;
	}
}
BOOL setMaxEstimatedFrames( USHORT value )
{
	if ( value < 0 )
	{
		g_usMaxEstimatedFrames = 0;
		return FALSE;
	}
	else
	{
		g_usMaxEstimatedFrames = value;
		return TRUE;
	}
}
BOOL setBackgroundIirFilterCoeffNonObjPixel( float value )
{
	if ( value < 0 )
	{
		g_shBackgroundIirFilterCoeffNonObjPixel = 0;
		return FALSE;
	}
	else if( value > 1 )
	{
		g_shBackgroundIirFilterCoeffNonObjPixel = 1;
		return FALSE;
	}
	else
	{
		g_shBackgroundIirFilterCoeffNonObjPixel = shAMG_PUB_PT_ConvFtoS( value );
		return TRUE;
	}
}
BOOL setBackgroundIirFilterCoeffObjPixel( float value )
{
	if ( value < 0 )
	{
		g_shBackgroundIirFilterCoeffObjPixel = 0;
		return FALSE;
	}
	else if( value > 1 )
	{
		g_shBackgroundIirFilterCoeffObjPixel = 1;
		return FALSE;
	}
	else
	{
		g_shBackgroundIirFilterCoeffObjPixel = shAMG_PUB_PT_ConvFtoS( value );
		return TRUE;
	}
}

//*****************************************************************************
//
//! \brief Converts a floating point format number into the Q8 fix point format.
//! \param [in] fVal The input number in a floating point format.
//
//! \return Returns the input value in a Q8 fix point number format.
//
//*****************************************************************************
short shAMG_PUB_PT_ConvFtoS( float fVal )
{
	return( ( fVal > 0 ) ? (short)(fVal * 256 + 0.5) : (short)(fVal * 256 - 0.5) );
}

/*------------------------------------------------------------------------------
	Calculate average temperature.
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_PT_CalcAveTemp( USHORT usSize, UCHAR ucLabelNo, UCHAR* pucImg, short* pshImg, short* pshRet )
{
	BOOL	bRet	= FALSE;
	USHORT	usImg	= 0;
	long	loSum	= 0;
	USHORT	usCnt	= 0;

	for( usImg = 0; usImg < usSize; usImg++ )
	{
		if( ucLabelNo == pucImg[usImg] )
		{
			bRet = TRUE;
			loSum += pshImg[usImg];
			usCnt++;
		}
	}

	if( FALSE != bRet )
	{
		*pshRet = (short)( loSum / usCnt );
	}

	return( bRet );
}

/*------------------------------------------------------------------------------
	Calculate the minimum temperature of the image.
------------------------------------------------------------------------------*/
BOOL bAMG_PUB_PT_CalcMinTemp( USHORT usSize, UCHAR ucLabelNo, UCHAR* pucImg, short* pshImg, short* pshRet )
{
	BOOL	bRet	= FALSE;
	USHORT	usImg	= 0;
	short	shMin	= SHORT_MAX_VAL;

	for( usImg = 0; usImg < usSize; usImg++ )
	{
		if( ucLabelNo == pucImg[usImg] )
		{
			bRet = TRUE;
			if( shMin > pshImg[usImg] )
			{
				shMin = pshImg[usImg];
			}
		}
	}

	if( FALSE != bRet )
	{
		*pshRet = shMin;
	}

	return( bRet );
}

//*****************************************************************************
//
//! \brief Calculate the center of heat position.
//! \param [in] ucWidth The image width in pixel.
//! \param [in] ucHeight The image height in pixel.
//! \param [in] ucLabelNo The label number of the area for which the center of heat should be calculated.
//! \param [in] pucImg Image which holds the labeled areas.
//! \param [in] pshImg Image which holds the temperature values.
//! \param [out] pshRet Stores the coordinates of the center of heat.<br>[0]: x-axis [1]: y-axis<br>Min value (outer pixel boundary): 0<br>Max value (outer pixel boundary for pixel 15): 3840
//
//! \return Returns false if there is no pixel with the given label.
//
//*****************************************************************************
BOOL bAMG_PUB_PT_CalcCenterTemp( UCHAR ucWidth, UCHAR ucHeight, UCHAR ucLabelNo, UCHAR* pucImg, short* pshImg, short* pshRet )
{
	BOOL	bRet	= FALSE;
	USHORT	usImg	= 0;
	USHORT	usSize	= ucWidth * ucHeight;
	short	shMin	= SHORT_MAX_VAL;
	ULONG	ulGx	= 0;
	ULONG	ulGy	= 0;
	ULONG	ulGw	= 0;

	if( FALSE != bAMG_PUB_PT_CalcMinTemp( usSize, ucLabelNo, pucImg, pshImg, &shMin ) )
	{
		shMin -= 1;
		bRet = TRUE;
	}
	if( FALSE != bRet )
	{
		for( usImg = 0; usImg < usSize; usImg++ )
		{
			if( ucLabelNo == pucImg[usImg] )
			{
				UCHAR	ucImgX = usImg % ucWidth;
				UCHAR	ucImgY = usImg / ucWidth;
				ULONG	ulWeight = (ULONG)( pshImg[usImg] - shMin );
				ulGx += ulWeight * (ucImgX + 1);
				ulGy += ulWeight * (ucImgY + 1);
				ulGw += ulWeight;
			}
		}

		if( 0 < ulGw )
		{
			pshRet[0] = ulGx * 256 / ulGw - 256;
			pshRet[1] = ulGy * 256 / ulGw - 256;
		}
		else
		{
			pshRet[0] = 0;
			pshRet[1] = 0;
		}
	}

	return( bRet );
}

//*****************************************************************************
//
//! \brief Calculate average and median filtered value.
//! \param [in] pshArray Pointer to the array which stores the values which should be filtered.
//! \param [in] ucSkip The raw image size in pixel.
//! \param [in] pbMedianWork Pointer to a helper array (like external storage).
//
//! \return Returns the filtered value.
//
//*****************************************************************************
short shAMG_PUB_PT_CalcAverage( short* pshArray, UCHAR ucSkip, BOOL* pbMedianWork )
{
	UCHAR ucMedian = g_usMedianFilterLength;
	USHORT usSize = g_usRectangleFilterLength;
	short shAve = 0;

	if( 1 >= usSize )
	{
		return( *pshArray );
	}

	/* Adjust parameter. */
	if( 1 > ucSkip )
	{
		ucSkip = 1;
	}
	if( ucMedian > ((usSize - 1) / 2) )
	{
		ucMedian = ((usSize - 1) / 2);
	}

	/* Calculate average. */
	if( 0 == ucMedian )
	{
		USHORT	usCnt = 0;
		long	loSum = 0;

		for( usCnt = 0; usCnt < usSize; usCnt++ )
		{
			short	shCurData = pshArray[usCnt * ucSkip];
			loSum += shCurData;
		}
		shAve = (short)(loSum / usSize);
	}
	else
	{
		USHORT	usCnt = 0;
		long	loSum = 0;
		UCHAR	ucMedianCnt = 0;

		for( usCnt = 0; usCnt < usSize; usCnt++ )
		{
			pbMedianWork[usCnt] = TRUE;
		}

		for( ucMedianCnt = 0; ucMedianCnt < ucMedian; ucMedianCnt++ )
		{
			short	shMaxData = SHORT_MIN_VAL;
			short	shMinData = SHORT_MAX_VAL;
			UCHAR	ucIndex = 0;

			for( usCnt = 0; usCnt < usSize; usCnt++ )
			{
				if( FALSE != pbMedianWork[usCnt] )
				{
					short	shCurData = pshArray[usCnt * ucSkip];
					if( shMaxData < shCurData )
					{
						shMaxData = shCurData;
						ucIndex = usCnt;
					}
				}
			}
			pbMedianWork[ucIndex] = FALSE;

			for( usCnt = 0; usCnt < usSize; usCnt++ )
			{
				if( FALSE != pbMedianWork[usCnt] )
				{
					short	shCurData = pshArray[usCnt * ucSkip];
					if( shMinData > shCurData )
					{
						shMinData = shCurData;
						ucIndex = usCnt;
					}
				}
			}
			pbMedianWork[ucIndex] = FALSE;
		}

		for( usCnt = 0; usCnt < usSize; usCnt++ )
		{
			short	shCurData = pshArray[usCnt * ucSkip];
			if( FALSE != pbMedianWork[usCnt] )
			{
				loSum += shCurData;
			}
		}
		shAve = (short)(loSum / ( usSize - ucMedian * 2 ));
	}

	return( shAve );
}

//*****************************************************************************
//
//! \brief Calculate IIR filtered value.
//! \param [in] shVal1 Last filtered value.
//! \param [in] shVal2 Actual value.
//
//! \return Returns the new filtered value.
//
//*****************************************************************************
short shAMG_PUB_PT_CalcIIR( short shVal1, short shVal2 )
{
	short shTh = g_shIirFilterValue;
	const short c_shMinTh = 0;
	const short c_shMaxTh = 256;
	long loAddVal = 0;

	/* Adjust parameter. */
	if( c_shMinTh > shTh )
	{
		shTh = c_shMinTh;
	}
	if( c_shMaxTh < shTh )
	{
		shTh = c_shMaxTh;
	}

	/* Calculate average. */
	loAddVal = (long)shTh * ( shVal2 - shVal1 );
	return( shVal1 + (short)(loAddVal / c_shMaxTh) );
}

//*****************************************************************************
//
//! \brief Converts the pixel temperature raw data into a temperature value with a Q8 fix point number format.
//! \param [in] aucRegVal The two bytes of the pixel temperature raw data you get from the I2C interface.
//
//! \return Returns the temperature value in a Q8 fix point number format.
//
//*****************************************************************************
short shAMG_PUB_PT_ConvTemperature( UCHAR aucRegVal[2] )
{
	short shVal = ((short)(aucRegVal[1] & 0x0F) << 8) | aucRegVal[0];
	if (0 != (0x08 & aucRegVal[1]) )
	{
		shVal = (short)(shVal | 0xF000);
	}
	if (shVal >= 512)
	{
		shVal = 511;
	}
	shVal *= 64;

	return( shVal );
}

//*****************************************************************************
//
//! \brief Converts the 64 pixel temperature raw data into a temperature values with a Q8 fix point number format.
//! \param [in] pucRegVal The 128 bytes raw data array of the pixel temperatures.
//! \param [out] pshVal An Array with 64 pixel temperature values in a Q8 fix point number format.
//
//*****************************************************************************
void vAMG_PUB_PT_ConvTemperature64( UCHAR* pucRegVal, short* pshVal, short shImageSize, ULONG ulFrameCounter )
{
	UCHAR ucCnt = 0;
	int value = 0;

	for( ucCnt = 0; ucCnt < SNR_SZ; ucCnt++ )
	{
		value = (ulFrameCounter % g_usRectangleFilterLength) * shImageSize + ucCnt;
		pshVal[ (ulFrameCounter % g_usRectangleFilterLength) * shImageSize + ucCnt ] = shAMG_PUB_PT_ConvTemperature( pucRegVal + ucCnt * 2 );
		//printf("  The %d tmp is %4x : %2x %2x \r\n",ucCnt,pshVal[ucCnt],*(pucRegVal + ucCnt * 2),*(pucRegVal + ucCnt * 2 + 1));
	}
}

//*****************************************************************************
//
//! \brief Creates an thermal image with temperature values only for pixels where an object is detected.
//! \param [in] usSize Total number of pixels of the image.
//! \param [in] pshOrigImg Original thermal image.
//! \param [in] pshDetectImg Binarisized image which includes information about object pixels.
//! \param [in] ucMark Pixel label within the binarisized image. Pixels with that label will be used to create the object image.
//! \param [out] pshOutImg Output image.
//
//*****************************************************************************
void vAMG_PUB_PT_CalcObjectImage( USHORT usSize, short* pshOrigImg, UCHAR* pshDetectImg, UCHAR ucMark, short* pshOutImg )
{
	USHORT usImg = 0;

	for( usImg = 0; usImg < usSize; usImg++ )
	{
		pshOutImg[usImg] = ( pshDetectImg[usImg] == ucMark ) ? pshOrigImg[usImg] : 0;
	}
}


//*****************************************************************************
//
//! \brief Search for potential persons within the object image and give them a label within the binarisized image.
//! \param [in] ucImgZsX Width of the image in pixels.
//! \param [in] ucImgZsY Height of the image in pixels.
//! \param [in] ucMark Pixel label within the binarisized image which indicates an object.
//! \param [in] ucMaxAmountOfHuman Maximum allowed amount of people within the image.
//! \param [in] ashInputImg The object image you can get with the vAMG_PUB_ODT_CalcObjectImage function.
//! \param [out] aucLabeledImg Binarisized image which includes information about object pixels. <b>This function will change the labels within this image</b>.
//! \param [in] abWork Working space for this function. <b>The size has to be ucImgZsX * ucImgZsY Bits</b>.
//
//*****************************************************************************
UCHAR ucAMG_PUB_PT_CalcPeopleLabeling( UCHAR ucImgZsX, UCHAR ucImgZsY, UCHAR ucMark, UCHAR ucMaxAmountOfHuman, short* ashInputImg, UCHAR* aucLabeledImg, BOOL* abWork )
{
	USHORT usIdx			= 0;
	short  shHiTemp			= -32768;
	short  shLoTemp			= SHORT_MAX_VAL;		// Max value for USHORT
	USHORT usHiTempIdx		= 0;
	short  shThresholdTemp	= 0;
	
	float flHmnRadius = 0.0f;
	
	USHORT usImgSize			= (USHORT)ucImgZsX * ucImgZsY;
	USHORT usAreaCounter		= 0;
	UCHAR  ucLabelNumber		= 1;
	
	USHORT usAreaThreshold		= (USHORT)(g_flPersonArea + 0.5) * g_flAreaFactor;		// Amount of pixels which are needed to recognize a person
	
	
	// Initialize work array
	for( usIdx = 0; usIdx < usImgSize; usIdx++ )
	{
		abWork[usIdx] = 0;
	}
	
	// Find highest and lowest Temperature and calculate the temperature threshold.
	shHiTemp = findHighestTemperature( usImgSize, (short*)ashInputImg, (BOOL*)abWork );
	shLoTemp = findLowestTemperature( usImgSize, (short*)ashInputImg );
	shThresholdTemp = (short)(shLoTemp + (shHiTemp - shLoTemp) * g_flTempFactor);	
	
	// People labeling algorithm
	if( shHiTemp != 0 )
	{
		while( shHiTemp > shThresholdTemp )
		{
			usAreaCounter		= 0;
			shHiTemp			= -32768;

			// Find the index of the hottest temperature pixel. Already labeled or used values are excluded
			shHiTemp = findHighestTemperature( usImgSize, (short*)ashInputImg, (BOOL*)abWork );
			usHiTempIdx = findHighestTempIndex( usImgSize, (short*)ashInputImg, (BOOL*)abWork );
			
			// Estimate the object size with the hottest temperature pixel
			flHmnRadius = estimateHumanSize( ucImgZsY, ucImgZsX, usHiTempIdx, (short*)ashInputImg, (UCHAR*)aucLabeledImg );
			
			// Calculate the object area Object area of the object with the hottest temperature pixel
			usAreaCounter = calculateObjectArea( ucImgZsX, ucImgZsY, ucMark, flHmnRadius, usHiTempIdx, (short*)ashInputImg, (UCHAR*)aucLabeledImg );
			
			// Label detected person if the size is big enough for a human
			if( usAreaCounter > usAreaThreshold )
			{
				// Actualize the human radius and area variable
				if (g_bAutomaticPersonParameterControl)
				{
					vAMG_PUB_ODT_ActualizeHumanRadiusAreaAndSpeed( flHmnRadius );
				}
				labelDetectedPerson( ucImgZsX, ucImgZsY, ucLabelNumber, flHmnRadius, usHiTempIdx, (short*)ashInputImg, (UCHAR*)aucLabeledImg );
				
				ucLabelNumber++;
				if ( (ucLabelNumber - 1) > ucMaxAmountOfHuman )
				{
					return(ucLabelNumber - 1);
				}
			}
			else
			{
				// Exclude this point for the following loops
				abWork[usHiTempIdx] = TRUE;
			}
		}
	}
	return( ucLabelNumber - 1 );		// No detectable people left
}


static void vAMG_PUB_ODT_ActualizeHumanRadiusAreaAndSpeed( float flRadius )
{
	float usHelper = 0;
	float iirCoeff = 0;
	if (g_bAutomaticPersonParameterControl)
	{
		iirCoeff = g_flAdaptionCoefficient;
	}
	if ( g_usHumanRadiusQuad == 0 )
	{
		// Initial value
		g_flPersonRadius = flRadius;
		g_flPersonArea = flRadius * flRadius * 3.14159265359f;
		g_usHumanRadiusQuad = flRadius * flRadius;
		g_flPersonMaxSpeed = flRadius * 0.24f / 0.225f;				// 0.24 = speed of a person in m/s. 0.225 = radius of a person in m.
	}
	else
	{
		g_flPersonRadius = flRadius + (g_flPersonRadius - flRadius) * iirCoeff;	// * 19 / 20 = 0.95
		
		usHelper = flRadius * flRadius * 3.14159265359f;
		g_flPersonArea = usHelper + (g_flPersonArea - usHelper) * iirCoeff;	// * 19 / 20 = 0.95
		
		usHelper = flRadius * flRadius + 0.5f;
		g_usHumanRadiusQuad = usHelper + (g_usHumanRadiusQuad - usHelper) * iirCoeff;	// * 19 / 20 = 0.95
		
		usHelper = flRadius * 0.24f / 0.225f;
		g_flPersonMaxSpeed = usHelper + (g_flPersonMaxSpeed - usHelper) * iirCoeff;
	}
}

static USHORT findHighestTemperature( USHORT usImgSize, short* ashInputImg, BOOL* abWork )
{
	USHORT usIdx = 0;
	short shHiTemp	= -32768;
	for( usIdx = 0; usIdx < usImgSize; usIdx++ )
	{
		if( (ashInputImg[usIdx] > shHiTemp) && (abWork[usIdx] != 1) )
		{
			shHiTemp = ashInputImg[usIdx];
		}
	}
	return shHiTemp;
}

static USHORT findLowestTemperature( USHORT usImgSize, short* ashInputImg )
{
	USHORT usIdx = 0;
	short shLoTemp	= SHORT_MAX_VAL;
	for( usIdx = 0; usIdx < usImgSize; usIdx++ )
	{
		if( (ashInputImg[usIdx] < shLoTemp) && (ashInputImg[usIdx] != 0) )
		{
			shLoTemp = ashInputImg[usIdx];
		}
	}
	return shLoTemp;
}

static USHORT findHighestTempIndex( USHORT usImgSize, short* ashInputImg , BOOL* abWork )
{
	USHORT usIdx = 0;
	short shHiTemp	= -32768;
	USHORT usHiTempIdx = 0;
	for( usIdx = 0; usIdx < usImgSize; usIdx++ )
	{
		if( (ashInputImg[usIdx] > shHiTemp) && (abWork[usIdx] != 1) )
		{
			shHiTemp = ashInputImg[usIdx];
			usHiTempIdx = usIdx;
		}
	}
	return usHiTempIdx;
}

static float estimateHumanSize( UCHAR ucImgZsY, UCHAR ucImgZsX, USHORT usHiTempIdx, short* ashInputImg, UCHAR* aucDetectImg )
{
	USHORT usImgSize = ucImgZsX * ucImgZsY;
	USHORT ausSizeArray[4] = { 0, 0, 0, 0 };
	USHORT usSizeArrayDevisor = 0;
	USHORT usLastTempValue = 0;
	short shSizeOffset = 0;
	short shOffset = 0;
	float flHmnRadius = 0.0f;
	BOOL bBigEnough = TRUE;
	
	
	
	ausSizeArray[0] = 0;
	ausSizeArray[1] = 0;
	ausSizeArray[2] = 0;
	ausSizeArray[3] = 0;
	usSizeArrayDevisor = 0;
	// Calculate coordinates of the actually point of interest and the start index
	//usPointYCoord = (ushort)(usHiTempIdx / ucImgZsX);     // Y coordinate
	USHORT usPointXCoord = (USHORT)(usHiTempIdx % ucImgZsX);     // X coordinate
	
	// Estimate distance to the object boundary in -y direction
	usLastTempValue = (USHORT)ashInputImg[usHiTempIdx];
	shSizeOffset = (short)-ucImgZsX;
	shOffset = (short)shSizeOffset;
	//Check if the point is too close to a labeled person
	if (usHiTempIdx + shOffset >= 0)
	{
		if (aucDetectImg[usHiTempIdx + shOffset] != DETECT_MARK)
		{
			bBigEnough = FALSE;
		}
	}
	while ((usHiTempIdx + shOffset >= 0)
	&& (ashInputImg[usHiTempIdx + shOffset] != 0)
	&& (ashInputImg[usHiTempIdx + shOffset] <= usLastTempValue)
	&& (bBigEnough) )
	{
		ausSizeArray[0]++;
		usLastTempValue = (USHORT)ashInputImg[usHiTempIdx + shOffset];
		shOffset += (short)shSizeOffset;
		if (usHiTempIdx + shOffset < 0)
		{
			ausSizeArray[0] = 0;
		}
	}
	if (ausSizeArray[0] != 0)
	{
		usSizeArrayDevisor++;
	}
	// Estimate distance to the object boundary in +y direction
	usLastTempValue = (USHORT)ashInputImg[usHiTempIdx];
	shSizeOffset = ucImgZsX;
	shOffset = (short)shSizeOffset;
	//Check if the point is too close to a labeled person
	if (usHiTempIdx + shOffset < usImgSize)
	{
		if (aucDetectImg[usHiTempIdx + shOffset] != DETECT_MARK)
		{
			bBigEnough = FALSE;
		}
	}
	while ((usHiTempIdx + shOffset < usImgSize)
	&& (ashInputImg[usHiTempIdx + shOffset] != 0)
	&& (ashInputImg[usHiTempIdx + shOffset] <= usLastTempValue)
	&& (bBigEnough))
	{
		ausSizeArray[1]++;
		usLastTempValue = (USHORT)ashInputImg[usHiTempIdx + shOffset];
		shOffset += (short)shSizeOffset;
		if (usHiTempIdx + shOffset >= usImgSize)
		{
			ausSizeArray[1] = 0;
		}
	}
	if (ausSizeArray[1] != 0)
	{
		usSizeArrayDevisor++;
	}
	// Estimate distance to the object boundary in -x direction
	usLastTempValue = (USHORT)ashInputImg[usHiTempIdx];
	shSizeOffset = -1;
	shOffset = shSizeOffset;
	//Check if the point is too close to a labeled person
	if (usPointXCoord - (ausSizeArray[2] + 1) >= 0)
	{
		if (aucDetectImg[usHiTempIdx + shOffset] != DETECT_MARK)
		{
			bBigEnough = FALSE;
		}
	}
	while ((usPointXCoord - (ausSizeArray[2] + 1) >= 0)
	&& (ashInputImg[usHiTempIdx + shOffset] != 0)
	&& (ashInputImg[usHiTempIdx + shOffset] <= usLastTempValue)
	&& (bBigEnough))
	{
		ausSizeArray[2]++;
		usLastTempValue = (USHORT)ashInputImg[usHiTempIdx + shOffset];
		shOffset += shSizeOffset;
	}
	if (usPointXCoord - (ausSizeArray[2] + 1) < 0)
	{
		ausSizeArray[2] = 0;
	}
	if (ausSizeArray[2] != 0)
	{
		usSizeArrayDevisor++;
	}
	// Estimate distance to the object boundary in +x direction
	usLastTempValue = (USHORT)ashInputImg[usHiTempIdx];
	shSizeOffset = 1;
	shOffset = shSizeOffset;
	//Check if the point is too close to a labeled person
	if (usPointXCoord + (ausSizeArray[3] + 1) < ucImgZsX)
	{
		if (aucDetectImg[usHiTempIdx + shOffset] != DETECT_MARK)
		{
			bBigEnough = FALSE;
		}
	}
	while ((usPointXCoord + (ausSizeArray[3] + 1) < ucImgZsX)
	&& (ashInputImg[usHiTempIdx + shOffset] != 0)
	&& (ashInputImg[usHiTempIdx + shOffset] <= usLastTempValue)
	&& (bBigEnough))
	{
		ausSizeArray[3]++;
		usLastTempValue = (USHORT)ashInputImg[usHiTempIdx + shOffset];
		shOffset += shSizeOffset;
	}
	if (usPointXCoord + (ausSizeArray[3] + 1) >= ucImgZsX)
	{
		ausSizeArray[3] = 0;
	}
	if (ausSizeArray[3] != 0)
	{
		usSizeArrayDevisor++;
	}

	if ( (usSizeArrayDevisor > 0) && (bBigEnough) )
	{
		flHmnRadius = (float)(ausSizeArray[0] + ausSizeArray[1] + ausSizeArray[2] + ausSizeArray[3]) / usSizeArrayDevisor;
	}
	else
	{
		flHmnRadius = 0;
	}
	
	return flHmnRadius;
}

static USHORT calculateObjectArea( UCHAR ucImgZsX, UCHAR ucImgZsY, UCHAR ucMark, float flHmnRadius, USHORT usHiTempIdx, short* ashInputImg, UCHAR* aucLabeledImg )
{
	USHORT usIdx_X			= 0;
	USHORT usIdx_Y			= 0;
	USHORT usPointXCoord	= 0;
	USHORT usPointYCoord	= 0;
	short  shActXCoord		= 0;
	short  shActYCoord		= 0;
	short  shXDist			= 0;
	short  shYDist			= 0;
	
	USHORT usHmnPxlRadius = (USHORT)(flHmnRadius + 0.5f);
	USHORT usRadiusQuad	= 0;
	
	USHORT usOverlappThreshold	= (USHORT)(g_flPersonArea + 0.5) * g_flOverlappFactor;		// Amount of pixels which are over lapping between two persons
	USHORT usAreaThreshold		= (USHORT)(g_flPersonArea + 0.5) * g_flAreaFactor;		// Amount of pixels which are needed to recognize a person
	
	USHORT usHmnMaskSizeX		= 0;
	USHORT usHmnMaskSizeY		= 0;
	USHORT usHmnMaskSize		= 0;
	short shOffset				= 0;
	USHORT usAreaCounter		= 0;
	USHORT usOverlappCounter	= 0;
	BOOL   bDetectInLine		= FALSE;
	
	// Convert radius in short for the following calculations
	shOffset = usHiTempIdx - usHmnPxlRadius * ucImgZsX - usHmnPxlRadius;	// Start index
	
	// Calculate coordinates of the actually point of interest and the start index
	usPointYCoord = usHiTempIdx / ucImgZsX;			// Y coordinate
	usPointXCoord = usHiTempIdx % ucImgZsX;			// X coordinate
	shActYCoord = usPointYCoord - usHmnPxlRadius;	// Y coordinate of first test point in human area
	shActXCoord = usPointXCoord - usHmnPxlRadius;	// X coordinate of first test point in human area
	// Count valid pixels within the radius
	usHmnMaskSizeX = usHmnPxlRadius * 2 + 1;
	usHmnMaskSizeY = usHmnPxlRadius * 2 + 1;
	usHmnMaskSize = usHmnMaskSizeX * usHmnMaskSizeY;
	usRadiusQuad = (USHORT)(flHmnRadius * flHmnRadius);
	for( usIdx_Y = 0; usIdx_Y < usHmnMaskSizeY; usIdx_Y++ )
	{
		bDetectInLine = FALSE;
		
		for( usIdx_X = 0; usIdx_X < usHmnMaskSizeX; usIdx_X++ )
		{
			// Check if (x1-x2)^2 + (y1-y2)^2 <= r^2
			shXDist = usPointXCoord - shActXCoord;
			shYDist = usPointYCoord - shActYCoord;
			if( (shXDist * shXDist) + (shYDist * shYDist) <= usRadiusQuad )
			{
				// Pixel is within the human radius
				if( shActXCoord < 0 || shActXCoord >= ucImgZsX || shActYCoord < 0 || shActYCoord >= ucImgZsY )
				{
					// Pixel is outside the image boundaries
					//usAreaCounter++;
				}
				else
				{
					// Pixel is inside the image boundaries
					if( (ashInputImg[ shOffset + usIdx_X + (usIdx_Y * ucImgZsX) ] != 0) && ( aucLabeledImg[ shOffset + usIdx_X + (usIdx_Y * ucImgZsX) ] == ucMark) )
					{
						usAreaCounter++;
					}
					//
					if( aucLabeledImg[ shOffset + usIdx_X + (usIdx_Y * ucImgZsX) ] != ucMark && aucLabeledImg[ shOffset + usIdx_X + (usIdx_Y * ucImgZsX) ] != 0 )
					{
						usOverlappCounter++;
					}
				}
				bDetectInLine = TRUE;
			}
			else if( bDetectInLine == TRUE )
			{
				// In this line none of the next pixel will be within the radius
				break;
			}
			shActXCoord++;
		}
		if( (usAreaCounter + (usHmnMaskSize - usIdx_X - (usIdx_Y * usHmnMaskSizeY) ) ) < usAreaThreshold )
		{
			// The needed size of AreaCounter cant be reached
			break;
		}
		if ( usOverlappCounter > usOverlappThreshold )
		{
			// The Area overlaps a labeled area too much
			usAreaCounter = 0;
			break;
		}
		shActYCoord++;
		shActXCoord = usPointXCoord - usHmnPxlRadius;
	}
	return usAreaCounter;
}

static void labelDetectedPerson( UCHAR ucImgZsX, UCHAR ucImgZsY, UCHAR  ucLabelNumber, float flHmnRadius, USHORT usHiTempIdx, short* ashInputImg, UCHAR* aucLabeledImg )
{
	USHORT usIdx_X			= 0;
	USHORT usIdx_Y			= 0;
	USHORT usPointXCoord	= 0;
	USHORT usPointYCoord	= 0;
	short  shActXCoord		= 0;
	short  shActYCoord		= 0;
	short  shXDist			= 0;
	short  shYDist			= 0;
	
	USHORT usHmnPxlRadius	= (USHORT)(flHmnRadius + 0.5f);
	USHORT usLabelRadius	= (USHORT)(g_usHumanRadiusQuad + 0.5f);
	
	USHORT usHmnMaskSizeX	= 0;
	USHORT usHmnMaskSizeY	= 0;
	short shOffset			= 0;
	BOOL   bDetectInLine	= FALSE;
	
	// Calculate the start index
	shOffset = usHiTempIdx - usHmnPxlRadius * ucImgZsX - usHmnPxlRadius;
	
	// Calculate coordinates of the actually point of interest and the start index
	usPointYCoord = usHiTempIdx / ucImgZsY;			// Y coordinate
	usPointXCoord = usHiTempIdx % ucImgZsX;			// X coordinate
	
	// Calculate coordinates of the actually point of interest
	shActYCoord = usPointYCoord - usHmnPxlRadius;	// Y coordinate of first test point in human area
	shActXCoord = usPointXCoord - usHmnPxlRadius;	// X coordinate of first test point in human area
	
	// Calculate the area of interest around the label center
	usHmnMaskSizeX = usHmnPxlRadius * 2 + 1;
	usHmnMaskSizeY = usHmnPxlRadius * 2 + 1;
	
	// Count valid pixels within the radius
	for( usIdx_Y = 0; usIdx_Y < usHmnMaskSizeY; usIdx_Y++ )
	{
		if( (shActYCoord >= 0) && (shActYCoord < ucImgZsY) )
		{
			// Line is inside the boundaries
			bDetectInLine = FALSE;
			
			for( usIdx_X = 0; usIdx_X < usHmnMaskSizeX; usIdx_X++ )
			{
				// Check if (x1-x2)^2 + (y1-y2)^2 <= r^2
				shXDist = usPointXCoord - shActXCoord;
				shYDist = usPointYCoord - shActYCoord;
				if( (shXDist * shXDist) + (shYDist * shYDist) <= usLabelRadius )
				{
					// Pixel is within the human radius
					if( shActXCoord >= 0 && shActXCoord < ucImgZsX && shActYCoord >= 0 && shActYCoord < ucImgZsY )
					{
						// It is within the image boundaries
						if( ashInputImg[ shOffset + usIdx_X + (usIdx_Y * ucImgZsX) ] != 0 )
						{
							aucLabeledImg[ shOffset + usIdx_X + (usIdx_Y * ucImgZsX) ] = ucLabelNumber;
							ashInputImg[ shOffset + usIdx_X + (usIdx_Y * ucImgZsX) ] = 0;
						}
					}
					bDetectInLine = TRUE;
				}
				else if( bDetectInLine == TRUE )
				{
					// In this line none of the next pixel will be within the radius
					break;
				}
				shActXCoord++;
			}
		}
		shActYCoord++;
		shActXCoord = usPointXCoord - usHmnPxlRadius;
	}
}




//*****************************************************************************
//
//! \brief This function uses the information of detected potential people to track people. Every tracked person get an ID, an actual position and a last position.
//! \param [in] ucXSize Width of the image in pixels.
//! \param [in] ucYSize Height of the image in pixels.
//! \param [in] ucDetectNum Amount of detected persons. You will get this number from the ucAMG_PUB_ODT_CalcPeopleLabeling function.
//! \param [in] astrCenterOfHeat The center of heat for every detected potential person.
//! \param [in] ucPeopleInMemory The amount of tracked persons. Its the result of this function one frame before.
//! \param [in] ucMaxPossiblePeople Maximum allowed amount of people within the image.
//! \param [out] astrPeoplePass It stores every necessary information about the tracked persons.
//
//! \return This function returns the amount of actual tracked persons.
//
//*****************************************************************************
UCHAR vAMG_PUB_PT_PeopleTracking( UCHAR ucXSize, UCHAR ucYSize, coh_t* astrCenterOfHeat, UCHAR ucDetectedPeople, UCHAR ucMaxPossiblePeople, peopleStruct_t* astrPeoplePass, UCHAR* aucDetectImg )
{
	USHORT	usNextID			= 0;
	UCHAR	ucReturnValue		= 0;
	
	short	shShortXSize		= shAMG_PUB_PT_ConvFtoS(ucXSize);
	short	shShortYSize		= shAMG_PUB_PT_ConvFtoS(ucYSize);
	ULONG	ulDistThreshold		= (ULONG)shAMG_PUB_PT_ConvFtoS(g_flPersonMaxSpeed) * (ULONG)shAMG_PUB_PT_ConvFtoS(g_flPersonMaxSpeed) * g_shMaxMovementDistanceFactor;
	//ULONG	ulDistRadiusThresh	= (ULONG)shAMG_PUB_CMN_ConvFtoS(g_usHumanRadius) * (ULONG)shAMG_PUB_CMN_ConvFtoS(g_usHumanRadius) * 0.3;
	ULONG	ulDistRadiusThresh = (ULONG)((ULONG)shAMG_PUB_PT_ConvFtoS(g_flPersonRadius) * (ULONG)shAMG_PUB_PT_ConvFtoS(g_flPersonRadius) * g_shMinPersonDistFactor);
	
	USHORT	ausMatchedPeople[ucMaxPossiblePeople];
	USHORT	ausMatchedCOH[ucDetectedPeople];
	
	
	// Initialize match array and search for highest ID and save it
	initializeArrays( ucMaxPossiblePeople, ucDetectedPeople, (USHORT*)ausMatchedPeople, (USHORT*)ausMatchedCOH );
	usNextID = findHighestPeopleID( ucMaxPossiblePeople, (peopleStruct_t*)astrPeoplePass );
	
	//// Mark persons who potentially leaved the sensor view
	//markPotentialLeavedPersons( shShortXSize, shShortYSize, ucMaxPossiblePeople, ucPeopleInMemory, (USHORT*)ausMatchedPeople, (peopleStruct_t*)astrPeoplePass );
	
	// Calculate best fitting spots for existing persons
	calculateDistances(ucMaxPossiblePeople, ucDetectedPeople, ulDistThreshold, astrCenterOfHeat, astrPeoplePass);
	
	// Match coh with estimated positions first.
	matchEstimatedPositionsWithCOH( ucMaxPossiblePeople, ulDistThreshold, (USHORT*)ausMatchedPeople, (USHORT*)ausMatchedCOH, (coh_t*)astrCenterOfHeat, (peopleStruct_t*)astrPeoplePass);
	
	// All remaining center of heats must be new persons, doubled persons or persons who was estimated outside the image
	matchMissingCOH( ucXSize, ucYSize, ucMaxPossiblePeople, ucDetectedPeople, ulDistThreshold, usNextID, (USHORT*)ausMatchedPeople, (USHORT*)ausMatchedCOH, (coh_t*)astrCenterOfHeat, (peopleStruct_t*)astrPeoplePass);
	
	// Actualize people data
	actualizePeopleData( ucDetectedPeople, (USHORT*)ausMatchedCOH, (USHORT*)ausMatchedPeople, (peopleStruct_t*)astrPeoplePass, (coh_t*)astrCenterOfHeat );
	
	// Choose estimated position, if the person cant be outside the image and there is no labeled coh that fits.
	setEstimatedPositionForTheRestOfPersons( ucMaxPossiblePeople, ulDistRadiusThresh, (USHORT*)ausMatchedPeople, (peopleStruct_t*)astrPeoplePass, (UCHAR*)aucDetectImg );
	
	// Delete persons
	deletePersons( ucMaxPossiblePeople, (USHORT*)ausMatchedPeople, (peopleStruct_t*)astrPeoplePass);
	
	// Get the amount of tracked persons
	ucReturnValue = getTrackedPersonNumber( (peopleStruct_t*)astrPeoplePass );
	
	return ucReturnValue;
}

static void initializeArrays( UCHAR ucMaxPossiblePeople, UCHAR ucDetectNum, USHORT* ausMatchedPeople, USHORT* ausMatchedCOH )
{
	USHORT idx = 0;
	for ( idx = 0; idx < ucMaxPossiblePeople; idx++ )
	{
		ausMatchedPeople[idx] = unproved;
	}
	
	// Initialize center of heat match array
	for ( idx = 0; idx < ucDetectNum; idx++ )
	{
		ausMatchedCOH[idx] = SHORT_MAX_VAL;
	}
}

static USHORT findHighestPeopleID( UCHAR ucMaxPossiblePeople, peopleStruct_t* astrPeoplePass )
{
	USHORT idx = 0;
	USHORT usNextID = 0;
	for ( idx = 0; idx < ucMaxPossiblePeople; idx++ )
	{
		if( astrPeoplePass[idx].ucID > usNextID )
		{
			usNextID = astrPeoplePass[idx].ucID;
		}
	}
	return usNextID;
}

static void calculateDistances(UCHAR ucMaxPossiblePeople, UCHAR ucDetectNum, ULONG ulDistThreshold, coh_t* astrCenterOfHeat, peopleStruct_t* astrPeoplePass)
{
	USHORT usPplIdx = 0;
	UCHAR ucHeatSpotIdx = 0;
	USHORT usBreakIdx = 0;
	UCHAR bPeoplePassHeatSpotIdx = 0;
	UCHAR bMaxValueIdx = UCHAR_MAX_VAL;
	ULONG ulDistance = ULONG_MAX_VAL;
	ULONG ulMaxDistance = 0;
	
	// Initialize fitting heat spot arrays
	for (usPplIdx = 0; usPplIdx < ucMaxPossiblePeople; usPplIdx++)
	{
		if (astrPeoplePass[usPplIdx].ucID != 0)
		{
			for (bPeoplePassHeatSpotIdx = 0; bPeoplePassHeatSpotIdx < 3; bPeoplePassHeatSpotIdx++)
			{
				astrPeoplePass[usPplIdx].strFittingHeatSpots[bPeoplePassHeatSpotIdx].distance = ULONG_MAX_VAL;
				astrPeoplePass[usPplIdx].strFittingHeatSpots[bPeoplePassHeatSpotIdx].ucHeatSpotId = UCHAR_MAX_VAL;
			}
		}
	}

	for (usPplIdx = 0; usPplIdx < ucMaxPossiblePeople; usPplIdx++)
	{
		if (astrPeoplePass[usPplIdx].ucID != 0)
		{
			for (ucHeatSpotIdx = 0; ucHeatSpotIdx < ucDetectNum; ucHeatSpotIdx++)
			{
				ulDistance = calculateDistance(astrCenterOfHeat[ucHeatSpotIdx].ashCOH[0], astrCenterOfHeat[ucHeatSpotIdx].ashCOH[1], (short)astrPeoplePass[usPplIdx].usEstPosX, (short)astrPeoplePass[usPplIdx].usEstPosY);
				// !!! Prove if it is in the allowed range !!!
				if (ulDistance <= ulDistThreshold)
				{
					ulMaxDistance = 0;
					for (bPeoplePassHeatSpotIdx = 0; bPeoplePassHeatSpotIdx < 3; bPeoplePassHeatSpotIdx++)
					{
						if (ulDistance < astrPeoplePass[usPplIdx].strFittingHeatSpots[bPeoplePassHeatSpotIdx].distance)
						{
							if(ulMaxDistance < astrPeoplePass[usPplIdx].strFittingHeatSpots[bPeoplePassHeatSpotIdx].distance)
							{
								bMaxValueIdx = bPeoplePassHeatSpotIdx;
								ulMaxDistance = astrPeoplePass[usPplIdx].strFittingHeatSpots[bPeoplePassHeatSpotIdx].distance;
							}
							if (astrPeoplePass[usPplIdx].strFittingHeatSpots[bPeoplePassHeatSpotIdx].distance == ULONG_MAX_VAL)
							{
								break;
							}
						}
					}
					if (bMaxValueIdx != UCHAR_MAX_VAL)
					{
						astrPeoplePass[usPplIdx].strFittingHeatSpots[bMaxValueIdx].distance = ulDistance;
						astrPeoplePass[usPplIdx].strFittingHeatSpots[bMaxValueIdx].ucHeatSpotId = ucHeatSpotIdx;
					}
				}
			}
			// Sort the spots
			sortSpots(astrPeoplePass[usPplIdx].strFittingHeatSpots);

			usBreakIdx++;
			if (usBreakIdx >= g_ucPeopleInMemory)
			{
				usBreakIdx = 0;
				break;
			}
		}
	}
}

static void sortSpots(strFittingHeatSpot* strFittingHeatSpots)
{
	ULONG ulDistanceHelper = 0;
	UCHAR ucIndexHelper = 0;
	UCHAR bPeoplePassHeatSpotIdx = 0;
	BOOL bUnsorted = TRUE;
	while (bUnsorted)
	{
		bUnsorted = FALSE;
		for (bPeoplePassHeatSpotIdx = 0; bPeoplePassHeatSpotIdx < 2; bPeoplePassHeatSpotIdx++)
		{
			if (strFittingHeatSpots[bPeoplePassHeatSpotIdx].distance > strFittingHeatSpots[bPeoplePassHeatSpotIdx + 1].distance)
			{
				// Change positions
				ulDistanceHelper = strFittingHeatSpots[bPeoplePassHeatSpotIdx].distance;
				strFittingHeatSpots[bPeoplePassHeatSpotIdx].distance = strFittingHeatSpots[bPeoplePassHeatSpotIdx + 1].distance;
				strFittingHeatSpots[bPeoplePassHeatSpotIdx + 1].distance = ulDistanceHelper;
				ucIndexHelper = strFittingHeatSpots[bPeoplePassHeatSpotIdx].ucHeatSpotId;
				strFittingHeatSpots[bPeoplePassHeatSpotIdx].ucHeatSpotId = strFittingHeatSpots[bPeoplePassHeatSpotIdx + 1].ucHeatSpotId;
				strFittingHeatSpots[bPeoplePassHeatSpotIdx + 1].ucHeatSpotId = ucIndexHelper;
				bUnsorted = TRUE;
			}
		}
	}
}

static void markPotentialLeavedPersons( short shShortXSize, short shShortYSize, UCHAR ucMaxPossiblePeople, UCHAR ucPeopleInMemory, USHORT* ausMatchedPeople, peopleStruct_t* astrPeoplePass )
{
	USHORT usPplIdx = 0;
	USHORT usBreakIdx = 0;
	// Estimate if people potentially leaved the image
	for( usPplIdx = 0; usPplIdx < ucMaxPossiblePeople; usPplIdx++ )
	{
		if ( astrPeoplePass[usPplIdx].ucID != 0 )
		{
			if( astrPeoplePass[usPplIdx].bNewPerson == FALSE )
			{
				// Is the estimated position within the image?
				if( (astrPeoplePass[usPplIdx].usEstPosX < 0) || (astrPeoplePass[usPplIdx].usEstPosX > shShortXSize) || (astrPeoplePass[usPplIdx].usEstPosY < 0) || (astrPeoplePass[usPplIdx].usEstPosY > shShortYSize) )
				{
					ausMatchedPeople[usPplIdx] = perhapsLeaved;
				}
			}
			usBreakIdx++;
			if( usBreakIdx >= ucPeopleInMemory)
			{
				usBreakIdx = 0;
				break;
			}
		}
	}
}

static unsigned long calculateDistance( short x1, short y1, short x2, short y2 )
{
	short shXDist	= x1 - x2;
	short shYDist	= y1 - y2;
	return (shXDist * shXDist) + (shYDist * shYDist);
}

static void matchEstimatedPositionsWithCOH(UCHAR ucMaxPossiblePeople, ULONG ulDistThreshold, USHORT* ausMatchedPeople, USHORT* ausMatchedCOH, coh_t* astrLineOfHeat, peopleStruct_t* astrPeoplePass)
{
	USHORT usPplIdx = 0;
	USHORT usBreakIdx = 0;

	UCHAR bPerson1HeatSpotIdx = 0;
	UCHAR bPerson2HeatSpotIdx = 0;
	UCHAR bPerson2Idx = 0;
	BOOL conflict = TRUE;

	// Match coh for people with estimated coh first.
	while (conflict)
	{
		// reset conflict indicator
		conflict = FALSE;
		for (usPplIdx = 0; usPplIdx < ucMaxPossiblePeople; usPplIdx++)
		{
			if (astrPeoplePass[usPplIdx].ucID != 0)
			{
				if (ausMatchedPeople[usPplIdx] == (USHORT)unproved)
				{
					for (bPerson1HeatSpotIdx = 0; bPerson1HeatSpotIdx < 3; bPerson1HeatSpotIdx++)
					{
						// Hot spot is not matched until now
						if (astrPeoplePass[usPplIdx].strFittingHeatSpots[bPerson1HeatSpotIdx].ucHeatSpotId != UCHAR_MAX_VAL)
						{
							if (ausMatchedCOH[astrPeoplePass[usPplIdx].strFittingHeatSpots[bPerson1HeatSpotIdx].ucHeatSpotId] == SHORT_MAX_VAL)
							{
								ausMatchedPeople[usPplIdx] = (USHORT)matched;
								ausMatchedCOH[astrPeoplePass[usPplIdx].strFittingHeatSpots[bPerson1HeatSpotIdx].ucHeatSpotId] = usPplIdx;
								break;
							}
							// Hot spot is already matched. Check which person fits better.
							else
							{
								// Get index of the person which is bound to the same hot spot (Person 2)
								bPerson2Idx = (UCHAR)ausMatchedCOH[astrPeoplePass[usPplIdx].strFittingHeatSpots[bPerson1HeatSpotIdx].ucHeatSpotId];
								// Search for the right hot spot index in people pass of person 2
								for (bPerson2HeatSpotIdx = 0; bPerson2HeatSpotIdx < 3; bPerson2HeatSpotIdx++)
								{
									if (astrPeoplePass[bPerson2Idx].strFittingHeatSpots[bPerson2HeatSpotIdx].ucHeatSpotId == astrPeoplePass[usPplIdx].strFittingHeatSpots[bPerson1HeatSpotIdx].ucHeatSpotId)
									{
										break;
									}
								}
								// Check which person fits better
								if (astrPeoplePass[usPplIdx].strFittingHeatSpots[bPerson1HeatSpotIdx].distance < astrPeoplePass[bPerson2Idx].strFittingHeatSpots[bPerson2HeatSpotIdx].distance)
								{
									ausMatchedPeople[usPplIdx] = (USHORT)matched;
									ausMatchedCOH[astrPeoplePass[usPplIdx].strFittingHeatSpots[bPerson1HeatSpotIdx].ucHeatSpotId] = usPplIdx;

									ausMatchedPeople[bPerson2Idx] = (USHORT)unproved;
									conflict = TRUE;
									break;
								}
							}
						}
					}
				}
				usBreakIdx++;
				if (usBreakIdx >= g_ucPeopleInMemory)
				{
					usBreakIdx = 0;
					break;
				}
			}
		}
	}
}

static void matchMissingCOH( UCHAR ucImgSzX, UCHAR ucImgSzY, UCHAR ucMaxPossiblePeople, UCHAR ucDetectNum, ULONG ulDistThreshold, USHORT usNextID, USHORT* ausMatchedPeople, USHORT* ausMatchedCOH, coh_t* astrCenterOfHeat, peopleStruct_t* astrPeoplePass )
{
	USHORT usPplIdx		= 0;
	USHORT usHeatIdx	= 0;
	
	ULONG	ulDistance			= 0;
	ULONG	ulShortestDist		= ULONG_MAX_VAL;		// Highest Value for unsigned long
	USHORT	usIdxShortestDist	= 0;
	
	// All remaining center of heats must be new persons, doubled persons or persons who was estimated outside the image
	for (usHeatIdx = 0; usHeatIdx < ucDetectNum; usHeatIdx++)
	{
		if (ausMatchedCOH[usHeatIdx] == SHORT_MAX_VAL)
		{
			// Search for shortest distance between coh and estimated position
			ulShortestDist = ULONG_MAX_VAL;
			for (usPplIdx = 0; usPplIdx < MAX_PEOPLE_IN_IMG; usPplIdx++)
			{
				if (astrPeoplePass[usPplIdx].ucID != 0)
				{
					ulDistance = calculateDistance(astrCenterOfHeat[usHeatIdx].ashCOH[0], astrCenterOfHeat[usHeatIdx].ashCOH[1], (short)astrPeoplePass[usPplIdx].usEstPosX, (short)astrPeoplePass[usPplIdx].usEstPosY);
					if (ulShortestDist > ulDistance)
					{
						ulShortestDist = ulDistance;
						usIdxShortestDist = usPplIdx;
					}
				}
			}
			if (astrCenterOfHeat[usHeatIdx].ashCOH[0] <= 5 * 256 || astrCenterOfHeat[usHeatIdx].ashCOH[0] >= (ucImgSzX - 6) * 256 || astrCenterOfHeat[usHeatIdx].ashCOH[1] <= 5 * 256 || astrCenterOfHeat[usHeatIdx].ashCOH[1] >= (ucImgSzY - 6) * 256)
			{
				// Must be a new person. Create a new person.
				for (usPplIdx = 0; usPplIdx < MAX_PEOPLE_IN_IMG; usPplIdx++)
				{
					if (astrPeoplePass[usPplIdx].ucID == 0)
					{
						astrPeoplePass[usPplIdx].ucID = getNewID((peopleStruct_t*)astrPeoplePass);//(UCHAR)usNextID;
						astrPeoplePass[usPplIdx].usActPosX = astrCenterOfHeat[usHeatIdx].ashCOH[0];
						astrPeoplePass[usPplIdx].usActPosY = astrCenterOfHeat[usHeatIdx].ashCOH[1];
						astrPeoplePass[usPplIdx].usOldPosX = astrCenterOfHeat[usHeatIdx].ashCOH[0];
						astrPeoplePass[usPplIdx].usOldPosY = astrCenterOfHeat[usHeatIdx].ashCOH[1];
						astrPeoplePass[usPplIdx].usEstPosX = astrCenterOfHeat[usHeatIdx].ashCOH[0];
						astrPeoplePass[usPplIdx].usEstPosY = astrCenterOfHeat[usHeatIdx].ashCOH[1];
						//astrPeoplePass[usPplIdx].bNewPerson = true;
						ausMatchedPeople[usPplIdx] = (USHORT)finished;

						break;
					}
				}
			}
		}
	}
}

UCHAR getNewID(peopleStruct_t* astrPeoplePass)
{
	UCHAR newIndex = 1;
	short shPplIndex = 0;
	BOOL bFoundNewIndex = FALSE;
	while (bFoundNewIndex == FALSE || newIndex == 0xFF)
	{
		for (shPplIndex = 0; shPplIndex < MAX_PEOPLE_IN_IMG; shPplIndex++)
		{
			if(astrPeoplePass[shPplIndex].ucID == newIndex)
			{
				break;
			}
		}
		if(shPplIndex >= MAX_PEOPLE_IN_IMG)
		{
			bFoundNewIndex = true;
		}
		else
		{
			newIndex++;
		}
	}
	return newIndex;
}

static void actualizePeopleData( UCHAR ucDetectNum, USHORT* ausMatchedCOH, USHORT* ausMatchedPeople, peopleStruct_t* astrPeoplePass, coh_t* astrCenterOfHeat )
{
	USHORT usHeatIdx	= 0;
	short	shXDist				= 0;
	short	shYDist				= 0;
	
	// Actualize people data
	for (usHeatIdx = 0; usHeatIdx < ucDetectNum; usHeatIdx++)
	{
		if ((ausMatchedCOH[usHeatIdx] != SHORT_MAX_VAL) && (ausMatchedPeople[ausMatchedCOH[usHeatIdx]] != (USHORT)finished))
		{
			// Save last position
			astrPeoplePass[ausMatchedCOH[usHeatIdx]].usOldPosX = astrPeoplePass[ausMatchedCOH[usHeatIdx]].usActPosX;
			astrPeoplePass[ausMatchedCOH[usHeatIdx]].usOldPosY = astrPeoplePass[ausMatchedCOH[usHeatIdx]].usActPosY;

			// Save new position
			astrPeoplePass[ausMatchedCOH[usHeatIdx]].usActPosX = astrCenterOfHeat[usHeatIdx].ashCOH[0];
			astrPeoplePass[ausMatchedCOH[usHeatIdx]].usActPosY = astrCenterOfHeat[usHeatIdx].ashCOH[1];
			//astrPeoplePass[ausMatchedCOH[usHeatIdx]].usActPos.X = (peopleDetector.astrCenterOfHeat[usHeatIdx].ashCOH[0] + astrPeoplePass[ausMatchedCOH[usHeatIdx]].usOldPos.X) / 2;
			//astrPeoplePass[ausMatchedCOH[usHeatIdx]].usActPos.Y = (peopleDetector.astrCenterOfHeat[usHeatIdx].ashCOH[1] + astrPeoplePass[ausMatchedCOH[usHeatIdx]].usOldPos.Y) / 2;


			// Estimate next position
			// Calculate the difference between old and last position
			shXDist = (short)(astrPeoplePass[ausMatchedCOH[usHeatIdx]].usActPosX - astrPeoplePass[ausMatchedCOH[usHeatIdx]].usOldPosX);
			shYDist = (short)(astrPeoplePass[ausMatchedCOH[usHeatIdx]].usActPosY - astrPeoplePass[ausMatchedCOH[usHeatIdx]].usOldPosY);
			// Estimate
			//astrPeoplePass[ausMatchedCOH[usHeatIdx]].usEstPos.X = astrPeoplePass[ausMatchedCOH[usHeatIdx]].usActPos.X + shXDist;
			//astrPeoplePass[ausMatchedCOH[usHeatIdx]].usEstPos.Y = astrPeoplePass[ausMatchedCOH[usHeatIdx]].usActPos.Y + shYDist;
			astrPeoplePass[ausMatchedCOH[usHeatIdx]].usEstPosX = astrPeoplePass[ausMatchedCOH[usHeatIdx]].usActPosX;
			astrPeoplePass[ausMatchedCOH[usHeatIdx]].usEstPosY = astrPeoplePass[ausMatchedCOH[usHeatIdx]].usActPosY;

			// Delete the new person marker
			//astrPeoplePass[ausMatchedCOH[usHeatIdx]].bNewPerson = false;
			ausMatchedPeople[ausMatchedCOH[usHeatIdx]] = (USHORT)finished;
			// Reset estimation counter
			astrPeoplePass[ausMatchedCOH[usHeatIdx]].shEstFrames = 0;

			astrPeoplePass[ausMatchedCOH[usHeatIdx]].bHided = FALSE;

			// Trust only detected persons which are matched at least 3 times in a row after first detection.
			if(astrPeoplePass[ausMatchedCOH[usHeatIdx]].shTrackCounter >= 3)
			{
				astrPeoplePass[ausMatchedCOH[usHeatIdx]].bValidPerson = TRUE;
			}
			else
			{
				astrPeoplePass[ausMatchedCOH[usHeatIdx]].shTrackCounter++;
			}
		}
	}
}

static void setEstimatedPositionForTheRestOfPersons( UCHAR ucMaxPossiblePeople, ULONG ulDistRadiusThresh, USHORT* ausMatchedPeople, peopleStruct_t* astrPeoplePass, UCHAR* aucDetectImg )
{
	USHORT	usPplIdx = 0;
	USHORT	usPplIdx2 = 0;
	short	shXDist	= 0;
	short	shYDist	= 0;
	USHORT	usBreakIdx = 0;
	USHORT	usBreakIdx2 = 0;
	ULONG	ulDistance = 0;
	USHORT	usPosInDetectionArray = 0;
	USHORT usEstFrameThreshold = g_usMaxEstimatedFrames;
	
	// Choose estimated position, if the person cant be outside the image and there is no labeled coh that fits.
	for (usPplIdx = 0; usPplIdx < MAX_PEOPLE_IN_IMG; usPplIdx++)
	{
		if (astrPeoplePass[usPplIdx].ucID != 0)
		{
			if (ausMatchedPeople[usPplIdx] != (USHORT)finished)
			{
				// Delete persons which are less than three times in a row detected after first detection.
				if (astrPeoplePass[usPplIdx].shTrackCounter < 3)
				{
					ausMatchedPeople[usPplIdx] = (USHORT)perhapsLeaved;
				}
				else
				{
					astrPeoplePass[usPplIdx].usOldPosX = astrPeoplePass[usPplIdx].usActPosX;
					astrPeoplePass[usPplIdx].usOldPosY = astrPeoplePass[usPplIdx].usActPosY;
					astrPeoplePass[usPplIdx].usActPosX = astrPeoplePass[usPplIdx].usEstPosX;
					astrPeoplePass[usPplIdx].usActPosY = astrPeoplePass[usPplIdx].usEstPosY;

					// Estimate next position
					// Calculate the difference between old and last position
					shXDist = (short)(astrPeoplePass[usPplIdx].usActPosX - astrPeoplePass[usPplIdx].usOldPosX);
					shYDist = (short)(astrPeoplePass[usPplIdx].usActPosY - astrPeoplePass[usPplIdx].usOldPosY);
					// Estimate
					if (astrPeoplePass[usPplIdx].shEstFrames == 0)
					{
						//astrPeoplePass[usPplIdx].usEstPos.X = astrPeoplePass[usPplIdx].usActPos.X + shXDist;
						//astrPeoplePass[usPplIdx].usEstPos.Y = astrPeoplePass[usPplIdx].usActPos.Y + shYDist;
						astrPeoplePass[usPplIdx].usEstPosX = astrPeoplePass[usPplIdx].usActPosX;
						astrPeoplePass[usPplIdx].usEstPosY = astrPeoplePass[usPplIdx].usActPosY;
					}
					else
					{
						//astrPeoplePass[usPplIdx].usEstPos.X = astrPeoplePass[usPplIdx].usActPos.X + (shXDist / 8);
						//astrPeoplePass[usPplIdx].usEstPos.Y = astrPeoplePass[usPplIdx].usActPos.Y + (shYDist / 8);
						astrPeoplePass[usPplIdx].usEstPosX = astrPeoplePass[usPplIdx].usActPosX;
						astrPeoplePass[usPplIdx].usEstPosY = astrPeoplePass[usPplIdx].usActPosY;
					}

					// Check if the person is maybe hidden from another person.
					usBreakIdx2 = 0;
					astrPeoplePass[usPplIdx].bHided = FALSE;
					for (usPplIdx2 = 0; usPplIdx2 < MAX_PEOPLE_IN_IMG; usPplIdx2++)
					{
						if (astrPeoplePass[usPplIdx2].ucID != 0)
						{
							usBreakIdx2++;
							if (usPplIdx != usPplIdx2)
							{
								ulDistance = calculateDistance((short)astrPeoplePass[usPplIdx2].usActPosX, (short)astrPeoplePass[usPplIdx2].usActPosY, (short)astrPeoplePass[usPplIdx].usEstPosX, (short)astrPeoplePass[usPplIdx].usEstPosY);
								if(ulDistance < ulDistRadiusThresh * 3 && astrPeoplePass[usPplIdx2].bHided == FALSE)
								{
									// Give the hided person the chance to be found again after he is visible again before deleting
									astrPeoplePass[usPplIdx].bHided = TRUE;
									break;
								}
							}
							if (usBreakIdx2 >= g_ucPeopleInMemory)
							{
								break;
							}
						}
					}
					// Check if the position is still on a position with object pixels
					usPosInDetectionArray = (USHORT)( (astrPeoplePass[usPplIdx].usActPosX >> 8) + ((astrPeoplePass[usPplIdx].usActPosY >> 8) * IMG_SZ_X) );
					if( aucDetectImg[usPosInDetectionArray] == 0 && astrPeoplePass[usPplIdx].bHided == FALSE )
					{
						astrPeoplePass[usPplIdx].shEstFrames++;
					}

					if (astrPeoplePass[usPplIdx].shEstFrames >= usEstFrameThreshold)
					{
						ausMatchedPeople[usPplIdx] = (USHORT)perhapsLeaved;
					}
					else
					{
						ausMatchedPeople[usPplIdx] = (USHORT)finished;
					}
				}
			}
			
			if(ausMatchedPeople[usPplIdx] != (USHORT)finished)
			{
				// This part should avoid that detected persons because of noise will stay when they only where detected in one frame.
				// Set estimation counter
				astrPeoplePass[usPplIdx].shEstFrames++;
				if (astrPeoplePass[usPplIdx].shEstFrames >= usEstFrameThreshold)
				{
					ausMatchedPeople[usPplIdx] = (USHORT)perhapsLeaved;
				}
			}
			
			usBreakIdx++;
			if (usBreakIdx >= g_ucPeopleInMemory)
			{
				break;
			}
		}
	}
}

static void deletePersons( UCHAR ucMaxPossiblePeople, USHORT* ausMatchedPeople, peopleStruct_t* astrPeoplePass )
{
	USHORT usPplIdx = 0;
	// Delete persons who are outside the image
	for( usPplIdx = 0; usPplIdx < ucMaxPossiblePeople; usPplIdx++ )
	{
		if ( astrPeoplePass[usPplIdx].ucID != 0 && ausMatchedPeople[usPplIdx] == perhapsLeaved  )
		{
			astrPeoplePass[usPplIdx].ucID = 0;
			astrPeoplePass[usPplIdx].usActPosX = 0;
			astrPeoplePass[usPplIdx].usActPosY = 0;
			astrPeoplePass[usPplIdx].usOldPosX = 0;
			astrPeoplePass[usPplIdx].usOldPosY = 0;
			astrPeoplePass[usPplIdx].usEstPosX = 0;
			astrPeoplePass[usPplIdx].usEstPosY = 0;
			astrPeoplePass[usPplIdx].counted = FALSE;
			astrPeoplePass[usPplIdx].bNewPerson = FALSE;
			astrPeoplePass[usPplIdx].shEstFrames = 0;
			astrPeoplePass[usPplIdx].bValidPerson = FALSE;
			astrPeoplePass[usPplIdx].shTrackCounter = 0;
			astrPeoplePass[usPplIdx].bHided = FALSE;
			astrPeoplePass[usPplIdx].direction = 0;
			for (int i = 0; i < 3; i++)
			{
				astrPeoplePass[usPplIdx].strFittingHeatSpots[i].distance = ULONG_MAX_VAL;
				astrPeoplePass[usPplIdx].strFittingHeatSpots[i].ucHeatSpotId = UCHAR_MAX_VAL;
			}
		}
	}
}

static UCHAR getTrackedPersonNumber( peopleStruct_t* astrPeoplePass )
{
	USHORT usPplIdx = 0;
	UCHAR ucReturnValue = 0;
	UCHAR ucPersonsInMemory = 0;
	UCHAR ucValidPersonsInMemory = 0;
	
	for (usPplIdx = 0; usPplIdx < MAX_PEOPLE_IN_IMG; usPplIdx++)
	{
		if ( (astrPeoplePass[usPplIdx].ucID != 0) )
		{
			ucPersonsInMemory++;
		}
		if ((astrPeoplePass[usPplIdx].ucID != 0) && (astrPeoplePass[usPplIdx].bValidPerson == TRUE))
		{
			ucValidPersonsInMemory++;
		}
	}
	g_ucPeopleInMemory = ucPersonsInMemory;
	
	return ucValidPersonsInMemory;
}


//*****************************************************************************
//
//! \brief This function uses the information of the tracked persons together with counting lines to increase a counter if a person cross one of that lines. It is possble to feed this algorithm with more than one line. A hysteresis function is also included so that if a persons stands on a line the counter will not be increased serveral times in a row.
//! \param [in]	strCountline Counting lines.
//! \param [in]	ucNumberOfLines Total number of lines.
//! \param [in]	ucPeopleAmount Amount of detected persons. You will get this number from the vAMG_PUB_PC_PeopleTracking function.
//! \param [in]	g_astrPeoplePass Information about the tracked persons.
//! \param [out]	output Its an array consisting of two values. the first is the counter value for persons who crossed the lines in one direction. The other value is the counter for persons who crossed the lines in the other direction.
//
//*****************************************************************************
void vAMG_PUB_PT_PeopleCounting( line_t* strCountline, UCHAR ucNumberOfLines, UCHAR ucPeopleAmount, peopleStruct_t* g_astrPeoplePass, short* output )
{
	short n = 0;	
	
	for( n = 0; n < ucPeopleAmount; n++ )
	{
		if( g_astrPeoplePass[n].ucID != 0 && g_astrPeoplePass[n].bValidPerson == TRUE)
		{
			// If the person is already counted, use a hysteresis function.
			// It is realized with a least distance calculation between the line and the actual person position
			if (g_astrPeoplePass[n].counted)
			{
				actualizeCounterWithHysteresis( n, (peopleStruct_t*)g_astrPeoplePass, ucNumberOfLines, (line_t*)strCountline, (short*)output );
			}
			else
			{
				actualizeCounter( n, (peopleStruct_t*)g_astrPeoplePass, ucNumberOfLines, (line_t*)strCountline, (short*)output );
			}
		}
	}
}

static void actualizeCounterWithHysteresis( short personIndex, peopleStruct_t* g_astrPeoplePass, UCHAR ucNumberOfLines, line_t* strCountline, short* output )
{
	short i = 0;
	short shA, shB, shC, shD;
	long long llDot, llSqLen, llResult, llDet;
	long long llDetSave;
	unsigned long long ullLeastDistance = ULONGLONG_MAX_VAL;
	float flParam;
	long lDx, lDy, lXx, lYy;
	
	for ( i = 0; i < ucNumberOfLines; i++ )
	{
		shA = g_astrPeoplePass[personIndex].usActPosX - strCountline[i].start.X;
		shB = g_astrPeoplePass[personIndex].usActPosY - strCountline[i].start.Y;
		shC = strCountline[i].end.X - strCountline[i].start.X;
		shD = strCountline[i].end.Y - strCountline[i].start.Y;
		llDot = shA*shC + shB*shD;
		llSqLen = shC*shC + shD*shD;
		flParam = (float)llDot / llSqLen;
		
		// Check if the actual person position is parallel to the line
		if(flParam > 0 && flParam <= 1)
		{
			lXx = strCountline[i].start.X + flParam*shC;
			lYy = strCountline[i].start.Y + flParam*shD;
			lDx = g_astrPeoplePass[personIndex].usActPosX - lXx;
			lDy = g_astrPeoplePass[personIndex].usActPosY - lYy;
			llResult = lDx*lDx + lDy*lDy;
			llDet = shA*shD - shB*shC;
			if (llResult < ullLeastDistance)
			{
				ullLeastDistance = llResult;
				llDetSave = llDet;
			}
		}
	}
	// Check if persons position is far away enough to clear the counted flag
	if ( ullLeastDistance >= 100000u )
	{
		// Count if person changed the direction within the hysteresis and reached the minimum distance to the line
		if ( llDetSave > 0 && g_astrPeoplePass[personIndex].direction == 1 )
		{
			output[1]++;
			g_astrPeoplePass[personIndex].counted = FALSE;
		}
		// Count if person changed the direction within the hysteresis and reached the minimum distance to the line
		else if ( llDetSave < 0 && g_astrPeoplePass[personIndex].direction == 0 )
		{
			output[0]++;
			g_astrPeoplePass[personIndex].counted = FALSE;
		}
		// Clear the flag if the person is far away enough from the counting line
		else
		{
			g_astrPeoplePass[personIndex].counted = FALSE;
		}
	}
}

static void actualizeCounter( short personIndex, peopleStruct_t* g_astrPeoplePass, UCHAR ucNumberOfLines, line_t* strCountline, short* output )
{
	long long llL1[3], llL2[3], llS[3];
	long lSch[2];
	short i = 0;
	short shHighX = 0;
	short shHighY = 0;
	short shLowX = 0;
	short shLowY = 0;
	short shHighPeopleX = 0;
	short shHighPeopleY = 0;
	short shLowPeopleX = 0;
	short shLowPeopleY = 0;
	
	for ( i = 0; i < ucNumberOfLines; i++ )
	{
		// Save the boundaries of the line
		shHighX = ( strCountline[i].start.X < strCountline[i].end.X ) ? strCountline[i].end.X : strCountline[i].start.X;
		shHighY = ( strCountline[i].start.Y < strCountline[i].end.Y ) ? strCountline[i].end.Y : strCountline[i].start.Y;
		shLowX = ( strCountline[i].start.X < strCountline[i].end.X ) ? strCountline[i].start.X : strCountline[i].end.X;
		shLowY = ( strCountline[i].start.Y < strCountline[i].end.Y ) ? strCountline[i].start.Y : strCountline[i].end.Y;
		
		// Calculate cross product of count vector
		llL1[0] = strCountline[i].start.Y - strCountline[i].end.Y;
		llL1[1] = strCountline[i].end.X - strCountline[i].start.X;
		llL1[2] = strCountline[i].start.X * strCountline[i].end.Y - strCountline[i].start.Y * strCountline[i].end.X;
		
		// Calculate cross product of motion vector
		llL2[0] = g_astrPeoplePass[personIndex].usActPosY - g_astrPeoplePass[personIndex].usOldPosY;
		llL2[1] = g_astrPeoplePass[personIndex].usOldPosX - g_astrPeoplePass[personIndex].usActPosX;
		llL2[2] = g_astrPeoplePass[personIndex].usActPosX * g_astrPeoplePass[personIndex].usOldPosY - g_astrPeoplePass[personIndex].usActPosY * g_astrPeoplePass[personIndex].usOldPosX;
		
		llS[0] = llL1[1] * llL2[2] - llL1[2] * llL2[1];
		llS[1] = llL1[2] * llL2[0] - llL1[0] * llL2[2];
		llS[2] = llL1[0] * llL2[1] - llL1[1] * llL2[0];
		// Calculation of intersection between motion and count vector
		lSch[0] = (double)llS[0] / (double)llS[2];
		lSch[1] = (double)llS[1] / (double)llS[2];
		// Prove if intersection is within motion and count vector boundaries
		if( lSch[0] >= shLowX && lSch[0] <= shHighX && lSch[1] >= shLowY && lSch[1] <= shHighY )
		{
			shHighPeopleX = ( g_astrPeoplePass[personIndex].usActPosX < g_astrPeoplePass[personIndex].usOldPosX ) ? g_astrPeoplePass[personIndex].usOldPosX : g_astrPeoplePass[personIndex].usActPosX;
			shHighPeopleY = ( g_astrPeoplePass[personIndex].usActPosY < g_astrPeoplePass[personIndex].usOldPosY ) ? g_astrPeoplePass[personIndex].usOldPosY : g_astrPeoplePass[personIndex].usActPosY;
			shLowPeopleX = ( g_astrPeoplePass[personIndex].usActPosX < g_astrPeoplePass[personIndex].usOldPosX ) ? g_astrPeoplePass[personIndex].usActPosX : g_astrPeoplePass[personIndex].usOldPosX;
			shLowPeopleY = ( g_astrPeoplePass[personIndex].usActPosY < g_astrPeoplePass[personIndex].usOldPosY ) ? g_astrPeoplePass[personIndex].usActPosY : g_astrPeoplePass[personIndex].usOldPosY;
			
			if( lSch[0] >= shLowPeopleX && lSch[0] <= shHighPeopleX && lSch[1] >= shLowPeopleY && lSch[1] <= shHighPeopleY )
			{
				// Prove in which direction the count vector is crossed by the motion vector
				if( llS[2] < 0 )
				{
					output[0]++;
					//abCountFlag[n] = true;
					g_astrPeoplePass[personIndex].counted = TRUE;
					g_astrPeoplePass[personIndex].direction = 1;
				}
				else
				{
					output[1]++;
					//abCountFlag[n] = true;
					g_astrPeoplePass[personIndex].counted = TRUE;
					g_astrPeoplePass[personIndex].direction = 0;
				}
			}
		}
	}
}

//*****************************************************************************
//
//! \brief Creates the difference image between background and actual image.
//! \param [in] usSize Image size.
//! \param [in] pshInImg1 Image 1.
//! \param [in] pshInImg2 Image 2.
//! \param [out] pshOutImg Difference image between image 1 and 2.
//
//*****************************************************************************
void vAMG_PUB_PT_CalcDiffImage( USHORT usSize, short* pshInImg1, short* pshInImg2, short* pshOutImg )
{
	USHORT usImg = 0;

	for( usImg = 0; usImg < usSize; usImg++ )
	{
		pshOutImg[usImg] = pshInImg1[usImg] - pshInImg2[usImg];
	}
}

//*****************************************************************************
//
//! \brief Creates the binary image in which objects will be labeled.
//! \param [in] usSize Image size.
//! \param [in] pshInImg Difference image between background and actual image.
//! \param [in] ucMark Label which should be used for detected objects.
//! \param [out] pshOutImg Binary image with labeled object pixels.
//
//*****************************************************************************
void vAMG_PUB_PT_CalcDetectImage( USHORT usSize, short* pshInImg, UCHAR ucMark, UCHAR* pucOutImg )
{
	USHORT usImg = 0;

	for( usImg = 0; usImg < usSize; usImg++)
	{
		//if(shTh <= g_ashDiffTemp[usImg])
		if (g_thresholdArray[usImg] <= pshInImg[usImg])
		{
			pucOutImg[usImg] = ucMark;
		}
		//else if(-shTh >= g_ashDiffTemp[usImg])
		//{
		//    g_aucDetectImg[usImg] = (byte)(ucMark - 1);
		//}
		else
		{
			pucOutImg[usImg] = 0;
		}
	}
}

//*****************************************************************************
//
//! \brief Interpolates the raw image from 8x8 pixels to 15x15 pixels.
//! \param [in] pshInImg 8x8 input image.
//! \param [out] pshOutImg 15x15 output image.
//
//! \return Returns false if input image and output image is the same pointer.
//
//*****************************************************************************
BOOL bAMG_PUB_PT_Interpolation( short* pshInImg, short* pshOutImg )
{
	if (g_eInterpolationType == linear)
	{
		return bAMG_PUB_PT_LinearInterpolationSQ15( (short*)pshInImg, (short*)pshOutImg );
	}
	else if (g_eInterpolationType == tangential)
	{
		return bAMG_PUB_PT_SpecialInterpolationSQ15( (short*)pshInImg, (short*)pshOutImg );
	}
	return FALSE;
}

BOOL bAMG_PUB_PT_LinearInterpolationSQ15( short* pshInImg, short* pshOutImg )
{
	const UCHAR c_ucImgWidth  = 15;
	const UCHAR c_ucImgHeight = 15;
	BOOL bRet = FALSE;

	if( pshInImg != pshOutImg )
	{
		UCHAR ucX = 0;
		UCHAR ucY = 0;
		for( ucY = 0; ucY < c_ucImgHeight; ucY += 2 )
		{
			for( ucX = 0; ucX < c_ucImgWidth; ucX += 2 )
			{
				UCHAR ucSnr = ucX / 2 + ucY / 2 * SNR_SZ_X;
				UCHAR ucImg = ucX + ucY * c_ucImgWidth;
				pshOutImg[ucImg] = pshInImg[ucSnr];
			}
			for( ucX = 1; ucX < c_ucImgWidth; ucX += 2 )
			{
				UCHAR ucImg = ucX + ucY * c_ucImgWidth;
				pshOutImg[ucImg] = ( pshOutImg[ucImg-1] + pshOutImg[ucImg+1] ) / 2;
			}
		}
		for( ucY = 1; ucY < c_ucImgHeight; ucY += 2 )
		{
			for( ucX = 0; ucX < c_ucImgWidth; ucX++ )
			{
				UCHAR ucImg = ucX + ucY * c_ucImgWidth;
				pshOutImg[ucImg] = ( pshOutImg[ucImg-c_ucImgWidth] + pshOutImg[ucImg+c_ucImgWidth] ) / 2;
			}
		}

		bRet = TRUE;
	}

	return( bRet );
}

BOOL bAMG_PUB_PT_SpecialInterpolationSQ15( short* pshInImg, short* pshOutImg )
{
	const UCHAR c_ucImgWidth = 8;
	const UCHAR c_ucImgHeight = 8;
	int ucY = 0;
	int ucX = 0;
	short shSlewRateLeft = 0;
	short shSlewRateRight = 0;
	short shSlewRateAbove = 0;
	short shSlewRateBelow = 0;
	short shTangent = 0;
	short shInterpolatedPoint1 = 0;
	short shInterpolatedPoint2 = 0;
	short shAvgTangent1 = 0;
	//int[] g_ashSnrAveTemp = new int[100];

	// It could be, that there are problems on higher temperatures because of the Data type.
	// Grid-EYE value are 12 Bit with the shift we have 20 Bit which doesn't fit into short.
	// Nevertheless, it should work because the values shSlewRateLeft and shSlewRateRight are normally very low values.
	short shDeltaTFactor = shAMG_PUB_PT_ConvFtoS( g_flTangentialInterpolationFactor );

	for (ucY = 0; ucY < c_ucImgHeight; ucY++)
	{
		for (ucX = 0; ucX < c_ucImgWidth - 1; ucX++)
		{
			// Known values
			pshOutImg[ucY * 2 * (c_ucImgWidth * 2 - 1) + ucX * 2] = pshInImg[ucY * c_ucImgWidth + ucX];
			if (ucX == 0)
			{
				// Left border
				shInterpolatedPoint1 = (short)(pshInImg[ucY * c_ucImgWidth + ucX]);

				shSlewRateLeft = (short)(pshInImg[ucY * c_ucImgWidth + ucX + 1] - pshInImg[ucY * c_ucImgWidth + ucX]);
				shSlewRateRight = (short)(pshInImg[ucY * c_ucImgWidth + ucX + 2] - pshInImg[ucY * c_ucImgWidth + ucX + 1]);
				// Shifting 9 bit because 1 bit for averaging + 8 bit for skaling with shDeltaFactor whch is q8-fixpoint number
				shTangent = (short)(((int)shDeltaTFactor * (shSlewRateLeft + shSlewRateRight)) >> 9);
				shInterpolatedPoint2 = (short)(pshInImg[ucY * c_ucImgWidth + ucX + 1] - shTangent);

				pshOutImg[ucY * 2 * (c_ucImgWidth * 2 - 1) + ucX * 2 + 1] = (short)((shInterpolatedPoint1 + shInterpolatedPoint2) >> 1);
			}
			else if (ucX == (c_ucImgWidth - 2))
			{
				// Right border
				shSlewRateLeft = (short)(pshInImg[ucY * c_ucImgWidth + ucX] - pshInImg[ucY * c_ucImgWidth + ucX - 1]);
				shSlewRateRight = (short)(pshInImg[ucY * c_ucImgWidth + ucX + 1] - pshInImg[ucY * c_ucImgWidth + ucX]);
				// Shifting 9 bit because 1 bit for averaging + 8 bit for skaling with shDeltaFactor whch is q8-fixpoint number
				shTangent = (short)(((int)shDeltaTFactor * (shSlewRateLeft + shSlewRateRight)) >> 9);
				shInterpolatedPoint1 = (short)(pshInImg[ucY * c_ucImgWidth + ucX] + shTangent);

				shTangent = 0;
				shInterpolatedPoint2 = (short)(pshInImg[ucY * c_ucImgWidth + ucX + 1] + shTangent);

				pshOutImg[ucY * 2 * (c_ucImgWidth * 2 - 1) + ucX * 2 + 1] = (short)((shInterpolatedPoint1 + shInterpolatedPoint2) >> 1);
			}
			else
			{
				// Middle part
				shSlewRateLeft = (short)(pshInImg[ucY * c_ucImgWidth + ucX] - pshInImg[ucY * c_ucImgWidth + ucX - 1]);
				shSlewRateRight = (short)(pshInImg[ucY * c_ucImgWidth + ucX + 1] - pshInImg[ucY * c_ucImgWidth + ucX]);
				// Shifting 9 bit because 1 bit for averaging + 8 bit for skaling with shDeltaFactor whch is q8-fixpoint number
				shTangent = (short)(((int)shDeltaTFactor * (shSlewRateLeft + shSlewRateRight)) >> 9);
				shInterpolatedPoint1 = (short)(pshInImg[ucY * c_ucImgWidth + ucX] + shTangent);

				shSlewRateLeft = shSlewRateRight;
				shSlewRateRight = (short)(pshInImg[ucY * c_ucImgWidth + ucX + 2] - pshInImg[ucY * c_ucImgWidth + ucX + 1]);
				// Shifting 9 bit because 1 bit for averaging + 8 bit for skaling with shDeltaFactor whch is q8-fixpoint number
				shTangent = (short)( ((int)shDeltaTFactor * (shSlewRateLeft + shSlewRateRight)) >> 9);
				shInterpolatedPoint2 = (short)(pshInImg[ucY * c_ucImgWidth + ucX + 1] - shTangent);

				pshOutImg[ucY * 2 * (c_ucImgWidth * 2 - 1) + ucX * 2 + 1] = (short)((shInterpolatedPoint1 + shInterpolatedPoint2) >> 1);
			}
		}
		pshOutImg[ucY * 2 * (c_ucImgWidth * 2 - 1) + ucX * 2] = pshInImg[ucY * c_ucImgWidth + ucX];
	}
	for (ucY = 0; ucY < c_ucImgHeight - 1; ucY++)
	{
		for (ucX = 0; ucX < c_ucImgWidth * 2 - 1; ucX++)
		{
			if (ucY == 0)
			{
				shTangent = 0;
				shInterpolatedPoint1 = (short)(pshOutImg[ucY * 2 * (c_ucImgWidth * 2 - 1) + ucX] + shTangent);

				shSlewRateAbove = (short)(pshOutImg[(ucY + 1) * 2 * (c_ucImgWidth * 2 - 1) + ucX] - pshOutImg[ucY * 2 * (c_ucImgWidth * 2 - 1) + ucX]);
				shSlewRateBelow = (short)(pshOutImg[(ucY + 2) * 2 * (c_ucImgWidth * 2 - 1) + ucX] - pshOutImg[(ucY + 1) * 2 * (c_ucImgWidth * 2 - 1) + ucX]);
				// Shifting 9 bit because 1 bit for averaging + 8 bit for skaling with shDeltaFactor whch is q8-fixpoint number
				shTangent = (short)(((int)shDeltaTFactor * (shSlewRateAbove + shSlewRateBelow)) >> 9);
				shInterpolatedPoint2 = (short)(pshOutImg[(ucY + 1) * 2 * (c_ucImgWidth * 2 - 1) + ucX] - shTangent);

				pshOutImg[ucY * 2 * (c_ucImgWidth * 2 - 1) + (c_ucImgWidth * 2 - 1) + ucX] = (short)((shInterpolatedPoint1 + shInterpolatedPoint2) >> 1);
			}
			else if (ucY == (c_ucImgHeight - 2))
			{
				shSlewRateAbove = (short)(pshOutImg[ucY * 2 * (c_ucImgWidth * 2 - 1) + ucX] - pshOutImg[(ucY - 1) * 2 * (c_ucImgWidth * 2 - 1) + ucX]);
				shSlewRateBelow = (short)(pshOutImg[(ucY + 1) * 2 * (c_ucImgWidth * 2 - 1) + ucX] - pshOutImg[ucY * 2 * (c_ucImgWidth * 2 - 1) + ucX]);
				// Shifting 9 bit because 1 bit for averaging + 8 bit for skaling with shDeltaFactor whch is q8-fixpoint number
				shTangent = (short)(((int)shDeltaTFactor * (shSlewRateAbove + shSlewRateBelow)) >> 9);
				shInterpolatedPoint1 = (short)(pshOutImg[ucY * 2 * (c_ucImgWidth * 2 - 1) + ucX] + shTangent);

				shTangent = 0;
				shInterpolatedPoint2 = (short)(pshOutImg[(ucY + 1) * 2 * (c_ucImgWidth * 2 - 1) + ucX] + shTangent);

				pshOutImg[ucY * 2 * (c_ucImgWidth * 2 - 1) + (c_ucImgWidth * 2 - 1) + ucX] = (short)((shInterpolatedPoint1 + shInterpolatedPoint2) >> 1);
			}
			else
			{
				shSlewRateAbove = (short)(pshOutImg[ucY * 2 * (c_ucImgWidth * 2 - 1) + ucX] - pshOutImg[(ucY - 1) * 2 * (c_ucImgWidth * 2 - 1) + ucX]);
				shSlewRateBelow = (short)(pshOutImg[(ucY + 1) * 2 * (c_ucImgWidth * 2 - 1) + ucX] - pshOutImg[ucY * 2 * (c_ucImgWidth * 2 - 1) + ucX]);
				// Shifting 9 bit because 1 bit for averaging + 8 bit for skaling with shDeltaFactor whch is q8-fixpoint number
				shTangent = (short)( ((int)shDeltaTFactor * (shSlewRateAbove + shSlewRateBelow)) >> 9);
				shInterpolatedPoint1 = (short)(pshOutImg[ucY * 2 * (c_ucImgWidth * 2 - 1) + ucX] + shTangent);

				shSlewRateAbove = shSlewRateBelow;
				shSlewRateBelow = (short)(pshOutImg[(ucY + 2) * 2 * (c_ucImgWidth * 2 - 1) + ucX] - pshOutImg[(ucY + 1) * 2 * (c_ucImgWidth * 2 - 1) + ucX]);
				// Shifting 9 bit because 1 bit for averaging + 8 bit for skaling with shDeltaFactor whch is q8-fixpoint number
				shTangent = (short)(((int)shDeltaTFactor * (shSlewRateAbove + shSlewRateBelow)) >> 9);
				shInterpolatedPoint2 = (short)(pshOutImg[(ucY + 1) * 2 * (c_ucImgWidth * 2 - 1) + ucX] - shTangent);

				pshOutImg[ucY * 2 * (c_ucImgWidth * 2 - 1) + (c_ucImgWidth * 2 - 1) + ucX] = (short)((shInterpolatedPoint1 + shInterpolatedPoint2) >> 1);
			}
		}
	}
	return TRUE;
}

//*****************************************************************************
//
//! \brief Image dilation. Increase objects size.
//! \param [in] ucWidth The image width.
//! \param [in] ucHeight The image height.
//! \param [in] pucInImg The binary image which holds the labels.
//! \param [out] pucOutImg The resulting binary image with increased object sizes.
//
//! \return Returns false if input image and output image is the same pointer.
//
//*****************************************************************************
BOOL bAMG_PUB_PT_ImageDilation1( UCHAR ucWidth, UCHAR ucHeight, UCHAR* pucInImg, UCHAR* pucOutImg )
{
	USHORT	usImg	= 0;
	USHORT	usSize	= ucWidth * ucHeight;
	BOOL	bRet	= FALSE;

	if( pucInImg != pucOutImg )
	{
		for( usImg = 0; usImg < usSize; usImg++ )
		{
			pucOutImg[usImg] = 0;
		}
		for( usImg = 0; usImg < usSize; usImg++ )
		{
			UCHAR	ucImgX = usImg % ucWidth;
			UCHAR	ucImgY = usImg / ucWidth;

			if( 0 != pucInImg[usImg] )
			{
				pucOutImg[usImg] = 1;
				if( 0 != ucImgX )
				{
					pucOutImg[usImg - 1] = 1;
				}
				if( (ucWidth - 1) != ucImgX )
				{
					pucOutImg[usImg + 1] = 1;
				}
				if( 0 != ucImgY )
				{
					pucOutImg[usImg - ucWidth] = 1;
				}
				if( (ucHeight - 1) != ucImgY )
				{
					pucOutImg[usImg + ucWidth] = 1;
				}
			}
		}

		bRet = TRUE;
	}

	return( bRet );
}

//*****************************************************************************
//
//! \brief Updates the background image.
//! \param [in] usSize The image size in pixel.
//! \param [in] pucImg The binary image which stores the labels.
//! \param [in] pshDiffImg The difference image between background and actual image.
//! \param [out] pshBackImg The background image which should be updated.
//
//! \return Returns false if there is no non object pixel in the actual input image.
//
//*****************************************************************************
BOOL bAMG_PUB_PT_UpdateBackTemp( USHORT usSize, UCHAR* pucImg, short* pshDiffImg, short* pshBackImg )
{
	short shThNonObjPixel = g_shBackgroundIirFilterCoeffNonObjPixel;
	short shThObjPixel = g_shBackgroundIirFilterCoeffObjPixel;
	
	const short c_shMinTh = 0;
	const short c_shMaxTh = 256;
	BOOL	bRet	= FALSE;
	USHORT	usImg	= 0;
	short	shAve	= 0;

	/* Adjust parameter. */
	if( c_shMinTh > shThNonObjPixel )
	{
		shThNonObjPixel = c_shMinTh;
	}
	if( c_shMaxTh < shThNonObjPixel )
	{
		shThNonObjPixel = c_shMaxTh;
	}

	if( FALSE != bAMG_PUB_PT_CalcAveTemp( usSize, 0, pucImg, pshDiffImg, &shAve ) )
	{
		bRet = TRUE;
		for( usImg = 0; usImg < usSize; usImg++ )
		{
			short shTemp = 0;
			if( 0 == pucImg[usImg] )
			{
				shTemp = (short)( (long)shThNonObjPixel * pshDiffImg[usImg] / c_shMaxTh );
			}
			else
			{
				shTemp = (short)( (long)shThObjPixel * shAve / c_shMaxTh );
			}
			pshBackImg[usImg] += shTemp;
		}
	}

	return( bRet );
}




BOOL bAMG_PUB_PT_IsImageFilledWithObjects( UCHAR* detectionImg )
{
	BOOL retValue = TRUE;

	for(int i = 0; i < IMG_SZ; i++)
	{
		if(detectionImg[i] == 0)
		{
			return FALSE;
		}
	}

	return retValue;
}

BOOL bAMG_PUB_PT_IsImageDifferenzHigh( short* bg, short* img, int maxDifference )
{
	BOOL retValue = FALSE;

	if( Abs( getAverage(bg) - getAverage(img) ) > maxDifference  )
	{
		errorFrameCounter++;
	}
	else
	{
		errorFrameCounter = 0;
	}

	if(errorFrameCounter > maxErrorFrames)
	{
		retValue = TRUE;
	}

	return retValue;
}

USHORT getAverage(short* inArray)
{
	int sum = 0;

	for(int i = 0; i < IMG_SZ; i++)
	{
		sum += inArray[i];
	}

	return sum / IMG_SZ;
}
//*****************************************************************************
//
//! @}
//
//*****************************************************************************
//*****************************************************************************
//
//! @}
//
//*****************************************************************************
