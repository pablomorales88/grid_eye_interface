/*
 * Grid_Eye_PTaC_Backend.c
 *
 * Created: 31.01.2017 08:54:25
 *  Author: Eric Paljan
 */ 
#include "grid_eye_config.h"
#include "Grid_Eye_People_Tracking_and_Counting.h"

#define DOXY
#include "Arduino.h"
static short	g_shChosenAverageFilterLength = 3;

static ULONG	g_ulFrameNum;
static short	g_a2shRawTemp  [AVRG_ARRAY_SIZE][SNR_SZ];
static short	g_ashSnrAveTemp[SNR_SZ];
static short	g_ashAveTemp   [IMG_SZ];
static short	g_ashBackTemp  [IMG_SZ];
static short	g_ashDiffTemp  [IMG_SZ];
static short	g_aucObjectImg [IMG_SZ];
static USHORT	g_ausWork      [IMG_SZ];


static line_t g_astrCountingLines[3];
static UCHAR g_ucNumberOfLines = 0;
static coh_t g_astrCOH[MAX_PEOPLE_IN_IMG];

static USHORT imageFullCounter = 0;

BOOL bAMG_PUB_PC_SetCountlines( line_t* astrLines );
BOOL bAMG_PUB_SMP_People_Tracking_and_Counting( UCHAR* g_aucRawI2C, peopleTrackandCountOutput_t* peopleTrackandCountOut );

static void executeFilterFunctions( void );
static void initBackgroundTemp( void );
static void calculateCOHPositions( UCHAR ucDetectNum, UCHAR* g_aucDetectImg, short* g_ashDiffTemp, coh_t* astrCOH );


//*****************************************************************************
//
//! \addtogroup PTaC GridEye People Tracking and Counting
//! \brief Component for the realization of a people detecting, tracking and counting algorithm
//! @{
//
//*****************************************************************************
//*****************************************************************************
//
//! \addtogroup PTaC_Mainfunc Main function
//! \brief The Main function reference for the People Tracking and Counting algorithm.
//! @image html TrackMeFlowDiagramm.svg width=15cm
//! @{
//
//*****************************************************************************
//*****************************************************************************
//
//! \brief Resets the TrackMe algorithm
//! \param [in / out] peopleTrackandCountOut The main output structure will be cleared also.
//
//*****************************************************************************
void vAMG_PUB_PT_Reset_Algorithm( peopleTrackandCountOutput_t* peopleTrackandCountOut )
{
	int i = 0;
	int j = 0;
	
	// Reset internal variables
	g_ulFrameNum = 0;
	for (i = 0; i < AVRG_ARRAY_SIZE; i++)
	{
		for (j = 0; j < SNR_SZ; j++)
		{
			g_a2shRawTemp[i][j] = 0;
		}
	}
	for (i = 0; i < SNR_SZ; i++)
	{
		g_ashSnrAveTemp[i] = 0;
		g_ashAveTemp   [i] = 0;
		g_ashBackTemp  [i] = 0;
		g_ashDiffTemp  [i] = 0;
		g_aucObjectImg [i] = 0;
	 	g_ausWork      [i] = 0;
	}
	g_astrCountingLines[0].start.X = 0;
	g_astrCountingLines[0].start.Y = 0;
	g_astrCountingLines[0].end.X = 0;
	g_astrCountingLines[0].end.Y = 0;
	g_astrCountingLines[1].start.X = 0;
	g_astrCountingLines[1].start.Y = 0;
	g_astrCountingLines[1].end.X = 0;
	g_astrCountingLines[1].end.Y = 0;
	g_astrCountingLines[2].start.X = 0;
	g_astrCountingLines[2].start.Y = 0;
	g_astrCountingLines[2].end.X = 0;
	g_astrCountingLines[2].end.Y = 0;
	g_ucNumberOfLines = 0;
	for (i = 0; i < MAX_PEOPLE_IN_IMG; i++)
	{
		g_astrCOH[i].ashCOH[0] = 0;
		g_astrCOH[i].ashCOH[1] = 0;
	}
	
	// Reset external variables
	for (i = 0; i < MAX_PEOPLE_IN_IMG; i++)
	{
		peopleTrackandCountOut->g_astrPeoplePass[i].bHided = FALSE;
		peopleTrackandCountOut->g_astrPeoplePass[i].bNewPerson = FALSE;
		peopleTrackandCountOut->g_astrPeoplePass[i].bValidPerson = FALSE;
		peopleTrackandCountOut->g_astrPeoplePass[i].counted = FALSE;
		peopleTrackandCountOut->g_astrPeoplePass[i].direction = 0;
		peopleTrackandCountOut->g_astrPeoplePass[i].shEstFrames = 0;
		peopleTrackandCountOut->g_astrPeoplePass[i].shTrackCounter = 0;
		peopleTrackandCountOut->g_astrPeoplePass[i].strFittingHeatSpots[0].distance = ULONG_MAX_VAL;
		peopleTrackandCountOut->g_astrPeoplePass[i].strFittingHeatSpots[0].ucHeatSpotId = UCHAR_MAX_VAL;
		peopleTrackandCountOut->g_astrPeoplePass[i].strFittingHeatSpots[1].distance = ULONG_MAX_VAL;
		peopleTrackandCountOut->g_astrPeoplePass[i].strFittingHeatSpots[1].ucHeatSpotId = UCHAR_MAX_VAL;
		peopleTrackandCountOut->g_astrPeoplePass[i].strFittingHeatSpots[2].distance = ULONG_MAX_VAL;
		peopleTrackandCountOut->g_astrPeoplePass[i].strFittingHeatSpots[2].ucHeatSpotId = UCHAR_MAX_VAL;
		peopleTrackandCountOut->g_astrPeoplePass[i].ucID = 0;
		peopleTrackandCountOut->g_astrPeoplePass[i].usActPosX = 0;
		peopleTrackandCountOut->g_astrPeoplePass[i].usActPosY = 0;
		peopleTrackandCountOut->g_astrPeoplePass[i].usEstPosX = 0;
		peopleTrackandCountOut->g_astrPeoplePass[i].usEstPosY = 0;
		peopleTrackandCountOut->g_astrPeoplePass[i].usOldPosX = 0;
		peopleTrackandCountOut->g_astrPeoplePass[i].usOldPosY = 0;
	}
	for (i = 0; i < IMG_SZ; i++)
	{
		peopleTrackandCountOut->g_aucDetectImg[i] = 0;
	}
	peopleTrackandCountOut->g_shPCcounter[0] = 0;
	peopleTrackandCountOut->g_shPCcounter[1] = 0;
	peopleTrackandCountOut->g_ucPCpeopleNum = 0;
	vAMG_PUB_PT_Reset();
}

//*****************************************************************************
//
//! \brief Interface function for setting the TrackMe algorithm parameters.
//! \param [in] strAlgoParams Input structure with the parameter configuration which should be set.
//
//*****************************************************************************
void vAMG_PUB_PC_SetTrackMeParameters( strAlgorithmParameters strAlgoParams )
{
	vAMG_PUB_PC_SetParameters( strAlgoParams );
}

//*****************************************************************************
//
//! \brief Interface function for setting the counting lines (3 are allowed).
//! \param [in] astrLines Counting lines. <b>For the input are three lines needed.</b>
//
//! \return Returns true if the counting lines were saved. Returns false if no line is valid. The Number of valid lines will be saved too.
//
//*****************************************************************************
BOOL bAMG_PUB_PC_SetCountlines( line_t* astrLines )
{
	int n = 0;
	for (int i = 0; i < 3; i++ )
	{
		if ( (astrLines[i].start.X != astrLines[i].end.X) || (astrLines[i].start.Y != astrLines[i].end.Y) )
		{
			g_astrCountingLines[n].start.X = astrLines[i].start.X;
			g_astrCountingLines[n].start.Y = astrLines[i].start.Y;
			g_astrCountingLines[n].end.X = astrLines[i].end.X;
			g_astrCountingLines[n].end.Y = astrLines[i].end.Y;
			n++;
		}
	}
	g_ucNumberOfLines = n;
	if (n == 0)
	{
		return FALSE;
	}
	return TRUE;
}


static void executeFilterFunctions()
{
	USHORT	usCnt = 0;
	for( usCnt = 0; usCnt < SNR_SZ; usCnt++ )
	{
		short shAveTemp = shAMG_PUB_PT_CalcAverage( &g_a2shRawTemp[0][usCnt], SNR_SZ, (BOOL*)g_ausWork );
		if( g_shChosenAverageFilterLength == g_ulFrameNum )
		{
			g_ashSnrAveTemp[usCnt] = shAveTemp;
		}
		else
		{
			g_ashSnrAveTemp[usCnt] = shAMG_PUB_PT_CalcIIR( g_ashSnrAveTemp[usCnt], shAveTemp );
		}
	}
}

static void initBackgroundTemp()
{
	USHORT	usCnt = 0;
	for( usCnt = 0; usCnt < IMG_SZ; usCnt++ )
	{
		g_ashBackTemp[usCnt] = g_ashAveTemp[usCnt];
	}
}

static void calculateCOHPositions( UCHAR ucDetectNum, UCHAR* aucDetectImg, short* ashDiffTemp, coh_t* astrCOH )
{
	USHORT	usCnt = 0;
	// Resets the center of heats array
	for( usCnt = 0; usCnt < MAX_PEOPLE_IN_IMG; usCnt++ )
	{
		astrCOH[usCnt].ashCOH[0] = 0;
		astrCOH[usCnt].ashCOH[1] = 0;
	}
	/* Calculate the position of the hot spots. */
	if(ucDetectNum != 0)
	{
		for( usCnt = 0; usCnt < ucDetectNum; usCnt++ )
		{
			bAMG_PUB_PT_CalcCenterTemp( IMG_SZ_X, IMG_SZ_Y, usCnt+1, (UCHAR*)aucDetectImg, (short*)ashDiffTemp, (short*)astrCOH[usCnt].ashCOH );
		}
	}
}
//*****************************************************************************
//
//! \brief Main function for People Tracking and Counting.
//! \param [in] g_aucRawI2C Raw data of the GridEye sensor. The size is 128 Bytes.
//! \param [out] peopleTrackandCountOut Output structure with relevant results of the algorithm.
//
//! \return Returns true if the whole algorithm was executed. Returns false for the initial phase. This phase lasts (1 + TEMP_FRAME_NUM) frames after setup.
//
//*****************************************************************************
BOOL bAMG_PUB_SMP_People_Tracking_and_Counting( UCHAR* g_aucRawI2C, peopleTrackandCountOutput_t* peopleTrackandCountOut )
{
	UCHAR	ucDetectNum = 0;
	BOOL errorReset = FALSE;
	USHORT imageFullCounter = 0;
	
	if( g_ulFrameNum == 0 )
	{
		//vAMG_PUB_PT_SetThresholdArray( 0.45f, 0.4f, 0.35f );
	}
	
	/* Convert temperature register value. */
	vAMG_PUB_PT_ConvTemperature64( (UCHAR*)g_aucRawI2C, (short*)g_a2shRawTemp, SNR_SZ, g_ulFrameNum );
	
	/* Increment number of measurement. */
	g_ulFrameNum++;
	if( g_shChosenAverageFilterLength > g_ulFrameNum )
	{
		return( FALSE );				/* Initial process */
	}

	/* Calculate average. */
	executeFilterFunctions();

	/* Linear interpolation. */
	bAMG_PUB_PT_Interpolation( g_ashSnrAveTemp, g_ashAveTemp );

	/* Initialize background temperature. */
	if( g_shChosenAverageFilterLength == g_ulFrameNum )
	{
		initBackgroundTemp();
		return( FALSE );				/* Initial process */
	}

	/* Object detection. */
	vAMG_PUB_PT_CalcDiffImage	( IMG_SZ, g_ashAveTemp, g_ashBackTemp, g_ashDiffTemp );
	vAMG_PUB_PT_CalcDetectImage	( IMG_SZ, g_ashDiffTemp, DETECT_MARK, peopleTrackandCountOut->g_aucDetectImg );
	vAMG_PUB_PT_CalcObjectImage	( IMG_SZ, g_ashAveTemp, peopleTrackandCountOut->g_aucDetectImg, DETECT_MARK, g_aucObjectImg );
	
	/* Error detection to check if the Background image is destroyed */
	if( bAMG_PUB_PT_IsImageFilledWithObjects( (UCHAR*)peopleTrackandCountOut->g_aucDetectImg ) == TRUE )
	{
		imageFullCounter++;
		if( (bAMG_PUB_PT_IsImageDifferenzHigh( g_ashBackTemp, g_ashAveTemp, shAMG_PUB_PT_ConvFtoS( 2.0f ) ) == TRUE) || (imageFullCounter >= 3000) )
		{
			errorReset = TRUE;
		}
	}
	// Reset if background is not valid anymore
	if (errorReset == TRUE)
	{
		vAMG_PUB_PT_Reset_Algorithm( (peopleTrackandCountOutput_t*)peopleTrackandCountOut );
		imageFullCounter = 0;
		errorReset = FALSE;
	}
	// Continue algorithm if background is still valid
	else
	{
		/* Labeling. */
		ucDetectNum = ucAMG_PUB_PT_CalcPeopleLabeling( IMG_SZ_X, IMG_SZ_Y, DETECT_MARK, MAX_PEOPLE_IN_IMG, g_aucObjectImg, peopleTrackandCountOut->g_aucDetectImg, (BOOL*)g_ausWork );

		/* Calculate the position of the hot spots */
		calculateCOHPositions(  ucDetectNum, (UCHAR*)peopleTrackandCountOut->g_aucDetectImg, (short*)g_ashDiffTemp, (coh_t*)g_astrCOH  );
			
		/* Execute the tracking algorithm */
		peopleTrackandCountOut->g_ucPCpeopleNum = vAMG_PUB_PT_PeopleTracking( IMG_SZ_X, IMG_SZ_Y, g_astrCOH, ucDetectNum, MAX_PEOPLE_IN_IMG, peopleTrackandCountOut->g_astrPeoplePass, peopleTrackandCountOut->g_aucDetectImg );
			
		if (g_ucNumberOfLines > 0)
		{
			/* Execute the counting algorithm */
			vAMG_PUB_PT_PeopleCounting( g_astrCountingLines, g_ucNumberOfLines, MAX_PEOPLE_IN_IMG, peopleTrackandCountOut->g_astrPeoplePass, peopleTrackandCountOut->g_shPCcounter );
		}

		/* Update background temperature. */
		bAMG_PUB_PT_ImageDilation1( IMG_SZ_X, IMG_SZ_Y, peopleTrackandCountOut->g_aucDetectImg, (UCHAR*)g_ausWork );
		bAMG_PUB_PT_UpdateBackTemp( IMG_SZ, (UCHAR*)g_ausWork, g_ashDiffTemp, g_ashBackTemp );
	}
	
	return( TRUE );
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
