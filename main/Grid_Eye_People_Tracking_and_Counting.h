/*
 * Grid_Eye_People_Tracking_and_Counting.h
 *
 * Created: 31.01.2017 08:51:23
 *  Author: Paljan
 */ 


#ifndef GRID_EYE_PEOPLE_TRACKING_AND_COUNTING_H_
#define GRID_EYE_PEOPLE_TRACKING_AND_COUNTING_H_

#include "grid_eye_config.h"


#define	ULONGLONG_MAX_VAL	    ( 18446744073709551615 )		    /* 0xFFFFFFFFFFFFFFFF	*/
#define MAX_PEOPLE_IN_IMG		(15)


//*****************************************************************************
//
//! \addtogroup PTaC GridEye People Tracking and Counting
//! \brief Component for the realization of a people detecting, tracking and counting algorithm
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup PTaC_enum Enumerations
//! \brief Enumerations
//! @{
//
//*****************************************************************************
/*typedef enum eInterpolationType	/// Type of interpolation
{
	tangential,					///< Special interpolation algorithm which is much more realistic than the linear interpolation
	linear						///< Linear interpolation algorithm
} eInterpolationType;
*/
typedef enum eInterpolationType//e_enum  /// Type of interpolation
{
  tangential,         ///< Special interpolation algorithm which is much more realistic than the linear interpolation
  linear            ///< Linear interpolation algorithm
} eInterpolationType_enum;
//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup PTaC_str Structures
//! \brief Input and output structures
//! @{
//
//*****************************************************************************
typedef struct strAlgorithmParameters			/// Structure which holds algorithm parameters. It is used as input structure to change the algorithm behavior.
{
	USHORT usRectangleFilterLength;				///< Amount of frames which areeInterpolationType used to filter each pixel over an average filter.<br>The minimum value is 1.<br>The maximum value is 10.
	USHORT usMedianFilterLength;				///< Additional to the average filter a median filter can be used. For example if the average filter length is 8 and the median filter length is 2, the 2 highest and lowest values will be sorted out, and the average will be calculated with the remaining 4 values.<br>The minimum value is 0.<br>The maximum value is 4.
	float flIirFilterValue;						///< After the average and median filter stage there is an IIR filter stage first order. For example a coefficient of 0.8 means that 80% of the new pixel and 20% of the old pixel value will be used for the new pixel value.<br>The minimum value is 0.<br>The maximum value is 1.
	eInterpolationType_enum eInterpolationType;		///< There are two options for the interpolation:<br>Linear: It�s a basic linear interpolation algorithm to calculate from the 8 x 8 temperature grid a 15 x 15 temperature grid.<br>Tangential: A special interpolation algorithm which is much more realistic then the linear interpolation.
	float flTangentialInterpolationFactor;		///< A factor for controlling the sensitivity of the tangential interpolation algorithm. A value of 0 leads to same results like the linear interpolation.<br>The minimum value is 0.<br>The maximum value is 1.
	float flCornerThreshold;					///< To detect an object, it is necessary that this object contrasts from the background. The three threshold values means, that when a pixel is for example 0.45 �C hotter than the background, it will be detected as object pixel.<br>The three different thresholds are distributed at different places within the image. Please see following picture for the distribution.<br>The minimum value is 0.<br>The maximum value is 1.\htmlonly <style>div.image img[src="distribution.png"]{width:300px;}</style> \endhtmlonly @image html distribution.png
	float flSideThreshold;						///< To detect an object, it is necessary that this object contrasts from the background. The three threshold values means, that when a pixel is for example 0.45 �C hotter than the background, it will be detected as object pixel.<br>The three different thresholds are distributed at different places within the image. Please see following picture for the distribution.<br>The minimum value is 0.<br>The maximum value is 1.\htmlonly <style>div.image img[src="distribution.png"]{width:300px;}</style> \endhtmlonly @image html distribution.png
	float flCenterThreshold;					///< To detect an object, it is necessary that this object contrasts from the background. The three threshold values means, that when a pixel is for example 0.45 �C hotter than the background, it will be detected as object pixel.<br>The three different thresholds are distributed at different places within the image. Please see following picture for the distribution.<br>The minimum value is 0.<br>The maximum value is 1.\htmlonly <style>div.image img[src="distribution.png"]{width:300px;}</style> \endhtmlonly @image html distribution.png
	BOOL bAutomaticPersonParameterControl;		///< In TrackMe there is an automatic adaption algorithm included which makes the system independent from the sensor mounting height. It calculates the standard values for �Person Radius�, �Person Area� and �Person max Speed� by learning from detected persons. This algorithm can be deactivated with setting this flag to 0, so that fixed values can be chosen.
	BOOL bAutomaticParameterCalculation;		///< Basically the �Person Area� and the �Person max Speed� can be calculated from the �Person Radius�. With setting the flag to 0 it is possible to choose fixed values for them independent from the logic relation between them.
	float flPersonRadius;						///< Standard value which is used on different algorithm parts for calculation of distances and areas.<br>The minimum value is 0.<br>The unit is �pixel�
	float flPersonArea;							///< This is a standard value which is used for example for calculating if a hot spot is considered as a person.<br>The minimum value is 0.<br>The unit is �pixel�
	float flPersonMaxSpeed;						///< Standard value which is used to create thresholds for checking distances between persons.<br>The minimum value is 0.<br>The unit is �pixel / frame�
	float flAdaptionCoefficient;				///< The Person Parameters will be adapted over an IIR-Filter first order. The lower the value is, the faster the parameters will adapt to detected persons.<br>The minimum value is 0.<br>The maximum value is 1.
	float flOverlappFactor;						///< If there are two hot spots close to each other it could be, that the calculated person areas overlap each other. The Overlap Factor is the rate it is allowed to overlap. That means a value of for example 0.1 means, that 10% of the area is allowed to share between two persons. If the shared area is above the resulting threshold the second hot spot will not be considered as center for a person.<br>The minimum value is 0.<br>The maximum value is 1.
	float flAreaFactor;							///< With the Area Factor, a threshold is calculated for the minimum area a hot spot has to fill out to be considered as person.<br>The minimum value is 0.<br>The maximum value is 1.
	float flTempFactor;							///< The Temperature Factor is used for the calculation of a threshold between the minimum and maximum temperature value. Every pixel which is below this threshold will not be considered as center point for a person.<br>The minimum value is 0.<br>The maximum value is 1.
	short shMaxMovementDistanceFactor;			///< This factor, together with the �Person max Speed�, is used to calculate thresholds for checking distances between persons.<br>The minimum value is 0.
	short shMinPersonDistFactor;				///< This Factor is used to control the distance two persons have to each other without the assumption that one could hide the other one if there is no second hot spot detected.<br>The minimum value is 0.
	USHORT usMaxEstimatedFrames;				///< The amount of Frames a person will be held without deleting, when there is no fitting hot spot detected.<br>The minimum value is 0.
	float flBackgroundIirFilterCoeffNonObjPixel;///< The background image will be adapted to changing environment temperature. The speed of this adaption is handled by an IIR filter first order. This coefficient is used for background pixels which are not blocked by detected objects.<br>The minimum value is 0.<br>The maximum value is 1.
	float flBackgroundIirFilterCoeffObjPixel;	///< The background image will be adapted to changing environment temperature. The speed of this adaption is handled by an IIR filter first order. This coefficient is used for background pixels which are blocked by detected objects.<br>The minimum value is 0.<br>The maximum value is 1.
}strAlgorithmParameters;

typedef struct strFittingHeatSpot		/// Stores information about a detected hotspot
{
	UCHAR ucHeatSpotId;					///< Id / Index of the hotspot within an hotspot array
	ULONG distance;						///< Distance to the persons position
}strFittingHeatSpot;

typedef struct peopleStruct_t		/// Structure to save the data of a single recognized person
{
	UCHAR	ucID;					///< Unique ID for the detected person. The range is from 1 to 255. The person which is detected after the one with ID 255 will get the ID 1.
	short	usOldPosX;				///< The X coordinate of the <b>last position</b> of the detected person
	short	usOldPosY;				///< The Y coordinate of the <b>last position</b> of the detected person
	short	usActPosX;				///< The X coordinate of the <b>actual position</b> of the detected person
	short	usActPosY;				///< The Y coordinate of the <b>actual position</b> of the detected person
	short	usEstPosX;				///< The X coordinate of the <b>estimated next position</b> of the detected person
	short	usEstPosY;				///< The Y coordinate of the <b>estimated next position</b> of the detected person
	BOOL	counted;				///< A flag to signalize that the person is counted and still within the hysteresis range of the counting line
	UCHAR	direction;				///< The direction in which the counting line was crossed. 0: one direction. 1: The opposite direction.
	BOOL	bNewPerson;				///< A flag to signalize if the person is recognized for the first time.
	BOOL	bHided;					///< A flag to signalize if the person is hided by another person.
	short	shTrackCounter;			///< A counter which counts up to three when the person is tracked. After it reached the value three the person will be marked as valid.
	BOOL	bValidPerson;			///< A flag to signalize if the person is valid. It is true when the person is tracked three times in a row after first detection.
	short	shEstFrames;			///< This counter shows how many frames the estimated position was chosen instead of a real hot spot.
	strFittingHeatSpot strFittingHeatSpots[3];	///< Stores the three best fitting hot spots with id's and distances.
}peopleStruct_t;

typedef struct peopleTrackandCountOutput_t		/// Output structure to deliver the results of the People Tracking and Counting main function.
{
	UCHAR g_aucDetectImg[IMG_SZ];							///< The binarisized image with a size of 15 x 15 pixels. 0: No object. 255: detected Object. 1..254 Detected person.
	UCHAR g_ucPCpeopleNum;								///< The number of detected persons
	short g_shPCcounter[2];								///< Stores the two counter values. One for each direction.
	peopleStruct_t g_astrPeoplePass[MAX_PEOPLE_IN_IMG];	///< A data set with the information about the detected persons
}peopleTrackandCountOutput_t;

typedef struct coh_t			/// A data pair to store the coordinates of a local center of heat.
{
	short ashCOH[2];			///< [0]: X coordinate. [1]: Y coordinate.
}coh_t;

typedef struct point_t			/// Structure to save the coordinates of a point.
{
	short X;					///< X coordinate
	short Y;					///< Y coordinate
}point_t;
typedef struct line_t			/// Structure to save a line between two points.
{
	point_t start;				///< Start point of the line.
	point_t end;				///< End point of the line.
}line_t;
//*****************************************************************************
//
//! @}
//
//*****************************************************************************
//*****************************************************************************
//
//! \addtogroup PTaC_func Sub functions
//! \brief The sub function reference for the People Tracking and Counting algorithm
//! @{
//
//*****************************************************************************
void	vAMG_PUB_PT_Reset();
void	vAMG_PUB_PC_SetParameters( strAlgorithmParameters );
short	shAMG_PUB_PT_ConvFtoS( float );
BOOL	bAMG_PUB_PT_CalcCenterTemp( UCHAR, UCHAR, UCHAR, UCHAR*, short*, short* );
short	shAMG_PUB_PT_CalcAverage( short*, UCHAR, BOOL* );
short	shAMG_PUB_PT_CalcIIR( short, short );
short	shAMG_PUB_PT_ConvTemperature( UCHAR[2] );
void	vAMG_PUB_PT_ConvTemperature64( UCHAR*, short*, short, ULONG );
void	vAMG_PUB_PT_CalcObjectImage ( USHORT, short*, UCHAR*, UCHAR, short* );
UCHAR	ucAMG_PUB_PT_CalcPeopleLabeling( UCHAR, UCHAR, UCHAR, UCHAR, short*, UCHAR*, BOOL* );
UCHAR	vAMG_PUB_PT_PeopleTracking( UCHAR, UCHAR, coh_t*, UCHAR, UCHAR, peopleStruct_t*, UCHAR* );
void	vAMG_PUB_PT_PeopleCounting( line_t*, UCHAR, UCHAR, peopleStruct_t*, short* );
BOOL	bAMG_PUB_PT_Interpolation( short*, short* );
void	vAMG_PUB_PT_CalcDiffImage( USHORT, short*, short*, short* );
void	vAMG_PUB_PT_CalcDetectImage( USHORT, short*, UCHAR, UCHAR* );
BOOL	bAMG_PUB_PT_ImageDilation1( UCHAR, UCHAR, UCHAR*, UCHAR* );
BOOL	bAMG_PUB_PT_UpdateBackTemp( USHORT, UCHAR*, short*, short* );
BOOL	bAMG_PUB_PT_IsImageFilledWithObjects( UCHAR* );
BOOL	bAMG_PUB_PT_IsImageDifferenzHigh( short*, short*, int );
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

#endif /* GRID_EYE_PEOPLE_TRACKING_AND_COUNTING_H_ */
