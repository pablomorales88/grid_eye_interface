/*
 * grid_eye_cinfig.h
 *
 * Created: 2015/5/21 11:23:53
 *  Author: Sam
 */ 


#ifndef GRID_EYE_CINFIG_H_
#define GRID_EYE_CINFIG_H_

//*****************************************************************************
//
//! \addtogroup PTaC GridEye People Tracking and Counting
//! @{
//
//*****************************************************************************
//*****************************************************************************
//
//! \addtogroup GRID_EYE_CONFIGURE Configuration
//! \brief The configure macro for grid eye sensor
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup GRID_EYE_Data_Type Data Type
//! \brief The data type for grid eye sensor
//! @{
//
//*****************************************************************************
typedef unsigned char			BOOL;
typedef unsigned char			UCHAR;
typedef unsigned short			USHORT;
typedef unsigned long			ULONG;
typedef signed char				CHAR;

#ifdef POLYSPACE
#define false                              0
#define true                      (!(false))
typedef unsigned char				uint8_t;
typedef signed short				int16_t;
typedef unsigned short				ushort;

#ifndef NULL
#define NULL   0
#endif
#endif

#include "Hw_grid_eye.h"

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup GRID_EYE_System_Config Feature Setting
//! \brief The configure macro for grid eye sensor
//! @{
//
//*****************************************************************************
/* Grid-EYE's number of pixels */
#define	TRUE					(1)
#define	FALSE					(0)
#define	SHORT_MAX_VAL	   	    ( 32767)				/* 0x7FFF		*/
#define	SHORT_MIN_VAL		    (-32768)					/* 0x8000		*/
#define UCHAR_MAX_VAL			( 255)
#define	ULONG_MAX_VAL		    ( 4294967295)		    /* 0xFFFFFFFF	*/

/* Grid-EYE's number of image pixels */
#define imageWidth				8
#define imageHeight				8

/* Grid-EYE's I2C Sampling period */
#define GRID_EYE_SAMP_PERI      600   ///< Sampling period (ms) ;
#define GRID_EYE_IIC_SPEED      I2C_MASTER_BAUD_RATE_400KHZ ///< IIC interface speed ;

/* Grid-EYE's I2C slave address */
#define GRID_EYE_ADDRESS        0x68 ///< AD_SELECT == GND ? 0x68 : 0x69 ;
#define	GRIDEYE_ADR			    GRID_EYE_ADDRESS

/* Grid-EYE's register address */
#define	GRIDEYE_REG_THS00	   (GE_TTHL_REG)	/* head address of thermistor  register	*/
#define	GRIDEYE_REG_TMP00	   (GE_PIXEL_BASE)	/* head address of temperature register	*/

/* Grid-EYE's register size */
#define	GRIDEYE_REGSZ_THS  	   (0x02)	/* size of thermistor  register		*/
#define	GRIDEYE_REGSZ_TMP	   (0x80)	/* size of temperature register		*/

/* Grid-EYE's number of pixels */
#define	SNR_SZ_X			   (imageWidth)
#define	SNR_SZ_Y			   (imageHeight)
#define	SNR_SZ				   (SNR_SZ_X * SNR_SZ_Y)

/* Setting size of human detection */
#define	IMG_SZ_X			   (SNR_SZ_X * 2 - 1)
#define	IMG_SZ_Y			   (SNR_SZ_Y * 2 - 1)
#define	IMG_SZ				   (IMG_SZ_X * IMG_SZ_Y)

/* Parameters of human detection */
#define	AVRG_ARRAY_SIZE		   (10)
//#define	TEMP_MEDIAN_FILTER	   (0)
//#define	TEMP_SMOOTH_COEFF	   (0.8f)
//#define	DIFFTEMP_THRESH		   (0.3f)
//#define PERSON_SIZE_FILTER_COEFF (0.995f)
#define	DETECT_MARK			   ((UCHAR)0xFF)
//#define	BKUPDT_COEFF		   (0.3f)


#define Abs(a)              (((a) <  0 ) ? -(a) : (a))

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
//*****************************************************************************
//
//! @}
//
//*****************************************************************************
#endif /* GRID_EYE_CINFIG_H_ */
