#include <Wire.h>

#include "grid_eye_config.h"
#include "Grid_Eye_People_Tracking_and_Counting.h"

#define I2C_SDA             21
#define I2C_SCL             22
#define PIXELS              64
#define RX_BUFF_LEN         1024
#define MAX_LEN_ASCII       6

BOOL    bAMG_PUB_PC_SetCountlines( line_t* astrLines );
BOOL    bAMG_PUB_SMP_People_Tracking_and_Counting( UCHAR*, peopleTrackandCountOutput_t* );
void    vAMG_PUB_PT_Reset_Algorithm( peopleTrackandCountOutput_t* );
void    vAMG_PUB_PC_SetTrackMeParameters( strAlgorithmParameters strAlgoParams );

static peopleTrackandCountOutput_t peopleTrackandCountOut;
static UCHAR g_aucRawI2C [IMG_SZ * 2];
static uint16_t pixel[PIXELS];
static byte address = 104;

static void tryToSendResults()
{
    UCHAR package[2];
    UCHAR i;

    // Send start sign "***"
    Serial.write( "***", 3 );

    // Send thermistor value (Actual is not needed)
    //Serial.write( "00", 2 );

    // Send the raw Image data
    Serial.write( (UCHAR*)g_aucRawI2C, 128 );

    // Send the binarisized Image
    for (i = 0; i < 225; i++)
    {
        package[0] = (UCHAR)peopleTrackandCountOut.g_aucDetectImg[i];
        Serial.write( (UCHAR*)package, 1 );
    }

    // Send the number of people
    package[0] = (UCHAR)peopleTrackandCountOut.g_ucPCpeopleNum;
    Serial.write( (UCHAR*)package, 1 );

    // Send the counter values
    package[0] = (UCHAR)(peopleTrackandCountOut.g_shPCcounter[0] >> 8);
    package[1] = (UCHAR)(peopleTrackandCountOut.g_shPCcounter[0] & 0xFF);
    Serial.write( (UCHAR*)package, 2 );
    package[0] = (UCHAR)(peopleTrackandCountOut.g_shPCcounter[1] >> 8);
    package[1] = (UCHAR)(peopleTrackandCountOut.g_shPCcounter[1] & 0xFF);
    Serial.write( (UCHAR*)package, 2 );

    /*
    // Send the information of the detected people
    for(i=0;i<MAX_PEOPLE_IN_IMG;i++)
    {
        if( peopleTrackandCountOut.g_astrPeoplePass[i].ucID != 0  &&
            peopleTrackandCountOut.g_astrPeoplePass[i].bValidPerson == TRUE)
        {
            package[0] = (char)peopleTrackandCountOut.g_astrPeoplePass[i].ucID;
            Serial.write( (UCHAR*)package, 1 );
            package[0] = (UCHAR)(peopleTrackandCountOut.g_astrPeoplePass[i].usOldPosX >> 8);
            package[1] = (UCHAR)(peopleTrackandCountOut.g_astrPeoplePass[i].usOldPosX & 0xFF);
            Serial.write( (UCHAR*)package, 2 );
            package[0] = (UCHAR)(peopleTrackandCountOut.g_astrPeoplePass[i].usOldPosY >> 8);
            package[1] = (UCHAR)(peopleTrackandCountOut.g_astrPeoplePass[i].usOldPosY & 0xFF);
            Serial.write( (UCHAR*)package, 2 );
            package[0] = (UCHAR)(peopleTrackandCountOut.g_astrPeoplePass[i].usActPosX >> 8);
            package[1] = (UCHAR)(peopleTrackandCountOut.g_astrPeoplePass[i].usActPosX & 0xFF);
            Serial.write( (UCHAR*)package, 2 );
            package[0] = (UCHAR)(peopleTrackandCountOut.g_astrPeoplePass[i].usActPosY >> 8);
            package[1] = (UCHAR)(peopleTrackandCountOut.g_astrPeoplePass[i].usActPosY & 0xFF);
            Serial.write( (UCHAR*)package, 2 );
        }
    }
    */
}

/*------------------------------------------------------------------------------
  Sensor initialize
------------------------------------------------------------------------------*/
static void GE_Init( void )
{
    GE_SetOperationMode( GE_PCTL_NORMAL_MODE );
    GE_SetResetMode( GE_RST_INITIAL_RST );
}

/*------------------------------------------------------------------------------
  Set sensor operation mode.
------------------------------------------------------------------------------*/
static void GE_SetOperationMode( uint8_t Op_Mode )
{
    Wire.beginTransmission(address);
    uint8_t raw[2];
    raw[0] = GE_POWER_CTL_REG;
    raw[1] = Op_Mode;
    Wire.write(raw,2);
    Wire.endTransmission();
}

/*------------------------------------------------------------------------------
  Set sensor reset mode.
------------------------------------------------------------------------------*/
static void GE_SetResetMode(uint8_t Rst_Mode )
{
    Wire.beginTransmission(address);
    uint8_t raw[2];
    raw[0] = GE_RESET_REG;
    raw[1] = Rst_Mode;
    Wire.write(raw,2);
    Wire.endTransmission();
}

/*------------------------------------------------------------------------------
  Set frame rate
------------------------------------------------------------------------------*/
static void GE_SetFrameRate(uint8_t Frame_Rate )
{
    Wire.beginTransmission(address);
    uint8_t raw[2];
    raw[0] = GE_FPSC_REG;
    raw[1] = Frame_Rate;
    Wire.write(raw,2);
    Wire.endTransmission();
}

/*------------------------------------------------------------------------------
  Set average mode
------------------------------------------------------------------------------*/
static void GE_SetAverageMode(uint8_t Average_Mode )
{
    Wire.beginTransmission(address);
    uint8_t raw[2];
    raw[0] = GE_AVE_REG;
    raw[1] = Average_Mode;
    Wire.write(raw,2);
    Wire.endTransmission();
}

static uint16_t GE_ReadThermistor()
{
    return getRegister(GE_TTHL_REG,2);
}

static int16_t getRegister(unsigned char reg, int8_t len)
{
    int16_t result;

    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)address, (uint8_t)len);

    while(Wire.available())    // client may send less than requested
    {
      // Get bytes from sensor
      uint8_t lsb = Wire.read(); 
      uint8_t msb = Wire.read(); 
  
      // concat bytes into int
      result = (uint16_t)msb << 8 | lsb;
    }

    Wire.endTransmission();

    return result;
}

static void getPixels()
{
    Wire.beginTransmission(address);
    Wire.write(GE_PIXEL_BASE);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)address, (uint8_t)128);

    for(int i=0; i<128; i++)
    {
        if(Wire.available())
        {
          g_aucRawI2C[i] = Wire.read();
        }
    }
    Wire.endTransmission();

    for (int i = 0; i < 64; i++)
    {
        uint8_t pos = i << 1;
        pixel[i] = ((uint16_t)g_aucRawI2C[pos + 1] << 8) | ((uint16_t)g_aucRawI2C[pos]);
    }

    bAMG_PUB_SMP_People_Tracking_and_Counting( g_aucRawI2C, &peopleTrackandCountOut );
}

static void initializeAMG()
{
    int error = 0;

    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error==4)
    {
        Serial.print("Unknow error at address 0x");
        if (address<16)
        {
            Serial.print("0");
        }
        Serial.println(address,HEX);
    }
    else if (error != 0)
    {
        Serial.print("I2C device error at address 0x");
        Serial.println(error);
    }

    GE_Init();

    vAMG_PUB_PT_Reset_Algorithm( &peopleTrackandCountOut );
}

static void initializeI2C()
{
    /* start serial port at 57600 bps:*/
    Serial.begin(57600);
    delay(1000);
    pinMode(12,OUTPUT);
    pinMode(13, OUTPUT);
    digitalWrite(12, HIGH);
    digitalWrite(13, HIGH);

    Wire.begin(I2C_SDA, I2C_SCL, 400000);
}

static float buildUp(unsigned char* outbox)
{
  unsigned long d;
 
  d = (outbox[3] << 24) | (outbox[2] << 16) | (outbox[1] << 8) | (outbox[0]);
  float member = *(float *)&d;
  return member;
}

static strAlgorithmParameters extractParamsFromMessage( UCHAR* message )
{
    int i = 0;
    strAlgorithmParameters params;

    // TODO Interface protocol
    params.usRectangleFilterLength			= (message[i++] << 8) | (message[i++] & 0xFF);	// 2 Byte
    params.usMedianFilterLength				= (message[i++] << 8) | (message[i++] & 0xFF);	// 2 Byte
    params.flIirFilterValue					= buildUp(&message[i]);							// 4 Byte
    i = i + 4;
    params.eInterpolationType				= (eInterpolationType_enum) (message[i++] & 0xFF);						// 1 Byte
    params.flTangentialInterpolationFactor	= buildUp(&message[i]);						// 4 Byte
    i = i + 4;
    params.flCornerThreshold				= buildUp(&message[i]);						// 4 Byte
    i = i + 4;
    params.flSideThreshold					= buildUp(&message[i]);						// 4 Byte
    i = i + 4;
    params.flCenterThreshold				= buildUp(&message[i]);						// 4 Byte
    i = i + 4;
    params.bAutomaticPersonParameterControl = (message[i++] & 0xFF);						// 1 Byte
    params.bAutomaticParameterCalculation	= (message[i++] & 0xFF);						// 1 Byte
    params.flPersonRadius					= buildUp(&message[i]);						// 4 Byte
    i = i + 4;
    params.flPersonArea						= buildUp(&message[i]);						// 4 Byte
    i = i + 4;
    params.flPersonMaxSpeed					= buildUp(&message[i]);						// 4 Byte
    i = i + 4;
    params.flAdaptionCoefficient			= buildUp(&message[i]);						// 4 Byte
    i = i + 4;
    params.flOverlappFactor					= buildUp(&message[i]);						// 4 Byte
    i = i + 4;
    params.flAreaFactor						= buildUp(&message[i]);						// 4 Byte
    i = i + 4;
    params.flTempFactor						= buildUp(&message[i]);						// 4 Byte
    i = i + 4;
    params.shMaxMovementDistanceFactor		= (message[i++] << 8) | (message[i++] & 0xFF);	// 2 Byte
    params.shMinPersonDistFactor			= (message[i++] << 8) | (message[i++] & 0xFF);	// 2 Byte
    params.usMaxEstimatedFrames				= (message[i++] << 8) | (message[i++] & 0xFF);	// 2 Byte
    params.flBackgroundIirFilterCoeffNonObjPixel    = buildUp(&message[i]);						// 4 Byte
    params.flBackgroundIirFilterCoeffObjPixel		= buildUp(&message[i]);						// 4 Byte

    return params;
}

static BOOL GE_MessageInterpretion( UCHAR* input )
{
    USHORT usPosition = 0;
    USHORT usMessageSize = 0;
    USHORT usMessageType = 0;
    line_t astrNewCountLine[3];
    enum eMessageType {CHANGE_COUNT_LINES='0', CHANGE_FRAME_RATE, CHANGE_AVERAGE_MODE,
            RESET_BACKEND, CHANGE_ALGORITHM_PARAMETER, RESET_GRID_EYE };
    enum eFrameRate {FR_1FPS='0', FR_10FPS };
    enum eResetMode {RESET_FLAG='0', RESET_SOFTWARE };
    enum eAverageMode {TWICE_MA='0', NONE };
    strAlgorithmParameters algoParams;

    while ( input[usPosition++] == '*' ){}
    usMessageSize = input[--usPosition];
    usMessageType = input[++usPosition];

    switch (usMessageType)
    {
        case CHANGE_COUNT_LINES :
            astrNewCountLine[0].start.X = (short)(input[++usPosition] << 8) | input[++usPosition];
            astrNewCountLine[0].start.Y = (short)(input[++usPosition] << 8) | input[++usPosition];
            astrNewCountLine[0].end.X	= (short)(input[++usPosition] << 8) | input[++usPosition];
            astrNewCountLine[0].end.Y	= (short)(input[++usPosition] << 8) | input[++usPosition];
            astrNewCountLine[1].start.X = (short)(input[++usPosition] << 8) | input[++usPosition];
            astrNewCountLine[1].start.Y = (short)(input[++usPosition] << 8) | input[++usPosition];
            astrNewCountLine[1].end.X	= (short)(input[++usPosition] << 8) | input[++usPosition];
            astrNewCountLine[1].end.Y	= (short)(input[++usPosition] << 8) | input[++usPosition];
            astrNewCountLine[2].start.X = (short)(input[++usPosition] << 8) | input[++usPosition];
            astrNewCountLine[2].start.Y = (short)(input[++usPosition] << 8) | input[++usPosition];
            astrNewCountLine[2].end.X	= (short)(input[++usPosition] << 8) | input[++usPosition];
            astrNewCountLine[2].end.Y	= (short)(input[++usPosition] << 8) | input[++usPosition];
            Serial.write( "******", 6 );
            return bAMG_PUB_PC_SetCountlines( astrNewCountLine );
            break;
        case CHANGE_FRAME_RATE :
            if(input[++usPosition] == FR_1FPS)
            {
                GE_SetFrameRate(GE_FPSC_1FPS);
            }
            else
            {
                GE_SetFrameRate(GE_FPSC_10FPS);
            }
            Serial.write( "******", 6 );
            return false;
            break;
        case CHANGE_AVERAGE_MODE :
            if(input[++usPosition] == TWICE_MA)
            {
                GE_SetAverageMode(GE_AVE_TWICE_MA);
            }
            else
            {
                GE_SetAverageMode(GE_AVE_NONE);
            }
            Serial.write( "******", 6 );
            return false;
            return false;
            break;
        case RESET_BACKEND :
            vAMG_PUB_PT_Reset_Algorithm( &peopleTrackandCountOut );
            Serial.write( "******", 6 );
            return false;
            break;
        case CHANGE_ALGORITHM_PARAMETER :
            algoParams = extractParamsFromMessage( &input[++usPosition] );
            vAMG_PUB_PC_SetTrackMeParameters( algoParams );
            Serial.write( "******", 6 );
            return false;
            break;
        case RESET_GRID_EYE :
            if(input[++usPosition] == RESET_FLAG)
            {
                GE_SetResetMode(GE_RST_FLAG_RST);
            }
            else
            {
                GE_SetResetMode(GE_RST_INITIAL_RST);
            }
            Serial.write( "******", 6 );
            return false;
            break;
            
        default :
            return false;
            break;
    }
}

static void processCommand()
{
    static UCHAR rx_buffer[RX_BUFF_LEN];
    static uint16_t rx_count = 0;

    while (Serial.available())
    {
        if (rx_count >= RX_BUFF_LEN)
        {
            rx_count = 0;
        }

        rx_buffer[rx_count++] = Serial.read();
    }

    uint8_t command_len = 0xFF;

    for (uint16_t i=0; i<rx_count; ++i)
    {
        if (rx_buffer[i] == '*')
        {
            command_len = rx_buffer[i+1];

            if (rx_count >= command_len)
            {
                GE_MessageInterpretion(&rx_buffer[i]);
                i += command_len-1;
            }
        }

        if (i+1 >= rx_count)
        {
            memset(rx_buffer, 0, sizeof(rx_buffer));
            rx_count = 0;
        }
    }
}

void setup()
{
    initializeI2C();
}

void loop()
{
    initializeAMG();

    while(1)
    {
        processCommand();
        getPixels();
        tryToSendResults();
        delay(2000);
    }
}
