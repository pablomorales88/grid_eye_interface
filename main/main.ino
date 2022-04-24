
#include "grid_eye_config.h"
#include "Grid_Eye_People_Tracking_and_Counting.h"

#include <Wire.h>

#define I2C_SDA 
#define I2C_SCL 
#define AMG88xx_ADDRESS 0x68

void setup() {
  /* start serial port at 57600 bps:*/
  Serial.begin(57600);
  
  /* Debug */
	//configure_port_pins();
	/* Initialize Grid-Eye IIC interface */
	GE_IIC_InterfaceInit();

	/* Initialize the Grid-EYE Sensor */
	//GE_Init();
	
}

void loop() {
  // put your main code here, to run repeatedly:
  
}

/*------------------------------------------------------------------------------
	IIC interface initialize
------------------------------------------------------------------------------*/
void GE_IIC_InterfaceInit( void )
{
	
  //status_begin_amg_sensor = begin_AMG88xx(AMG88xx_ADDRESS, &Wire);
  
}

		  
static void configure_port_pins(void)
{
	// struct port_config config_port_pin;
	// struct port_config config_port_pin_UARTTX;
	
	// port_get_config_defaults(&config_port_pin);
	
	// config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	// port_pin_set_config(PIN_PA22, &config_port_pin);
	// port_pin_set_output_level(PIN_PA22, true);
	
	// port_pin_set_config(PIN_PA14, &config_port_pin);
	// port_pin_set_config(PIN_PA15, &config_port_pin);
	// port_pin_set_output_level(PIN_PA14, true);
	// port_pin_set_output_level(PIN_PA15, true);
	
	// port_get_config_defaults(&config_port_pin_UARTTX);
	// config_port_pin_UARTTX.direction = PORT_PIN_DIR_INPUT;
	// port_pin_set_config(PIN_PA08, &config_port_pin_UARTTX);


}


/*------------------------------------------------------------------------------
	Sensor initialize
------------------------------------------------------------------------------*/
void GE_Init( void )
{
	//GE_SetOperationMode( GE_PCTL_NORMAL_MODE );
	//GE_SetResetMode( GE_RST_INITIAL_RST );
}


/**************************************************************************/
/*!
    @brief  Setups the I2C interface and hardware
    @param  addr Optional I2C address the sensor can be found on. Default is
   0x69
    @param  theWire the I2C object to use, defaults to &Wire
    @returns True if device is set up, false on any failure
*/
/**************************************************************************/
bool begin_AMG88xx(uint8_t addr, TwoWire *theWire) {
  
  
  // if (i2c_dev)
  //   delete i2c_dev;
  // i2c_dev = new Adafruit_I2CDevice(addr, theWire);
  // if (!i2c_dev->begin())
  //   return false;

  //uint8_t _addr = addr;
  //TwoWire _wire = theWire;
  

  
  // enter normal mode
  /*_pctl.PCTL = AMG88xx_NORMAL_MODE;
  write8(AMG88xx_PCTL, _pctl.get());

  // software reset
  _rst.RST = AMG88xx_INITIAL_RESET;
  write8(AMG88xx_RST, _rst.get());

  // disable interrupts by default
  disableInterrupt();

  // set to 10 FPS
  _fpsc.FPS = AMG88xx_FPS_10;
  write8(AMG88xx_FPSC, _fpsc.get());*/

  delay(100);

  return true;
}
