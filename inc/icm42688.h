#pragma once

#include <stdint.h>
#include "icm42688_registers.h"
#include "icm42688_config.h"
#include <zephyr/drivers/spi.h>




typedef struct icm42688_struct icm42688_struct;

// Interfaces
typedef void      (*icm42688_interface_init)					(icm42688_struct*);
typedef int       (*icm42688_interface_begin)			    	(icm42688_struct*);	
typedef int       (*icm42688_interface_spi_init)		        (icm42688_struct*);
typedef int       (*icm42688_interface_spi_begin)		        (icm42688_struct*);	
typedef float     (*icm42688_interface_getTemperature)	    	(icm42688_struct*);
typedef float     (*icm42688_interface_getAccelDataX)	    	(icm42688_struct*);	
typedef float     (*icm42688_interface_getAccelDataY)	    	(icm42688_struct*);	
typedef float     (*icm42688_interface_getAccelDataZ)		    (icm42688_struct*);	
typedef float     (*icm42688_interface_getGyrolDataX)		    (icm42688_struct*);	
typedef float     (*icm42688_interface_getGyroDataY)		    (icm42688_struct*);	
typedef float     (*icm42688_interface_getGyroDataZ)		    (icm42688_struct*);	
typedef void      (*icm42688_interface_tapDetectionInit)		(icm42688_struct*,uint8_t);
typedef void      (*icm42688_interface_getTapInformation)		(icm42688_struct*);
typedef uint8_t   (*icm42688_interface_numberOfTap)	            (icm42688_struct*);
typedef uint8_t   (*icm42688_interface_axisOfTap)	            (icm42688_struct*);
typedef void      (*icm42688_interface_wakeOnMotionInit)		(icm42688_struct*);
typedef void      (*icm42688_interface_setWOMTh)		        (icm42688_struct*,uint8_t,uint8_t);
typedef void      (*icm42688_interface_setWOMInterrupt)		(icm42688_struct*,uint8_t);
typedef void      (*icm42688_interface_enableSMDInterrupt)		(icm42688_struct*,uint8_t);
typedef uint8_t   (*icm42688_interface_readInterruptStatus)	    (icm42688_struct*,uint8_t);
typedef bool      (*icm42688_interface_setODRAndFSR)	        (icm42688_struct*,uint8_t,uint8_t,uint8_t);
typedef void      (*icm42688_interface_startFIFOMode)	        (icm42688_struct*);
typedef void      (*icm42688_interface_stopFIFOMode)	        (icm42688_struct*);
typedef void      (*icm42688_interface_getFIFOData)	            (icm42688_struct*);
typedef void      (*icm42688_interface_setINTMode)	            (icm42688_struct*,uint8_t,uint8_t,uint8_t,uint8_t);
typedef void      (*icm42688_interface_startGyroMeasure)		(icm42688_struct*,uint8_t);
typedef void      (*icm42688_interface_startAccelMeasure)		(icm42688_struct*,uint8_t);
typedef void      (*icm42688_interface_startTempMeasure)	    (icm42688_struct*);

/// @brief Protected
typedef void      (*icm42688_interface_writeReg)				(uint8_t,void*,size_t);
typedef uint8_t    (*icm42688_interface_readReg)				(icm42688_struct*,uint8_t, void*,uint8_t);
typedef void      (*icm42688_interface_setFIFODataMode)	(icm42688_struct*);

/// @brief Private 
typedef void      (*icm42688_interface_setGyroNotchFilterFHz)	(icm42688_struct*,double,uint8_t);
typedef void      (*icm42688_interface_setGyroNFbandwidth)		(icm42688_struct*,uint8_t);
typedef void      (*icm42688_interface_setGyroNotchFilter)		(icm42688_struct*,bool);
typedef void      (*icm42688_interface_setAAFBandwidth)		    (icm42688_struct*,uint8_t,uint8_t);
typedef void      (*icm42688_interface_setAAF)		            (icm42688_struct*,uint8_t,bool);
typedef void      (*icm42688_interface_setUIFilter)		        (icm42688_struct*,uint8_t,uint8_t,uint8_t);

struct icm42688_struct{

  uint8_t _r;
  uint8_t _g;
  uint8_t _b;
  uint8_t _mode;
  uint8_t _tapNum ;
  uint8_t _tapAxis;
  uint8_t _tapDir ;
  float _gyroRange;
  float _accelRange;
  bool FIFOMode;
  int16_t _accelX;
  int16_t _accelY;
  int16_t _accelZ;
  int16_t _gyroX;
  int16_t _gyroZ;
  int16_t _gyroY;
  int8_t _temp;
  int8_t _INTPin;

  sAccelConfig0_t accelConfig0;
  sPWRMgmt0_t PWRMgmt0;
  sINTFConfig1_t INTFConfig1;
  sAccelConfig1_t accelConfig1;
  sGyroAccelConfig0_t gyroAccelConfig0;
  sAPEXConfig7_t APEXConfig7;
  sAPEXConfig8_t APEXConfig8;
  sSMDConfig_t SMDConfig;
  sGyroConfig1_t  gyroConfig1;
  sFIFOConfig1_t FIFOConfig1;
  sINTConfig_t INTConfig;
  sGyroConfig0_t gyroConfig0;
  sAPEXConfig0_t APEXConfig0;
  sGyroConfigStatic9_t gyroConfigStatic9;
  sGyroConfigStatic2_t gyroConfigStatic2;
  sGyroConfigStatic5_t gyroConfigStatic5;
  sAccelConfigStatic2_t accelConfigStatic2;
  sAccelConfigStatic4_t accelConfigStatic4;
  sINTSource_t  INTSource;

  icm42688_interface_begin	                begin;  
  icm42688_interface_spi_init		            spi_init;  
  icm42688_interface_spi_begin		          spi_begin;  
  icm42688_interface_getTemperature	        getTemperature;  
  icm42688_interface_getAccelDataX	        getAccelDataX;  
  icm42688_interface_getAccelDataY	        getAccelDataY;  
  icm42688_interface_getAccelDataZ		      getAccelDataZ;  
  icm42688_interface_getGyrolDataX		      getGyroDataX;  
  icm42688_interface_getGyroDataY		        getGyroDataY;  
  icm42688_interface_getGyroDataZ		        getGyroDataZ;  
  icm42688_interface_tapDetectionInit	      tapDetectionInit;  
  icm42688_interface_getTapInformation	    getTapInformation;  
  icm42688_interface_numberOfTap	          numberOfTap;  
  icm42688_interface_axisOfTap	            axisOfTap;  
  icm42688_interface_wakeOnMotionInit	      wakeOnMotionInit;  
  icm42688_interface_setWOMTh		            setWOMTh;  
  icm42688_interface_setWOMInterrupt        setWOMInterrupt;  
  icm42688_interface_enableSMDInterrupt	    enableSMDInterrupt;  
  icm42688_interface_readInterruptStatus	  readInterruptStatus;  
  icm42688_interface_setODRAndFSR	          setODRAndFSR;  
  icm42688_interface_startFIFOMode	        startFIFOMode;	    
  icm42688_interface_stopFIFOMode	          stopFIFOMode;	   
  icm42688_interface_getFIFOData	          getFIFOData;	         
  icm42688_interface_setINTMode	            setINTMode;	         
  icm42688_interface_startGyroMeasure	      startGyroMeasure;  
  icm42688_interface_startAccelMeasure	    startAccelMeasure;  
  icm42688_interface_startTempMeasure	      startTempMeasure;  
                                                                    
                                                                    
  icm42688_interface_writeReg			          writeReg;			 
  icm42688_interface_readReg				        readReg;		
  icm42688_interface_setFIFODataMode	      setFIFODataMode;	    
                                                                    
                                                                    
  icm42688_interface_setGyroNotchFilterFHz  setGyroNotchFilterFHz; 
  icm42688_interface_setGyroNFbandwidth	    setGyroNFbandwidth;	 
  icm42688_interface_setGyroNotchFilter	    setGyroNotchFilter;	 
  icm42688_interface_setAAFBandwidth		    setAAFBandwidth;		 
  icm42688_interface_setAAF		              setAAF;		         
  icm42688_interface_setUIFilter	          setUIFilter;	         	    


};

void icm42688();
  /**
   * @fn begin
   * @brief Init function
   * @return Init result
   * @retval ERR_OK         Init succeed
   * @retval ERR_DATA_BUS   Bus data read error
   * @retval ERR_IC_VERSION The read Sensor ID is wrong
   */
int icm42688_begin(icm42688_struct *s);

int icm42688_spi_init(icm42688_struct*);

int icm42688_spi_begin(icm42688_struct *s);


void icm42688_writeReg(uint8_t reg, void* pBuf, size_t size) ;

uint8_t icm42688_readReg(icm42688_struct *s,uint8_t reg, void* pBuf, uint8_t size);



float icm42688_getTemperature(icm42688_struct *s);

  /**
   * @fn getAccelDataX
   * @brief Get X-axis accelerometer value
   * @return X-axis accelerometer value, unit: mg
   */
float icm42688_getAccelDataX(icm42688_struct *s);

  /**
   * @fn getAccelDataY
   * @brief Get Y-axis accelerometer value
   * @return Y-axis accelerometer value, unit: mg
   */
float icm42688_getAccelDataY(icm42688_struct *s);

  /**
   * @fn getAccelDataZ
   * @brief Get Z-axis accelerometer value
   * @return Z-axis accelerometer value, unit: mg
   */
float icm42688_getAccelDataZ(icm42688_struct *s);

  /**
   * @fn getGyroDataX
   * @brief Get X-axis gyroscope value
   * @return X-axis gyroscope value, unit: dps
   */
float icm42688_getGyroDataX(icm42688_struct *s);

  /**
   * @fn getGyroDataY
   * @brief Get Y-axis gyroscope value
   * @return Y-axis gyroscope value, unit: dps
   */
float icm42688_getGyroDataY(icm42688_struct *s);

  /**
   * @fn getGyroDataZ
   * @brief Get Z-axis gyroscope value
   * @return Z-axis gyroscope value, unit: dps
   */
float icm42688_getGyroDataZ(icm42688_struct *s);

  /**
   * @fn tapDetectionInit
   * @brief Tap detection init
   * @param accelMode Accelerometer operating mode
   * @n      0 for operating in low-power mode
   * @n      1 for operating in low-noise mode
   */
void icm42688_tapDetectionInit(icm42688_struct *s,uint8_t accelMode);

  /**
   * @fn getTapInformation
   * @brief Get tap information
   */
void icm42688_getTapInformation(icm42688_struct *s);

  /**
   * @fn numberOfTap
   * @brief Get the number of tap: single-tap or double tap
   * @return The number of tap
   * @retval TAP_SINGLE   Single-tap
   * @retval TAP_DOUBLE   Double tap
   */
uint8_t icm42688_numberOfTap(icm42688_struct *s);

  /**
   * @fn axisOfTap
   * @brief Get the axis on which the tap occurred: X-axis, Y-axis, or Z-axis
   * @return Tap axis
   * @retval X_AXIS   X-axis
   * @retval Y_AXIS   Y-axis
   * @retval Z_AXIS   Z-axis
   */
uint8_t icm42688_axisOfTap(icm42688_struct *s);

  /**
   * @fn wakeOnMotionInit
   * @brief Wake on motion init
   */
void icm42688_wakeOnMotionInit(icm42688_struct *s);

  /**
   * @fn setWOMTh
   * @brief Set wake on motion interrupt threshold of axis accelerometer
   * @param axis  x/y/z轴
   * @n       X_AXIS_WOM
   * @n       Y_AXIS_WOM
   * @n       Z_AXIS_WOM
   * @n       ALL
   * @param threshold  Range(0-255) [WoM thresholds are expressed in fixed “mg” independent of the selected Range [0g : 1g]; Resolution 1g/256=~3.9mg]
   */
void icm42688_setWOMTh(icm42688_struct *s,uint8_t axis,uint8_t threshold);

  /**
   * @fn setWOMInterrupt
   * @brief Enable wake on motion interrupt
   * @param axis  X-axis, Y-axis, or Z-axis
   * @n       X_AXIS_WOM
   * @n       Y_AXIS_WOM
   * @n       Z_AXIS_WOM
   */
void icm42688_setWOMInterrupt(icm42688_struct *s,uint8_t axis);

  /**
   * @fn enableSMDInterrupt
   * @brief Set important motion detection mode and enable SMD interrupt
   * @param mode  
   * @n      0: disable SMD
   * @n      2 : SMD short (1 sec wait) An SMD event is detected when two WOM are detected 1 sec apart
   * @n      3 : SMD long (3 sec wait) An SMD event is detected when two WOM are detected 3 sec apart
   */
void icm42688_enableSMDInterrupt(icm42688_struct *s,uint8_t mode);

  /**
   * @fn readInterruptStatus
   * @brief Read interrupt information and clear interrupt
   * @param reg Interrupt information register
   * @n      ICM42688_INT_STATUS2    Obtain interrupt information of SMD_INT, WOM_X_INT, WOM_Y_INT, WOM_Z_INT and clear them
   * @n      ICM42688_INT_STATUS3    Obtain interrupt information of STEP_DET_INT, STEP_CNT_OVF_INT, TILT_DET_INT, WAKE_INT, TAP_DET_INT and clear them
   * @return Interrupt information, return 0 when no interrupt.
   */
uint8_t icm42688_readInterruptStatus(icm42688_struct *s,uint8_t reg);

  /**
   * @fn setODRAndFSR
   * @brief Set ODR and Full-scale range of gyroscope or accelerometer.
   * @param who  GYRO/ACCEL/ALL
   * @n       GYRO: indicate only set gyroscope
   * @n       ACCEL: indicate only set accelerometer
   * @param ODR Output data rate
   * @n       ODR_32KHZ         Support: Gyro/Accel(LN mode)
   * @n       ODR_16KHZ         Support: Gyro/Accel(LN mode)
   * @n       ODR_8KHZ          Support: Gyro/Accel(LN mode)
   * @n       ODR_4KHZ          Support: Gyro/Accel(LN mode)
   * @n       ODR_2KHZ          Support: Gyro/Accel(LN mode)
   * @n       ODR_1KHZ          Support: Gyro/Accel(LN mode)
   * @n       ODR_200HZ         Support: Gyro/Accel(LP or LN mode)
   * @n       ODR_100HZ         Support: Gyro/Accel(LP or LN mode)
   * @n       ODR_50HZ          Support: Gyro/Accel(LP or LN mode)
   * @n       ODR_25KHZ         Support: Gyro/Accel(LP or LN mode)
   * @n       ODR_12_5KHZ       Support: Gyro/Accel(LP or LN mode)
   * @n       ODR_6_25KHZ       Support: Accel(LP mode)
   * @n       ODR_3_125HZ       Support: Accel(LP mode)
   * @n       ODR_1_5625HZ      Support: Accel(LP mode)
   * @n       ODR_500HZ         Support: Accel(LP or LN mode)
   * @param FSR Full-scale range
   * @n       FSR_0      Gyro:±2000dps   /   Accel: ±16g
   * @n       FSR_1      Gyro:±1000dps   /   Accel: ±8g
   * @n       FSR_2      Gyro:±500dps    /   Accel: ±4g
   * @n       FSR_3      Gyro:±250dps    /   Accel: ±2g
   * @n       FSR_4      Gyro:±125dps    /   Accel: not optional
   * @n       FSR_5      Gyro:±62.5dps   /   Accel: not optional
   * @n       FSR_6      Gyro:±31.25dps  /   Accel: not optional
   * @n       FSR_7      Gyro:±15.625dps /   Accel: not optional
   * @return Set result
   * @retval true   indicate the setting succeeds
   * @retval flase  indicate selected parameter is wrong
   */
bool icm42688_setODRAndFSR(icm42688_struct *s,uint8_t who,uint8_t ODR,uint8_t FSR);

  /**
   * @fn startFIFOMode
   * @brief Enable FIFO
   */
void icm42688_startFIFOMode(icm42688_struct *s);

  /**
   * @fn sotpFIFOMode
   * @brief Disable FIFO
   */
void icm42688_stopFIFOMode(icm42688_struct *s);

  /**
   * @fn getFIFOData
   * @brief Read FIFO data, read temperature, gyroscope and accelerometer data and save them for parse.
   */
void icm42688_getFIFOData(icm42688_struct *s);

  /**
   * @fn setINTMode
   * @brief Set interrupt mode
   * @param INTPin  Interrupt pin 
   * @n       1  Use INT1 interrupt pin
   * @n       2  Use INT2 interrupt pin
   * @param INTmode Set interrupt mode
   * @n       1  Interrupt lock mode (polarity remains unchanged when interrupt triggered, and restore after clearing interrupt)
   * @n       0  Pulse mode
   * @param INTPolarity Level polarity output by interrupt
   * @n       0  Interrupt pin polarity is LOW when producing interrupt
   * @n       1  Interrupt pin polarity is HIGH when producing interrupt
   * @param INTDriveCircuit  
   * @n       0  Open drain
   * @n       1  Push pull
   */
void icm42688_setINTMode(icm42688_struct *s,uint8_t INTPin,uint8_t INTmode,uint8_t INTPolarity,uint8_t INTDriveCircuit);

  /**
   * @fn startGyroMeasure
   * @brief Start gyroscope
   * @param mode Set gyroscope working mode
   * @n       STANDBY_MODE_ONLY_GYRO 1  Set stanby mode, only support gyroscope
   * @n       LN_MODE  3                Set low-noise mode
   */
void icm42688_startGyroMeasure(icm42688_struct *s,uint8_t mode);

  /**
   * @fn startAccelMeasure
   * @brief Start accelerometer
   * @param mode Set accelerometer working mode
   * @n       LP_MODE_ONLY_ACCEL  2     Set low-power mode, only support accelerometer
   * @n       LN_MODE  3                Set low-noise mode
   */
void icm42688_startAccelMeasure(icm42688_struct *s,uint8_t mode);

  /**
   * @fn startTempMeasure
   * @brief Start thermometer
   */
void icm42688_startTempMeasure(icm42688_struct *s);


  /**
   * @fn setFIFODataMode
   * @brief Set FIFO data packet format
   */
void icm42688_setFIFODataMode(icm42688_struct *s);
