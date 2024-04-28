#include <stdint.h>

  
  /**
   * @struct sSignalPathReset_t
   * @brief  Register:SIGNAL_PATH_RESET
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- ---
   * @n        |    b7    |      b6      |         b5        |         b4         |          b3         |        b2       |       b1       |      b0    |
   * @n        ------------------------------------------------------------------------------------------------------------------------------------------
   * @n        | Reserved | DMP_INIT_EN  |  DMP_MEM_RESET_EN |      Reserved      |   ABORT_AND_RESET   |   TMST_STROBE   |   FIFO_FLUSH   |  Reserved  |
   * @n        ------------------------------------------------------------------------------------------------------------------------------------------
   * @n        DMP_INIT_EN: When this bit is set to 1, the DMP is enabled
   * @n        DMP_MEM_RESET_EN: When this bit is set to 1, the DMP memory is reset
   * @n        ABORT_AND_RESET : When this bit is set to 1, the signal path is reset by restarting the ODR counter and signal path controls
   * @n        TMST_STROBE : When this bit is set to 1, the time stamp counter is latched into the time stamp register. This is a write on clear bit.
   * @n        FIFO_FLUSH : When set to 1, FIFO will get flushed.
   */
  typedef struct {
    uint8_t   reserved0: 1; 
    uint8_t   FIFOFlush: 1; 
    uint8_t   TMSTStrobe: 1; 
    uint8_t   abortAndReset: 1; 
    uint8_t   reserved4:1; 
    uint8_t   DMPMemResetEn:1;
    uint8_t   DMPInitEn:1;
    uint8_t   reserved7:1;
  } __attribute__ ((packed)) sSignalPathReset_t;
  /**
   * @struct sAPEXConfig0_t
   * @brief  Register:APEX_Config0
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- --------------------
   * @n        |            b7          |        b6      |         b5         |         b4         |          b3        |        b2      |       b1        |      b0     |
   * @n        -----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |     DMP_POWER_SAVE     |   TAP_ENABLE   |     PED_ENABLE     |     TILT_ENABLE    |        R2W_EN      |    Reserved    |             DMP_ODR           |
   * @n        -----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        DMP_POWER_SAVE : 0: DMP power save mode not active
   * @n                         1: DMP power save mode active (default)
   * @n        TAP_ENABLE :0: Tap Detection not enabled (tap detection function)
   * @n                    1: Tap Detection enabled when accelerometer ODR is set to one of the ODR 
   * @n                    values supported by Tap Detection (200Hz, 500Hz, 1kHz)
   * @n        PED_ENABLE  0: Pedometer not enabled
   * @n                    1: Pedometer enabled (Pedometer)
   * @n        TILT_ENABLE 0: Tilt Detection not enabled (tilt)
   * @n                    1: Tilt Detection enabled
   * @n        R2W_EN 0: Raise to Wake/Sleep not enabled
   * @n               1: Raise to Wake/Sleep enabled
   * @n        DMP_ODR 00: 25Hz
   * @n                01: Reserved
   * @n                10: 50Hz
   * @n                11: Reserved
   */
  typedef struct {
    uint8_t   dmpODR: 2; 
    uint8_t   reserved: 1; 
    uint8_t   R2WEn: 1; 
    uint8_t   tiltEnable:1; 
    uint8_t   PEDEnable:1;
    uint8_t   tapEnable:1;
    uint8_t   DMPPowerSave:1;
  } __attribute__ ((packed)) sAPEXConfig0_t;

  /**
   * @struct sAccelConfig0_t
   * @brief  Register:Accel_Config0
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- -------------------
   * @n        |            b7          |        b6       |         b5         |         b4         |          b3        |       b2       |       b1       |      b0    |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |                           ACCEL_FS_SEL                        |      Reserved      |                                ACCEL_ODR                          |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        ACCEL_FS_SEL :Full scale select for accelerometer UI interface output
   * @n                      000: ±16g (default)
   * @n                      001: ±8g
   * @n                      010: ±4g
   * @n                      011: ±2g
   * @n                      100: Reserved
   * @n                      101: Reserved
   * @n                      110: Reserved
   * @n                      111: Reserved
   * @n        ACCEL_ODR :Accelerometer ODR selection for UI interface output
   * @n                   0000: Reserved
   * @n                   0001: 32kHz (LN mode)
   * @n                   0010: 16kHz (LN mode)
   * @n                   0011: 8kHz (LN mode)
   * @n                   0100: 4kHz (LN mode)
   * @n                   0101: 2kHz (LN mode)
   * @n                   0110: 1kHz (LN mode) (default)
   * @n                   0111: 200Hz (LP or LN mode) 
   * @n                   1000: 100Hz (LP or LN mode)
   * @n                   1001: 50Hz (LP or LN mode)
   * @n                   1010: 25Hz (LP or LN mode)
   * @n                   1011: 12.5Hz (LP or LN mode)
   * @n                   1100: 6.25Hz (LP mode)
   * @n                   1101: 3.125Hz (LP mode)
   * @n                   1110: 1.5625Hz (LP mode)
   * @n                   1111: 500Hz (LP or LN mode)
   */
  typedef struct {
    uint8_t   accelODR: 4; 
    uint8_t   reserved: 1; 
    uint8_t   accelFsSel: 3; 
  } __attribute__ ((packed)) sAccelConfig0_t;

  /**
   * @struct sGyroConfig0_t
   * @brief  Register:Gyro_Config0
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- -------------------
   * @n        |            b7          |        b6       |         b5         |         b4         |          b3        |       b2       |       b1       |      b0    |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |                            GYRO_FS_SEL                        |      Reserved      |                                 GYRO_ODR                          |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        GYRO_FS_SEL : Full scale select for gyroscope UI interface output
   * @n                      000: ±2000dps (default)
   * @n                      001: ±1000dps
   * @n                      010: ±500dps
   * @n                      011: ±250dps
   * @n                      100: ±125dps
   * @n                      101: ±62.5dps
   * @n                      110: ±31.25dps
   * @n                      111: ±15.625dps
   * @n        GYRO_ODR :Gyroscope ODR selection for UI interface output
   * @n                  0000: Reserved
   * @n                  0001: 32kHz
   * @n                  0010: 16kHz
   * @n                  0011: 8kHz
   * @n                  0100: 4kHz
   * @n                  0101: 2kHz
   * @n                  0110: 1kHz (default)
   * @n                  0111: 200Hz 
   * @n                  1000: 100Hz
   * @n                  1001: 50Hz
   * @n                  1010: 25Hz
   * @n                  1011: 12.5Hz
   * @n                  1100: Reserved
   * @n                  1101: Reserved
   * @n                  1110: Reserved
   * @n                  1111: 500Hz
   */
  typedef struct {
    uint8_t   gyroODR: 4; 
    uint8_t   reserved: 1; 
    uint8_t   gyroFsSel: 3; 
  } __attribute__ ((packed)) sGyroConfig0_t;

  /**
   * @struct sGyroConfig1_t
   * @brief  Register:Gyro_Config1
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- -------------------
   * @n        |            b7          |        b6       |         b5         |         b4         |          b3        |       b2       |       b1       |      b0    |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |                          TEMP_FILT_BW                         |     reserved       |            GYRO_UI_FILT_ORD         |      GYRO_DEC2_M2_ORD       |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        TEMP_FILT_BW : Set the bandwidth of the temperature signal DLPF
   * @n                      000: DLPF BW = 4000Hz; DLPF Latency = 0.125ms (default)
   * @n                      001: DLPF BW = 170Hz; DLPF Latency = 1ms
   * @n                      010: DLPF BW = 82Hz; DLPF Latency = 2ms
   * @n                      011: DLPF BW = 40Hz; DLPF Latency = 4ms
   * @n                      100: DLPF BW = 20Hz; DLPF Latency = 8ms
   * @n                      101: DLPF BW = 10Hz; DLPF Latency = 16ms
   * @n                      110: DLPF BW = 5Hz; DLPF Latency = 32ms
   * @n                      111: DLPF BW = 5Hz; DLPF Latency = 32ms
   * @n        GYRO_UI_FILT_ORD :Selects order of GYRO UI filter
   * @n                           00: 1st Order
   * @n                           01: 2nd Order
   * @n                           10: 3rd Order
   * @n                           11: Reserved
   * @n        GYRO_DEC2_M2_ORD :Selects order of GYRO DEC2_M2 Filter
   * @n                          00: Reserved
   * @n                          01: Reserved
   * @n                          10: 3rd Order
   * @n                          11: Reserved
   */
  typedef struct {
    uint8_t   gyroDec2M2ODR: 2; 
    uint8_t   gyroUIFiltODR: 2; 
    uint8_t   reserved: 1; 
    uint8_t   agyroFiltBW: 3; 
  } __attribute__ ((packed)) sGyroConfig1_t;

  /**
   * @struct sGyroConfig1_t
   * @brief  Register:Gyro_Config1
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- -------------------
   * @n        |            b7          |        b6       |         b5         |         b4         |          b3        |       b2       |       b1       |      b0    |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |                    Reserved              |      TEMP_DIS      |        IDLE        |                GYRO_MODE            |           ACCEL_MODE        |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        TEMP_DIS : 0: Temperature sensor is enabled (default)
   * @n                   1: Temperature sensor is disabled
   * @n        IDLE : 0: When the accelerometer and gyroscope are powered off, the chip will go to OFF state, since the RC oscillator will also be powered off
   * @n               1: RC oscillator is powered on even if the accelerometer and gyroscope are powered off
   * @n        GYRO_MODE :00: Turns gyroscope off (default)
   * @n                   01: Places gyroscope in Standby Mode
   * @n                   10: Reserved
   * @n                   11: Places gyroscope in Low Noise (LN) Mode
   * @n                   Gyroscope needs to be kept ON for a minimum of 45ms. When transitioning from OFF to any of the other modes, do not issue any register writes for 200µs
   * @n        ACCEL_MODE: 00: Turns accelerometer off (default)
   * @n                    01: Turns accelerometer off
   * @n                    10: Places accelerometer in Low Power (LP) Mode
   * @n                    11: Places accelerometer in Low Noise (LN) Mode                
   * @n                    When transitioning from OFF to any of the other modes, do not issue any register writes for 200µs
   */
  typedef struct {
    uint8_t   accelMode: 2; 
    uint8_t   gyroMode: 2; 
    uint8_t   idle: 1; 
    uint8_t   tempDis:1; 
    uint8_t   reserved:2;
  } __attribute__ ((packed)) sPWRMgmt0_t;

  /**
   * @struct sINTFConfig0_t
   * @brief  Register:INTF_Config0
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- ---------------------
   * @n        |            b7          |        b6       |         b5         |         b4         |          b3         |        b2       |       b1       |      b0    |
   * @n        ------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        | FIFO_HOLD_LAST_DATA_EN | FIFO_COUNT_REC  |  FIFO_COUNT_ENDIAN | SENSOR_DATA_ENDIAN |                   Reserved            |          UI_SIFS_CFG        |
   * @n        ------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        FIFO_HOLD_LAST_DATA_EN: Method for processing invalid data, 0: Registers do not receive invalid samples. Registers hold last valid sample until new one arrives
   * @n                                                           FIFO : 16-bit FIFO packet: Same as Sense Registers (Do not receive invalid samples. They hold the last valid sample. Repeated reading before new sample received will yield copies of the last valid sample)
   * @n                                                                  20-bit FIFO packet: Invalid samples are indicated with the value -524288
   * @n                                                        1: Not receive invalid samples. Registers hold last valid sample until new one arrives
   * @n        FIFO_COUNT_REC: 0: FIFO count is reported in bytes
   * @n                        1: FIFO count is reported in records
   * @n        FIFO_COUNT_ENDIAN : 0: FIFO count is reported in Little Endian format
   * @n                            1: FIFO count is reported in Big Endian format (default)
   * @n        SENSOR_DATA_ENDIAN : 0: Sensor data is reported in Little Endian format
   * @n                             1: Sensor data is reported in Big Endian format (default)
   * @n        UI_SIFS_CFG: 0x: Reserved
   * @n                     10: Disable SPI
   * @n                     11: Disable I2C
   */
  typedef struct {
    uint8_t   UISifsConfig: 2; 
    uint8_t   reserved:2;
    uint8_t   sensorDataEndian: 1; 
    uint8_t   FIFOCountEndian: 1; 
    uint8_t   FIFOCountRec:1; 
    uint8_t   FIFOInvalidSampleTreatment:1;
  } __attribute__ ((packed)) sINTFConfig0_t;


  /**
   * @struct sINTFConfig1_t
   * @brief  Register:INTF_Config1
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- -------------------
   * @n        |            b7          |        b6       |         b5         |         b4         |          b3        |       b2       |       b1       |      b0    |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |                                       Reserved                                     |  ACCEL_LP_CLK_SEL  |    RTC_MODE    |              CLKSEL         |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        ACCEL_LP_CLK_SEL : 0: Accelerometer LP mode uses Wake Up oscillator clock
   * @n                           1: Accelerometer LP mode uses RC oscillator clock
   * @n        RTC_MODE : 0: No input RTC clock is required
   * @n                   1: RTC clock input is required
   * @n        CLKSEL : 00: Always select internal RC oscillator
   * @n                 01: Select PLL when available, else select RC oscillator (default)
   * @n                 10: Reserved
   * @n                 11: Disable all clocks
   */
  typedef struct {
    uint8_t   clksel: 2; 
    uint8_t   rtcMode: 1; 
    uint8_t   accelLpClkSel: 1; 
    uint8_t   reserved:4;
  } __attribute__ ((packed)) sINTFConfig1_t;

  /**
   * @struct sAccelConfig1_t
   * @brief  Register:Accel_Config1
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- -------------------
   * @n        |            b7          |        b6       |         b5         |         b4         |          b3        |       b2       |       b1       |      b0    |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |                            Reserved                           |          ACCEL_UI_FILT_ORD              |         ACCEL_DEC2_M2_ORD       |  Reserved  |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        ACCEL_UI_FILT_ORD : Selects order of ACCEL UI filter
   * @n                            00: 1st Order
   * @n                            01: 2nd Order
   * @n                            10: 3rd Order
   * @n                            11: Reserved
   * @n        ACCEL_DEC2_M2_ORD : Order of Accelerometer DEC2_M2 filter
   * @n                            00: Reserved
   * @n                            01: Reserved
   * @n                            10: 3rd order
   * @n                            11: Reserved
   */
  typedef struct {
    uint8_t   reserved: 1; 
    uint8_t   accelDec2M2ORD: 2; 
    uint8_t   accelUIFiltORD: 2; 
    uint8_t   reserved2: 3;
  } __attribute__ ((packed)) sAccelConfig1_t;

  /**
   * @struct sGyroAccelConfig0_t
   * @brief  Register:Gyro_Accel_Config0
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- -------------------
   * @n        |            b7          |        b6       |         b5         |         b4         |          b3        |       b2       |       b1       |      b0    |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |                                       ACCEL_UI_FILT_BW                             |                             GYRO_UI_FILT_BW                       |
   * @n        ----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        ACCEL_UI_FILT_BW : Bandwidth for Accel LPF
   * @n                           LN Mode:
   * @n                              0 BW=ODR/2
   * @n                              1 BW=max(400Hz, ODR)/4 (default)
   * @n                              2 BW=max(400Hz, ODR)/5
   * @n                              3 BW=max(400Hz, ODR)/8
   * @n                              4 BW=max(400Hz, ODR)/10
   * @n                              5 BW=max(400Hz, ODR)/16
   * @n                              6 BW=max(400Hz, ODR)/20
   * @n                              7 BW=max(400Hz, ODR)/40
   * @n                              8 to 13: Reserved
   * @n                              14 Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 
   * @n                              runs at max(400Hz, ODR) 
   * @n                              15 Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 
   * @n                              runs at max(200Hz, 8*ODR)
   * @n                            LP Mode:
   * @n                              0 Reserved
   * @n                              1 1x AVG filter (default)
   * @n                              2 to 5 Reserved
   * @n                              6 16x AVG filter
   * @n                              7 to 15 Reserved
   * @n        GYRO_UI_FILT_BW :Bandwidth for Gyro LPF
   * @n                           LN Mode:
   * @n                            0 BW=ODR/2
   * @n                            1 BW=max(400Hz, ODR)/4 (default)
   * @n                            2 BW=max(400Hz, ODR)/5
   * @n                            3 BW=max(400Hz, ODR)/8
   * @n                            4 BW=max(400Hz, ODR)/10
   * @n                            5 BW=max(400Hz, ODR)/16
   * @n                            6 BW=max(400Hz, ODR)/20
   * @n                            7 BW=max(400Hz, ODR)/40
   * @n                            8 to 13: Reserved
   * @n                            14 Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(400Hz, ODR) 
   * @n                            15 Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(200Hz, 8*ODR)
   */
  typedef struct {
    uint8_t   gyroUIFiltBW: 4; 
    uint8_t   accelUIFiltBW: 4; 
  } __attribute__ ((packed)) sGyroAccelConfig0_t;

  /**
   * @struct sAPEXConfig7_t
   * @brief  Register:APEX_Config7
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- --------------------------------------------------
   * @n        |            b7        |           b6         |           b5           |           b4           |           b3           |          b2        |          b1        |        b0          |
   * @n        -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |                                                      TAP_MIN_JERK_THR                                                                       |             TAP_MAX_PEAK_TOL            |
   * @n        -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        TAP_MIN_JERK_THR  Tap detection minimum jerk threshold, use default value 010001b
   * @n        TAP_MAX_PEAK_TOL  Point detection maximum peak tolerance, use default value 01b
   */
  typedef struct {
    uint8_t   tapMaxPeakTol: 2; 
    uint8_t   tapMinJerkThr: 6; 
  } __attribute__ ((packed)) sAPEXConfig7_t;

  /**
   * @struct sAPEXConfig8_t
   * @brief  Register:APEX_Config8
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- --------------------------------------------------
   * @n        |            b7        |           b6         |           b5           |           b4           |           b3           |          b2        |          b1        |        b0          |
   * @n        -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |        Reserved      |                   TAP_TMAX                    |                     TAP_TAVG                    |                           TAP_TMIN                           |
   * @n        -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        TAP_TMAX  Tap measurement window (number of samples), use default value 01b
   * @n        TAP_TAVG  Tap energy measurement window (number of samples), use default value 01b
   * @n        TAP_TMIN  Single tap window (number of samples), use default value 011b
   */
  typedef struct {
    uint8_t   tapTmin: 3; 
    uint8_t   tapTavg: 2; 
    uint8_t   tapTmax: 2;
    uint8_t   reserved:1;
  } __attribute__ ((packed)) sAPEXConfig8_t;

  /**
   * @struct sAPEXConfig4_t
   * @brief  Register:APEX_Config4
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- --------------------------------------------------
   * @n        |            b7        |           b6         |           b5           |           b4           |           b3           |          b2        |          b1        |        b0          |
   * @n        -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |           TILT_WAIT_TIME_SEL                |                                 SLEEP_TIME_OUT                           |                              Reserved                        |
   * @n        -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        TILT_WAIT_TIME_SEL  Config delay time after tilt is detected before interrupt is triggered
   * @n                            00: 0s
   * @n                            01: 2s
   * @n                            10: 4s (default)
   * @n                            11: 6s
   * @n        SLEEP_TIME_OUT  Configures the time for sleep detection, for Raise to Wake/Sleep
   * @n                        000: 1.28sec
   * @n                        001: 2.56sec
   * @n                        010: 3.84sec
   * @n                        011: 5.12sec
   * @n                        100: 6.40sec
   * @n                        101: 7.68sec
   * @n                        110: 8.96sec
   * @n                        111: 10.24sec
   */
  typedef struct {
    uint8_t   reserved: 3;
    uint8_t   sleepTimeOut: 3; 
    uint8_t   tiltWaitTimeSel: 2; 
  } __attribute__ ((packed)) sAPEXConfig4_t;

  /**
   * @struct sSMDConfig_t
   * @brief  Register:SMD_Config
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- --------------------
   * @n        |            b7        |        b6      |         b5         |         b4         |          b3        |        b2      |       b1        |      b0       |
   * @n        -----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |                                    Reserved                                     |     WOM_INT_MODE   |    WOM_MODE    |             SMD_MODE            |
   * @n        -----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        WOM_INT_MODE : (Wake on motion) 0: Set WoM interrupt on the OR of all enabled accelerometer thresholds
   * @n                                 1: Set WoM interrupt on the AND of all enabled accelerometer threshold
   * @n        WOM_MODE 0: Initial sample is stored. Future samples are compared to initial sample
   * @n                 1: Compare current sample to previous sample
   * @n        SMD_MODE  00: SMD disabled (important motion detector)
   * @n                  01: Reserved
   * @n                  10: SMD short (1 sec wait) An SMD event is detected when two WOM are detected 1 sec apart
   * @n                  11: SMD long (3 sec wait) An SMD event is detected when two WOM are detected 3 sec apart
   */
  typedef struct {
    uint8_t   SMDMode: 2; 
    uint8_t   WOMMode: 1; 
    uint8_t   WOMIntMode: 1;
    uint8_t   reserved: 4;
  } __attribute__ ((packed)) sSMDConfig_t;


  /**
   * @struct sGyroConfigStatic9_t
   * @brief  Register:Gyro_Config_Static9
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- --------------------------------------------------
   * @n        |            b7        |           b6         |           b5           |           b4           |           b3           |          b2        |          b1        |        b0          |
   * @n        -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |                  Reserved                   | GYRO_Z_NF_COSWZ_SEL[0] | GYRO_Y_NF_COSWZ_SEL[0] | GYRO_X_NF_COSWZ_SEL[0] | GYRO_Z_NF_COSWZ[8] | GYRO_Y_NF_COSWZ[8] | GYRO_X_NF_COSWZ[8] |
   * @n        -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        GYRO_Z_NF_COSWZ_SEL[0] :Used for gyroscope Z-axis notch filter frequency selection
   * @n        GYRO_Y_NF_COSWZ_SEL[0] :Used for gyroscope Y-axis notch filter frequency selection
   * @n        GYRO_X_NF_COSWZ_SEL[0] :Used for gyroscope X-axis notch filter frequency selection
   * @n        GYRO_Z_NF_COSWZ[8] :Used for gyroscope Z-axis notch filter frequency selection
   * @n        GYRO_Y_NF_COSWZ[8] :Used for gyroscope Y-axis notch filter frequency selection
   * @n        GYRO_X_NF_COSWZ[8] :Used for gyroscope X-axis notch filter frequency selection
   */
  typedef struct {
    uint8_t   gyroNFCoswzX8: 1; 
    uint8_t   gyroNFCoswzY8: 1; 
    uint8_t   gyroNFCoswzZ8: 1;
    uint8_t   gyroNFCoswzSelX: 1;
    uint8_t   gyroNFCoswzSelY: 1;
    uint8_t   gyroNFCoswzSelZ: 1;
    uint8_t   reserved:2;
  } __attribute__ ((packed)) sGyroConfigStatic9_t;

  /**
   * @struct sGyroConfigStatic2_t
   * @brief  Register:Gyro_Config_Static2
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- ---------------------------------------
   * @n        |            b7        |           b6         |           b5          |         b4         |          b3       |         b2       |          b1         |        b0          |
   * @n        ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |                                                                     Reserved                                                    |     GYRO_AAF_DIS    |     GYRO_NF_DIS    |
   * @n        ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        GYRO_AAF_DIS :0: Enable gyroscope anti-aliasing filter (default)
   * @n                      1: Disable gyroscope anti-aliasing filter
   * @n        GYRO_NF_DIS   0: Enable Notch Filter (default)
   * @n                      1: Disable Notch Filter
   */
  typedef struct {
    uint8_t   gyroNFDis: 1; 
    uint8_t   gyroAAFDis: 1; 
    uint8_t   reserved: 6;
  } __attribute__ ((packed)) sGyroConfigStatic2_t;

  /**
   * @struct sGyroConfigStatic5_t
   * @brief  Register:Gyro_Config_Static5
   * @n         -------------------------------------------------------------------------------------------------------------------------------------- ---------------------------------------
   * @n         |            b7        |           b6         |           b5          |         b4         |          b3       |         b2       |          b1         |        b0          |
   * @n         ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n         |                                     GYRO_AAF_BITSHIFT                                    |                            GYRO_AAF_DELTSQR[11:8]                               |
   * @n         ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n         GYRO_AAF_BITSHIFT :Controls bandwidth of the gyroscope anti-alias filter
   * @n         GYRO_AAF_DELTSQR[11:8]   Controls bandwidth of the gyroscope anti-alias filter
  */
  typedef struct {
    uint8_t   gyroAAFDeltsqr: 4; 
    uint8_t   gyroAAFBitshift: 4; 
  } __attribute__ ((packed)) sGyroConfigStatic5_t;

  /**
   * @struct sAccelConfigStatic2_t
   * @brief  Register:Accel_Config_Static2
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- --------------------------------------------------
   * @n        |            b7        |           b6         |           b5           |           b4           |           b3           |          b2        |          b1        |        b0          |
   * @n        -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |       Reserved       |                                                          ACCEL_AAF_DELT                                                                   |    ACCEL_AAF_DIS   |
   * @n        -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        ACCEL_AAF_DELT  Controls bandwidth of the accelerometer anti-alias filter
   * @n        ACCEL_AAF_DIS   0: Enable accelerometer anti-aliasing filter (default)
   * @n                        1: Disable accelerometer anti-aliasing filter
   */
  typedef struct {
    uint8_t   accelAAFDis: 1; 
    uint8_t   accelAAFDelt: 6;
    uint8_t   reserved: 1; 
  } __attribute__ ((packed)) sAccelConfigStatic2_t;

  /**
   * @struct sAccelConfigStatic4_t
   * @brief  Register:Accel_Config_Static4
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- --------------------------------------------------
   * @n        |            b7        |           b6         |           b5           |           b4           |           b3           |          b2        |          b1        |        b0          |
   * @n        -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |                                  ACCEL_AAF_BITSHIFT                                           |                               ACCEL_AAF_DELTSQR[11:8]                                 |
   * @n        -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        ACCEL_AAF_BITSHIFT  Controls bandwidth of the accelerometer anti-alias filter
   * @n        ACCEL_AAF_DELTSQR[11:8]  Controls bandwidth of the accelerometer anti-alias filter
   */
  typedef struct {
    uint8_t   accelAAFDeltsqr: 4; 
    uint8_t   accelAAFBitshift: 4; 
  } __attribute__ ((packed)) sAccelConfigStatic4_t;


  /**
   * @struct sFIFOConfig1_t
   * @brief  Register:FIFO_Config1
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- ---------------------------
   * @n        |            b7        |           b6           |          b5         |        b4         |          b3        |        b2      |      b1        |      b0       |
   * @n        ------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |       Reserved       | FIFO_RESUME_PARTIAL_RD |    FIFO_WM_GT_TH    |   FIFO_HIRES_EN   | FIFO_TMST_FSYNC_EN |  FIFO_TEMP_EN  |  FIFO_GYRO_EN  | FIFO_ACCEL_EN |
   * @n        ------------------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        FIFO_RESUME_PARTIAL_RD : 0: Partial FIFO read is disabled and the entire FIFO needs to be re-read
   * @n                                 1: FIFO read can be partial, and resume from last read point
   * @n        FIFO_WM_GT_TH Trigger FIFO watermark interrupt on every ODR (DMA write) if FIFO_COUNT ≥ FIFO_WM_TH
   * @n        FIFO_HIRES_EN : FIFO_HIRES_EN Enable 3 bytes of extended 20-bits accel, gyro data  +  1 byte of extended 16-bit temperature sensor data to be placed into the FIFO
   * @n        FIFO_TMST_FSYNC_EN: Must be set to 1 for all FIFO use cases when FSYNC is used.
   * @n        FIFO_TEMP_EN  Enable temperature sensor packets to go to FIFO
   * @n        FIFO_GYRO_EN  Enable gyroscope packets to go to FIFO
   * @n        FIFO_ACCEL_EN  Enable accelerometer packets to go to FIFO
   */
  typedef struct {
    uint8_t   FIFOAccelEn: 1; 
    uint8_t   FIFOGyroEn: 1; 
    uint8_t   FIFOTempEn: 1;
    uint8_t   FIFOTmstFsyncEn: 1;
    uint8_t   FIFOHiresEn: 1;
    uint8_t   FIFOWmGtTh: 1;
    uint8_t   FIFOResumeParialRd:1;
    uint8_t   reseved :1;
  } __attribute__ ((packed)) sFIFOConfig1_t;

  /**
   * @struct sTMSTConfig_t
   * @brief  Register:TMST_Config
   * @n        -------------------------------------------------------------------------------------------------------------------------------------- --------------------
   * @n        |            b7          |        b6       |         b5         |         b4         |          b3        |        b2       |       b1        |      b0   |
   * @n        -----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |                            Reserved                           |   TMST_TO_REGS_EN  |       TMST_RES     |  TMST_DELTA_EN  |  TMST_FSYNC_EN  |  TMST_EN  |
   * @n        -----------------------------------------------------------------------------------------------------------------------------------------------------------
   * @n        TMST_TO_REGS_EN : 0: TMST_VALUE[19:0] read always returns 0s
   * @n                          1: TMST_VALUE[19:0] read returns timestamp value
   * @n        TMST_RES : Time Stamp resolution: When set to 0 (default), time stamp resolution is 1 µs. When set to 1, resolution is 16µs
   * @n        TMST_DELTA_EN  Time Stamp delta enable: When set to 1, the time stamp field contains the measurement of time since the last occurrence of ODR.
   * @n        TMST_FSYNC_EN  Time Stamp register FSYNC enable (default). When set to 1, the contents of the Timestamp feature of FSYNC is enabled. The user also needs to select FIFO_TMST_FSYNC_EN in order to propagate the timestamp value to the FIFO.
   * @n        TMST_EN 0: Time Stamp register disable
   * @n                1: Time Stamp register enable (default)
   */
  typedef struct {
    uint8_t   TimeStampEn: 1; 
    uint8_t   TimeStampFSYNCEn: 1; 
    uint8_t   TimeStampDeltaEn: 1;
    uint8_t   TimeStampRes: 1;
    uint8_t   TimeStampToRegsEn: 1;
    uint8_t   reseved :3;
  } __attribute__ ((packed)) sTMSTConfig_t;

  /**
   * @struct sINTConfig_t
   * @brief  Register:INT_Config
   * @n        -------------------------------------------------------------------------------------------------------------------------------
   * @n        |    b7    |    b6    |       b5        |         b4         |       b3      |     b2    |         b1         |    b0         |
   * @n        -------------------------------------------------------------------------------------------------------------------------------
   * @n        |      reversed       |    INT2_MODE    | INT2_DRIVE_CIRCUIT | INT2_POLARITY | INT1_MODE | INT1_DRIVE_CIRCUIT | INT1_POLARITY |
   * @n        -------------------------------------------------------------------------------------------------------------------------------
   * @n        INT2_MODE:INT2 interrupt mode
   * @n                    0: Pulsed mode
   * @n                    1: Latched mode
   * @n        INT2_DRIVE_CIRCUIT:INT2 drive circuit
   * @n                           0: Open drain
   * @n                           1: Push pull
   * @n        INT2_POLARITY:INT2 interrupt polarity
   * @n                           0: Active low (default)
   * @n                           1: Active high
   * @n        INT1_MODE:INT1 interrupt mode
   * @n                    0: Pulsed mode
   * @n                    1: Latched mode
   * @n        INT1_DRIVE_CIRCUIT:INT1 drive circuit
   * @n                           0: Open drain
   * @n                           1: Push pull
   * @n        INT1_POLARITY:INT1 interrupt polarity
   * @n                           0: Active low (default)
   * @n                           1: Active high
   */
  typedef struct {
    uint8_t   INT1Polarity: 1;
    uint8_t   INT1DriveCirCuit: 1;
    uint8_t   INT1Mode: 1;
    uint8_t   INT2Polarity: 1;
    uint8_t   INT2DriveCirCuit: 1;
    uint8_t   INT2Mode :1;
    uint8_t   reversed :2;
  } __attribute__ ((packed)) sINTConfig_t;

  /**
   * @struct sINTSource_t
   * @brief  Register:INT_Source
   * @n        ---------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |    b7    |    b6    |            b5         |          b4         |        b3       |       b2        |         b1       |     b0         |
   * @n        ---------------------------------------------------------------------------------------------------------------------------------------------
   * @n        |      reversed       |    STEP_DET_INT_EN    | STEP_CNT_OFL_INT_EN | TILT_DET_INT_EN | WAKE_DET_INT_EN | SLEEP_DET_INT_EN | TAP_DET_INT_EN |
   * @n        ---------------------------------------------------------------------------------------------------------------------------------------------
   */
  typedef struct {
    uint8_t   tapDetIntEn: 1;
    uint8_t   sleepDetIntEn: 1;
    uint8_t   wakeDetIntEn: 1;
    uint8_t   tiltDetIntEn: 1;
    uint8_t   stepCntOflIntEn :1;
    uint8_t   stepDetIntEn :1;
    uint8_t   reserved:2;
  } __attribute__ ((packed)) sINTSource_t;