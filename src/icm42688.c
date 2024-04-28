#include "../inc/icm42688.h"
#include <zephyr/drivers/spi.h>
#include "stdint.h"

#define DBG(x) printk x



#define SPI1_NODE DT_NODELABEL(spi2)
static const struct device *spi1_dev = DEVICE_DT_GET(SPI1_NODE);

#define MY_GPIO1 DT_NODELABEL(gpio0)

#define GPIO_PIN_CS 10
const struct device *gpio1_dev = DEVICE_DT_GET(MY_GPIO1);



struct spi_config spi_cfg = {
    .frequency = 4000000,                 // Initializing the 'frequency' field with value 4000000
    .operation = SPI_OP_MODE_MASTER |SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE, // Initializing the 'operation' field with bitwise OR of SPI_OP_MODE_MASTER and SPI_TRANSFER_MSB
    .slave = 0,                    // Initializing the 'slave' field with value of SLAVE_ID
    //.cs = CS_PIN,                         // Initializing the 'cs' field with value of CS_PIN
};





void icm42688(icm42688_struct *s){
  s->accelConfig0.accelODR = 6;
  s->accelConfig0.accelFsSel = 0;
  s->gyroConfig0.gyroODR = 6;
  s->gyroConfig0.gyroFsSel = 0;
  s->_gyroRange = 4000/65535.0;
  s->_accelRange = 0.488f;
  s->FIFOMode = false;
}



int icm42688_spi_init(icm42688_struct* s) {

  gpio_pin_configure(gpio1_dev,GPIO_PIN_CS,GPIO_OUTPUT);
  gpio_pin_set(gpio1_dev,GPIO_PIN_CS,1);


  if (!device_is_ready(spi1_dev)){
    printk("spi not ready \r\n");
    return;
  }
  else { printk(" spi ok \r\n");}

}


int icm42688_spi_begin(icm42688_struct *s) {
   struct spi_cs_control cs_ctrl;


   // cs_ctrl.gpio.port = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
   // cs_ctrl.gpio.pin = SPI_SLAVE_PIN;
   // cs_ctrl.gpio.dt_flags = GPIO_ACTIVE_LOW;
    //spi_configure(spi_dev, &spi_cfg);
  //  gpio_pin_configure(spi_dev, icm->cs_pin, GPIO_OUTPUT_ACTIVE);
  //  gpio_pin_set(spi1_dev,CS_PIN, 1);
    return s->begin(s); // Appel à votre fonction ICM42688::begin() modifiée pour fonctionner dans l'environnement Zephyr.
}

void icm42688_writeReg(uint8_t reg, void* pBuf, size_t size) {
    __ASSERT_NO_MSG(pBuf != NULL);

    uint8_t * _pBuf = (uint8_t *)pBuf;


     struct spi_buf tx_buf = {
        .buf = &reg,
        .len = 1,
    };

     struct spi_buf rx_buf = {
        .buf = NULL,
        .len = 0,
    };

    const struct spi_buf_set tx_bufs = {
        .buffers = &tx_buf,
        .count = 1,
    };

    const struct spi_buf_set rx_bufs = {
        .buffers = &rx_buf,
        .count = 1,
    };

    int err = spi_transceive(spi1_dev, &spi_cfg, &tx_bufs, &rx_bufs);
    if (err) {
        printk("SPI transceive failed: %d\n", err);
        return;
    }

    tx_buf.buf = _pBuf;
    tx_buf.len = size;

    err = spi_transceive(spi1_dev, &spi_cfg, &tx_bufs, NULL);
    if (err) {
        printk("SPI transceive failed: %d\n", err);
        return;
    }

}


uint8_t icm42688_readReg(icm42688_struct *s,uint8_t reg, void* pBuf, uint8_t size) {


 uint8_t * _pBuf = (uint8_t *)pBuf;


    int err;
    uint8_t tx_buffer[1];
    tx_buffer[0]=reg;
   // size_t count = 0;


    struct spi_buf tx_spi_bufs[] ={
      {.buf = tx_buffer, .len = sizeof(tx_buffer)},
    };

    struct spi_buf_set spi_tx_buffer_set = {
        .buffers = &tx_spi_bufs,
        .count = 1,
    };

    struct spi_buf rx_spi_bufs[] = {
      { .buf = _pBuf, .len = size},
    };

    struct spi_buf_set spi_rx_buffer_set = {
        .buffers = &rx_spi_bufs,
        .count = 1,
    };


    gpio_pin_set(gpio1_dev,GPIO_PIN_CS,0);

  do{
   err = spi_write(spi1_dev, &spi_cfg, &spi_tx_buffer_set);
   // err =  spi_transceive(spi1_dev, &spi_cfg, &spi_tx_buffer_set, &spi_rx_buffer_set);
    if (err < 0) { break;}
   err = spi_read(spi1_dev, &spi_cfg, &spi_rx_buffer_set);
  }while(false);

    if (err < 0) { printk("read register failed \r\n");}

    gpio_pin_set(gpio1_dev,GPIO_PIN_CS,1);

    return size;
}



int icm42688_begin(icm42688_struct *s) {
    uint8_t bank = 0;

    s->writeReg(ICM42688_REG_BANK_SEL, &bank, 1);

    uint8_t id = 0;
    if (s->readReg(s,ICM42688_WHO_AM_I, &id, 1) != 1) {
        // DBG("bus data access error");
        return ERR_DATA_BUS;
    }

    DBG(("real sensor id= %d \n",id));

    if (id != DFRobot_ICM42688_ID) {
        return ERR_IC_VERSION;
    }

    uint8_t reset = 0;
    s->writeReg( ICM42688_DEVICE_CONFIG, &reset, 1);
    k_msleep(2);

    return ERR_OK;
}




float icm42688_getTemperature(icm42688_struct *s)
{
  float value;
  if(s->FIFOMode){
    value = (s->_temp/2.07) + 25;
  } else{
    uint8_t data[2];
    int16_t value2;
    s->readReg(s,ICM42688_TEMP_DATA1, data, 2);
    value2 = ((uint16_t )data[0]<<8) | (uint16_t )data[1];
    value = value2/132.48 + 25;
  }
  return value;
}



float icm42688_getAccelDataX(icm42688_struct *s)
{
  float value;
  if(s->FIFOMode){
    value = s->_accelX;
  } 
  else{
    uint8_t data[2];
    s->readReg(s,ICM42688_ACCEL_DATA_X1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value*s->_accelRange;
}

float icm42688_getAccelDataY(icm42688_struct *s)
{
  float value;
  if(s->FIFOMode){
    value = s->_accelY;
  } else{
    uint8_t data[2];
    s->readReg(s,ICM42688_ACCEL_DATA_Y1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value*s->_accelRange;
}

float icm42688_getAccelDataZ(icm42688_struct *s)
{
  float value;
  if(s->FIFOMode){
    value = s->_accelZ;
  } else{
    uint8_t data[2];
    s->readReg(s,ICM42688_ACCEL_DATA_Z1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value*s->_accelRange;
}

float icm42688_getGyroDataX(icm42688_struct *s)
{
  float value;
  if(s->FIFOMode){
    value = s->_gyroX;
  } else{
    uint8_t data[2];
    s->readReg(s,ICM42688_GYRO_DATA_X1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value*s->_gyroRange;
}

float icm42688_getGyroDataY(icm42688_struct *s)
{
  float value;
  if(s->FIFOMode){
    value = s->_gyroY;
  } else{
    uint8_t data[2];
    s->readReg(s,ICM42688_GYRO_DATA_Y1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value*s->_gyroRange;
}

float icm42688_getGyroDataZ(icm42688_struct *s)
{
  float value;
  if(s->FIFOMode){
    value = s->_gyroZ;
  } else{
    uint8_t data[2];
    s->readReg(s,ICM42688_GYRO_DATA_Z1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value*s->_gyroRange;
}



void  icm42688_tapDetectionInit(icm42688_struct *s,uint8_t accelMode)
{
  uint8_t bank = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  if(accelMode == 0){
    s->accelConfig0.accelODR = 15;
    s->writeReg(ICM42688_ACCEL_CONFIG0,&s->accelConfig0,1);
    s->PWRMgmt0.accelMode = 2;
    s->writeReg(ICM42688_PWR_MGMT0,&s->PWRMgmt0,1);
    k_msleep(1);
    s->INTFConfig1.accelLpClkSel = 0;
    s->writeReg(ICM42688_INTF_CONFIG1,&s->INTFConfig1,1);
    s->accelConfig1.accelUIFiltORD = 2;
    s->writeReg(ICM42688_ACCEL_CONFIG1,&s->accelConfig1,1);
    s->gyroAccelConfig0.accelUIFiltBW = 0;
    s->writeReg(ICM42688_GYRO_ACCEL_CONFIG0,&s->gyroAccelConfig0,1);
  } else if(accelMode == 1){
    s->accelConfig0.accelODR = 6;
    s->writeReg(ICM42688_ACCEL_CONFIG0,&s->accelConfig0,1);
    s->PWRMgmt0.accelMode = 3;
    s->writeReg(ICM42688_PWR_MGMT0,&s->PWRMgmt0,1);
    k_msleep(1);
    s->accelConfig1.accelUIFiltORD = 2;
    s->writeReg(ICM42688_ACCEL_CONFIG1,&s->accelConfig1,1);
    s->gyroAccelConfig0.accelUIFiltBW = 0;
    s->writeReg(ICM42688_GYRO_ACCEL_CONFIG0,&s->gyroAccelConfig0,1);
  } else{
    DBG(("accelMode invalid !"));
    return;
  }
  k_msleep(1);
  bank = 4;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  s->APEXConfig8.tapTmin = 3;
  s->APEXConfig8.tapTavg = 3;
  s->APEXConfig8.tapTmax = 2;
  s->writeReg(ICM42688_APEX_CONFIG8,&s->APEXConfig8,1);
  s->APEXConfig7.tapMinJerkThr = 17;
  s->APEXConfig7.tapMaxPeakTol = 1;
  s->writeReg(ICM42688_APEX_CONFIG7,&s->APEXConfig7,1);
  k_msleep(1);
  s->INTSource.tapDetIntEn = 1;
  if(s->_INTPin==1){
    s->writeReg(ICM42688_INT_SOURCE6,&s->INTSource,1);
  } else {
    s->writeReg(ICM42688_INT_SOURCE7,&s->INTSource,1);
  }
  k_msleep(50);
  bank = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  s->APEXConfig0.tapEnable = 1;
  s->writeReg(ICM42688_APEX_CONFIG0,&s->APEXConfig0,1);
}


void icm42688_getTapInformation(icm42688_struct *s)
{
  uint8_t data;
  s->readReg(s,ICM42688_APEX_DATA4, &data, 1);
  s->_tapNum = data & 0x18;
  s->_tapAxis = data & 0x06;
  s->_tapDir = data & 0x01;
}
uint8_t  icm42688_numberOfTap(icm42688_struct *s)
{
  return s->_tapNum;
}
uint8_t  icm42688_axisOfTap(icm42688_struct *s)
{
  return s->_tapAxis;
}


void icm42688_wakeOnMotionInit(icm42688_struct *s)
{
  uint8_t bank = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  s->accelConfig0.accelODR = 9;
  s->writeReg(ICM42688_ACCEL_CONFIG0,&s->accelConfig0,1);
  s->PWRMgmt0.accelMode = 2;
  s->writeReg(ICM42688_PWR_MGMT0,&s->PWRMgmt0,1);
  k_msleep(1);
  s->INTFConfig1.accelLpClkSel = 0;
  s->writeReg(ICM42688_INTF_CONFIG1,&s->INTFConfig1,1);
  k_msleep(1);
}


void icm42688_setWOMTh(icm42688_struct *s,uint8_t axis,uint8_t threshold)
{
  uint8_t bank = 4;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  uint8_t womValue = threshold;
  if(axis == X_AXIS){
    s->writeReg(ICM42688_ACCEL_WOM_X_THR,&womValue,1);
  } else if(axis == Y_AXIS){
    s->writeReg(ICM42688_ACCEL_WOM_Y_THR,&womValue,1);
  } else if(axis == Z_AXIS){
    s->writeReg(ICM42688_ACCEL_WOM_Z_THR,&womValue,1);
  } else if(axis == ALL){
    s->writeReg(ICM42688_ACCEL_WOM_X_THR,&womValue,1);
    s->writeReg(ICM42688_ACCEL_WOM_Y_THR,&womValue,1);
    s->writeReg(ICM42688_ACCEL_WOM_Z_THR,&womValue,1);
  }
  k_msleep(1);
  bank = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
}


void icm42688_setWOMInterrupt(icm42688_struct *s,uint8_t axis)
{
  uint8_t bank = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  if(s->_INTPin == 1){
    s->writeReg(ICM42688_INT_SOURCE1,&axis,1);
  } else {
    s->writeReg(ICM42688_INT_SOURCE4,&axis,1);
  }
  k_msleep(50);
  s->SMDConfig.SMDMode = 1;
  s->SMDConfig.WOMMode = 1;
  s->SMDConfig.WOMIntMode = 0;
  s->writeReg(ICM42688_SMD_CONFIG,&s->SMDConfig,1);
}

void icm42688_enableSMDInterrupt(icm42688_struct *s,uint8_t mode)
{
  uint8_t bank = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  uint8_t INT = 1<<3 ;
  if(mode != 0){
    if(s->_INTPin == 1){
      s->writeReg(ICM42688_INT_SOURCE1,&INT,1);
    } else {
      s->writeReg(ICM42688_INT_SOURCE4,&INT,1);
    }
  }
  k_msleep(50);
  s->SMDConfig.SMDMode = mode;
  s->SMDConfig.WOMMode = 1;
  s->SMDConfig.WOMIntMode = 0;
  s->writeReg(ICM42688_SMD_CONFIG,&s->SMDConfig,1);
}

uint8_t icm42688_readInterruptStatus(icm42688_struct *s,uint8_t reg)
{
  uint8_t bank = 0;
  uint8_t status = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  s->readReg(s,reg,&status,1);
  return status;
}




bool icm42688_setODRAndFSR(icm42688_struct *s,uint8_t who,uint8_t ODR,uint8_t FSR)
{
  bool ret = true;
  uint8_t bank = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  if(who == GYRO){
    if(ODR > ODR_12_5KHZ || FSR > FSR_7){
      ret = false;
    }else{
      s->gyroConfig0.gyroODR = ODR;
      s->gyroConfig0.gyroFsSel = FSR;
      s->writeReg(ICM42688_GYRO_CONFIG0,&s->gyroConfig0,1);
      switch(FSR){
        case FSR_0:
          s->_gyroRange = 4000/65535.0;
          break;
        case FSR_1:
          s->_gyroRange = 2000/65535.0;
          break;
        case FSR_2:
          s->_gyroRange = 1000/65535.0;
          break;
        case FSR_3:
          s->_gyroRange = 500/65535.0;
          break;
        case FSR_4:
          s->_gyroRange = 250/65535.0;
          break;
        case FSR_5:
          s->_gyroRange = 125/65535.0;
          break;
        case FSR_6:
          s->_gyroRange = 62.5/65535.0;
          break;
        case FSR_7:
          s->_gyroRange = 31.25/65535.0;
          break;
      }
    }
  } else if(who == ACCEL){
    if(ODR > ODR_500HZ || FSR > FSR_3){
      ret = false;
    } else{
      s->accelConfig0.accelODR = ODR;
      s->accelConfig0.accelFsSel = FSR;
      s->writeReg(ICM42688_ACCEL_CONFIG0,&s->accelConfig0,1);
      switch(FSR){
        case FSR_0:
          s->_accelRange = 0.488f;
          break;
        case FSR_1:
          s->_accelRange = 0.244f;
          break;
        case FSR_2:
          s->_accelRange = 0.122f;
          break;
        case FSR_3:
          s->_accelRange = 0.061f;
          break;
      }
    }
  } 
  return ret;
}


void icm42688_setFIFODataMode(icm42688_struct *s)
{
  uint8_t bank = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  s->FIFOConfig1.FIFOHiresEn = 0;
  s->FIFOConfig1.FIFOAccelEn = 1;
  s->FIFOConfig1.FIFOGyroEn = 1;
  s->FIFOConfig1.FIFOTempEn = 1;
  s->FIFOConfig1.FIFOTmstFsyncEn = 0;
  s->writeReg(ICM42688_FIFO_CONFIG1,&s->FIFOConfig1,1);
}

void icm42688_startFIFOMode(icm42688_struct *s)
{
  uint8_t bank = 0;
  s->FIFOMode = true;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  s->setFIFODataMode(s);
  uint8_t start = 1<<6;
  s->writeReg(ICM42688_FIFO_CONFIG,&start,1);
  s->getFIFOData(s);
}


void icm42688_getFIFOData(icm42688_struct *s)
{
  uint8_t data[16];
  s->readReg(s,ICM42688_FIFO_DATA,data,16);
  s->_accelX = (uint16_t)data[1]<<8 | (uint16_t)data[2];
  //DBG("_accelX");DBG(_accelX);
  s->_accelY = (uint16_t)data[3]<<8 | (uint16_t)data[4];
  //DBG("_accelY");DBG(_accelY);
  s->_accelZ = (uint16_t)data[5]<<8 | (uint16_t)data[6];
  //DBG("_accelZ");DBG(_accelZ);
  s->_gyroX = (uint16_t)data[7]<<8 | (uint16_t)data[8];
  //DBG("_gyroX");DBG(_gyroX);
  s->_gyroY = (uint16_t)data[9]<<8 | (uint16_t)data[10];
  //DBG("_gyroY");DBG(_gyroY);
  s->_gyroZ = (uint16_t)data[11]<<8 | (uint16_t)data[12];
  //DBG("_gyroZ");DBG(_gyroZ);
  s->_temp = (uint8_t)data[13];
  //DBG("_temp");DBG(data[13]);
}


void icm42688_stopFIFOMode(icm42688_struct *s)
{
  uint8_t bank = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  uint8_t start = 1<<7;
  s->writeReg(ICM42688_FIFO_CONFIG,&start,1);
}

void icm42688_setINTMode(icm42688_struct *s,uint8_t INTPin,uint8_t INTmode,uint8_t INTPolarity,uint8_t INTDriveCircuit)
{
  uint8_t bank = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  if(s->_INTPin == 1){
    s->_INTPin = 1;
    s->INTConfig.INT1Mode = INTmode;
    s->INTConfig.INT1DriveCirCuit = INTDriveCircuit;
    s->INTConfig.INT1Polarity = INTPolarity;
  } else if(INTPin == 2){
    s->_INTPin = 2;
    s->INTConfig.INT2Mode = INTmode;
    s->INTConfig.INT2DriveCirCuit = INTDriveCircuit;
    s->INTConfig.INT2Polarity = INTPolarity;
  }
  s->writeReg(ICM42688_INT_CONFIG,&s->INTConfig,1);
}

void icm42688_startTempMeasure(icm42688_struct *s)
{
  s->PWRMgmt0.tempDis = 0;
  uint8_t bank = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  s->writeReg(ICM42688_PWR_MGMT0,&s->PWRMgmt0,1);
  k_msleep(1);
}
void icm42688_startGyroMeasure(icm42688_struct *s,uint8_t mode)
{
  s->PWRMgmt0.gyroMode = mode;
  uint8_t bank = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  s->writeReg(ICM42688_PWR_MGMT0,&s->PWRMgmt0,1);
  k_msleep(1);
}

void icm42688_startAccelMeasure(icm42688_struct *s,uint8_t mode)
{
  s->PWRMgmt0.accelMode = mode;
  uint8_t bank = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  s->writeReg(ICM42688_PWR_MGMT0,&s->PWRMgmt0,1);
  k_msleep(10);
}

void icm42688_setGyroNotchFilterFHz(icm42688_struct *s,double freq,uint8_t axis)
{
  uint8_t bank = 1;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  double fdesired = freq * 1000;
  double coswz = cos(2*3.14*fdesired/32);
  int16_t nfCoswz;
  uint8_t nfCoswzSel;
  if(abs(coswz)<=0.875){
    nfCoswz = round(coswz*256);
    nfCoswzSel = 0;
  } else {
    nfCoswzSel = 1;
    if(coswz> 0.875){
      nfCoswz = round(8*(1-coswz)*256);
    } else if(coswz < -0.875){
      nfCoswz = round(-8*(1+coswz)*256);
    }
  }
  if(axis == X_AXIS){
    s->gyroConfigStatic9.gyroNFCoswzSelX = nfCoswzSel;
    s->gyroConfigStatic9.gyroNFCoswzX8 = nfCoswz>>8;
    s->writeReg(ICM42688_GYRO_CONFIG_STATIC6,&nfCoswz,1);
    s->writeReg(ICM42688_GYRO_CONFIG_STATIC9,&s->gyroConfigStatic9,1);
  } else if(axis == Y_AXIS){
    s->gyroConfigStatic9.gyroNFCoswzSelY = nfCoswzSel;
    s->gyroConfigStatic9.gyroNFCoswzY8 = nfCoswz>>8;
    s->writeReg(ICM42688_GYRO_CONFIG_STATIC7,&nfCoswz,1);
    s->writeReg(ICM42688_GYRO_CONFIG_STATIC9,&s->gyroConfigStatic9,1);
  } else if(axis == Z_AXIS){
    s->gyroConfigStatic9.gyroNFCoswzSelZ = nfCoswzSel;
    s->gyroConfigStatic9.gyroNFCoswzZ8 = nfCoswz>>8;
    s->writeReg(ICM42688_GYRO_CONFIG_STATIC8,&nfCoswz,1);
    s->writeReg(ICM42688_GYRO_CONFIG_STATIC9,&s->gyroConfigStatic9,1);
  } else if(axis == ALL)
  {
    s->gyroConfigStatic9.gyroNFCoswzSelX = nfCoswzSel;
    s->gyroConfigStatic9.gyroNFCoswzX8 = nfCoswz>>8;
    s->gyroConfigStatic9.gyroNFCoswzSelY = nfCoswzSel;
    s->gyroConfigStatic9.gyroNFCoswzY8 = nfCoswz>>8;
    s->gyroConfigStatic9.gyroNFCoswzSelZ = nfCoswzSel;
    s->gyroConfigStatic9.gyroNFCoswzZ8 = nfCoswz>>8;
    s->writeReg(ICM42688_GYRO_CONFIG_STATIC6,&nfCoswz,1);
    s->writeReg(ICM42688_GYRO_CONFIG_STATIC7,&nfCoswz,1);
    s->writeReg(ICM42688_GYRO_CONFIG_STATIC8,&nfCoswz,1);
    s->writeReg(ICM42688_GYRO_CONFIG_STATIC9,&s->gyroConfigStatic9,1);
  }
  bank = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
}

void icm42688_setGyroNFbandwidth(icm42688_struct *s,uint8_t bw)
{
  uint8_t bank = 1;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  uint8_t bandWidth = (bw<<4) | 0x01;
  s->writeReg(ICM42688_GYRO_CONFIG_STATIC10,&bandWidth,1);
  bank = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
}


void icm42688_setGyroNotchFilter(icm42688_struct *s,bool mode)
{
  if(mode){
    s->gyroConfigStatic2.gyroNFDis = 0;
  } else {
    s->gyroConfigStatic2.gyroNFDis = 1;
  }
  uint8_t bank = 1;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  s->writeReg(ICM42688_GYRO_CONFIG_STATIC2,&s->gyroConfigStatic2,1);
  bank = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
}


void icm42688_setAAFBandwidth(icm42688_struct *s,uint8_t who,uint8_t BWIndex)
{
  uint8_t bank = 0;
  uint16_t AAFDeltsqr = BWIndex*BWIndex;
  if(who == GYRO){
    bank = 1;
    s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
    s->writeReg(ICM42688_GYRO_CONFIG_STATIC3,&BWIndex,1);
    s->writeReg(ICM42688_GYRO_CONFIG_STATIC4,&AAFDeltsqr,1);
    s->gyroConfigStatic5.gyroAAFDeltsqr = AAFDeltsqr>>8;
    if(BWIndex == 1){
      s->gyroConfigStatic5.gyroAAFBitshift = 15;
    } else if(BWIndex == 2){
      s->gyroConfigStatic5.gyroAAFBitshift = 13;
    } else if(BWIndex == 3){
      s->gyroConfigStatic5.gyroAAFBitshift = 12;
    } else if(BWIndex == 4){
      s->gyroConfigStatic5.gyroAAFBitshift = 11;
    } else if(BWIndex == 5||BWIndex == 6){
      s->gyroConfigStatic5.gyroAAFBitshift = 10;
    } else if(BWIndex > 6 && BWIndex < 10){
      s->gyroConfigStatic5.gyroAAFBitshift = 9;
    } else if(BWIndex > 9 && BWIndex < 14){
      s->gyroConfigStatic5.gyroAAFBitshift = 8;
    } else if(BWIndex > 13 && BWIndex < 19){
      s->gyroConfigStatic5.gyroAAFBitshift = 7;
    } else if(BWIndex > 18 && BWIndex < 27){
      s->gyroConfigStatic5.gyroAAFBitshift = 6;
    } else if(BWIndex > 26 && BWIndex < 37){
      s->gyroConfigStatic5.gyroAAFBitshift = 5;
    } else if(BWIndex > 36 && BWIndex < 53){
      s->gyroConfigStatic5.gyroAAFBitshift = 4;
    } else if(BWIndex > 53 && BWIndex <= 63){
      s->gyroConfigStatic5.gyroAAFBitshift = 3;
    }
    s->writeReg(ICM42688_GYRO_CONFIG_STATIC5,&s->gyroConfigStatic5,1);
    bank = 0;
    s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  } else if(who == ACCEL){
    bank = 2;
    s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
    s->accelConfigStatic2.accelAAFDelt = BWIndex;
    s->writeReg(ICM42688_ACCEL_CONFIG_STATIC2,&s->accelConfigStatic2,1);
    s->writeReg(ICM42688_ACCEL_CONFIG_STATIC3,&AAFDeltsqr,1);
    s->accelConfigStatic4.accelAAFDeltsqr = AAFDeltsqr>>8;
    if(BWIndex == 1){
      s->accelConfigStatic4.accelAAFBitshift = 15;
    } else if(BWIndex == 2){
      s->accelConfigStatic4.accelAAFBitshift = 13;
    } else if(BWIndex == 3){
      s->accelConfigStatic4.accelAAFBitshift = 12;
    } else if(BWIndex == 4){
      s->accelConfigStatic4.accelAAFBitshift = 11;
    } else if(BWIndex == 5||BWIndex == 6){
      s->accelConfigStatic4.accelAAFBitshift = 10;
    } else if(BWIndex > 6 && BWIndex < 10){
      s->accelConfigStatic4.accelAAFBitshift = 9;
    } else if(BWIndex > 9 && BWIndex < 14){
      s->accelConfigStatic4.accelAAFBitshift = 8;
    } else if(BWIndex > 13 && BWIndex < 19){
      s->accelConfigStatic4.accelAAFBitshift = 7;
    } else if(BWIndex > 18 && BWIndex < 27){
      s->accelConfigStatic4.accelAAFBitshift = 6;
    } else if(BWIndex > 26 && BWIndex < 37){
      s->accelConfigStatic4.accelAAFBitshift = 5;
    } else if(BWIndex > 36 && BWIndex < 53){
      s->accelConfigStatic4.accelAAFBitshift = 4;
    } else if(BWIndex > 53 && BWIndex <= 63){
      s->accelConfigStatic4.accelAAFBitshift = 3;
    }
    s->writeReg(ICM42688_ACCEL_CONFIG_STATIC4,&s->accelConfigStatic4,1);

    bank = 0;
    s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  } else if(who == ALL){
    bank = 1;
    s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
    s->writeReg(ICM42688_GYRO_CONFIG_STATIC3,&BWIndex,1);
    s->writeReg(ICM42688_GYRO_CONFIG_STATIC4,&AAFDeltsqr,1);
    s->gyroConfigStatic5.gyroAAFDeltsqr = AAFDeltsqr>>8;
    if(BWIndex == 1){
      s->gyroConfigStatic5.gyroAAFBitshift = 15;
    } else if(BWIndex == 2){
      s->gyroConfigStatic5.gyroAAFBitshift = 13;
    } else if(BWIndex == 3){
      s->gyroConfigStatic5.gyroAAFBitshift = 12;
    } else if(BWIndex == 4){
      s->gyroConfigStatic5.gyroAAFBitshift = 11;
    } else if(BWIndex == 5||BWIndex == 6){
      s->gyroConfigStatic5.gyroAAFBitshift = 10;
    } else if(BWIndex > 6 && BWIndex < 10){
      s->gyroConfigStatic5.gyroAAFBitshift = 9;
    } else if(BWIndex > 9 && BWIndex < 14){
      s->gyroConfigStatic5.gyroAAFBitshift = 8;
    } else if(BWIndex > 13 && BWIndex < 19){
      s->gyroConfigStatic5.gyroAAFBitshift = 7;
    } else if(BWIndex > 18 && BWIndex < 27){
      s->gyroConfigStatic5.gyroAAFBitshift = 6;
    } else if(BWIndex > 26 && BWIndex < 37){
      s->gyroConfigStatic5.gyroAAFBitshift = 5;
    } else if(BWIndex > 36 && BWIndex < 53){
      s->gyroConfigStatic5.gyroAAFBitshift = 4;
    } else if(BWIndex > 53 && BWIndex <= 63){
      s->gyroConfigStatic5.gyroAAFBitshift = 3;
    }
    s->writeReg(ICM42688_GYRO_CONFIG_STATIC5,&s->gyroConfigStatic5,1);
    bank = 2;
    s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
    s->accelConfigStatic2.accelAAFDelt = BWIndex;
    s->writeReg(ICM42688_ACCEL_CONFIG_STATIC2,&s->accelConfigStatic2,1);
    s->writeReg(ICM42688_ACCEL_CONFIG_STATIC3,&AAFDeltsqr,1);
    s->accelConfigStatic4.accelAAFDeltsqr = AAFDeltsqr>>8;
    if(BWIndex == 1){
      s->accelConfigStatic4.accelAAFBitshift = 15;
    } else if(BWIndex == 2){
      s->accelConfigStatic4.accelAAFBitshift = 13;
    } else if(BWIndex == 3){
      s->accelConfigStatic4.accelAAFBitshift = 12;
    } else if(BWIndex == 4){
      s->accelConfigStatic4.accelAAFBitshift = 11;
    } else if(BWIndex == 5||BWIndex == 6){
      s->accelConfigStatic4.accelAAFBitshift = 10;
    } else if(BWIndex > 6 && BWIndex < 10){
      s->accelConfigStatic4.accelAAFBitshift = 9;
    } else if(BWIndex > 9 && BWIndex < 14){
      s->accelConfigStatic4.accelAAFBitshift = 8;
    } else if(BWIndex > 13 && BWIndex < 19){
      s->accelConfigStatic4.accelAAFBitshift = 7;
    } else if(BWIndex > 18 && BWIndex < 27){
      s->accelConfigStatic4.accelAAFBitshift = 6;
    } else if(BWIndex > 26 && BWIndex < 37){
      s->accelConfigStatic4.accelAAFBitshift = 5;
    } else if(BWIndex > 36 && BWIndex < 53){
      s->accelConfigStatic4.accelAAFBitshift = 4;
    } else if(BWIndex > 53 && BWIndex <= 63){
      s->accelConfigStatic4.accelAAFBitshift = 3;
    }
    s->writeReg(ICM42688_ACCEL_CONFIG_STATIC4,&s->accelConfigStatic4,1);
    bank = 0;
    s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  }
}


void icm42688_setAAF(icm42688_struct *s,uint8_t who,bool mode)
{
  uint8_t bank = 0;
  if(who == GYRO){
    if(mode){
      s->gyroConfigStatic2.gyroAAFDis = 0;
    } else {
      s->gyroConfigStatic2.gyroAAFDis = 1;
    }
    bank = 1;
    s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
    s->writeReg(ICM42688_GYRO_CONFIG_STATIC2,&s->gyroConfigStatic2,1);
  }else if(who == ACCEL){
    if(mode){
      s->accelConfigStatic2.accelAAFDis = 0;
    } else {
      s->accelConfigStatic2.accelAAFDis = 1;
    }
    bank = 2;
    s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
    s->writeReg(ICM42688_ACCEL_CONFIG_STATIC2,&s->accelConfigStatic2,1);
  } else if(who == ALL){
    if(mode){
      s->gyroConfigStatic2.gyroAAFDis = 0;
      s->accelConfigStatic2.accelAAFDis = 0;
    } else {
      s->gyroConfigStatic2.gyroAAFDis = 1;
      s->accelConfigStatic2.accelAAFDis = 1;
    }
    bank = 1;
    s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
    s->writeReg(ICM42688_GYRO_CONFIG_STATIC2,&s->gyroConfigStatic2,1);
    bank = 2;
    s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
    s->writeReg(ICM42688_ACCEL_CONFIG_STATIC2,&s->accelConfigStatic2,1);
  }
  bank = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
}


bool icm42688_setUIFilter(icm42688_struct *s,uint8_t who,uint8_t filterOrder ,uint8_t UIFilterIndex)
{
  bool ret = true;
  uint8_t bank = 0;
  s->writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  if(filterOrder > 3 || UIFilterIndex > 15){
    ret = false;
  } else{
    if(who == GYRO){
      s->gyroConfig1.gyroUIFiltODR = filterOrder;
      s->writeReg(ICM42688_GYRO_CONFIG1,&s->gyroConfig1,1);
      s->gyroAccelConfig0.gyroUIFiltBW = UIFilterIndex;
      s->writeReg(ICM42688_GYRO_ACCEL_CONFIG0,&s->gyroAccelConfig0,1);
    } else if(who == ACCEL){
      s->accelConfig1.accelUIFiltORD = filterOrder;
      s->writeReg(ICM42688_ACCEL_CONFIG1,&s->accelConfig1,1);
      s->gyroAccelConfig0.accelUIFiltBW = UIFilterIndex;
      s->writeReg(ICM42688_GYRO_ACCEL_CONFIG0,&s->gyroAccelConfig0,1);
    } else if(who == ALL){
      s->gyroConfig1.gyroUIFiltODR = filterOrder;
      s->writeReg(ICM42688_GYRO_CONFIG1,&s->gyroConfig1,1);
      s->accelConfig1.accelUIFiltORD = filterOrder;
      s->writeReg(ICM42688_ACCEL_CONFIG1,&s->accelConfig1,1);
      s->gyroAccelConfig0.gyroUIFiltBW = UIFilterIndex;
      s->gyroAccelConfig0.accelUIFiltBW = UIFilterIndex;
      s->writeReg(ICM42688_GYRO_ACCEL_CONFIG0,&s->gyroAccelConfig0,1);
    }
  } 
  return ret;
}


