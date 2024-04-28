#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "../inc/icm42688.h"

icm42688_struct icm_s;
icm42688_struct *icm = &icm_s;

#define MY_GPIO0 DT_NODELABEL(gpio0)
#define LED_PIN 6

const struct device *gpio0_dev = DEVICE_DT_GET(MY_GPIO0);

int main(void)
{

  icm->begin = &icm42688_begin;
  icm->spi_init = &icm42688_spi_init;
  icm->spi_begin = &icm42688_spi_begin;
  icm->getTemperature = &icm42688_getTemperature;
  icm->getAccelDataX = &icm42688_getAccelDataX;

  icm->getAccelDataY = &icm42688_getAccelDataY;
  icm->getAccelDataZ = &icm42688_getAccelDataZ;
  icm->getGyroDataX = &icm42688_getGyroDataX;
  icm->getGyroDataY = &icm42688_getGyroDataY;
  icm->getGyroDataZ = &icm42688_getGyroDataZ;
  icm->tapDetectionInit = &icm42688_tapDetectionInit;
  icm->getTapInformation = &icm42688_getTapInformation;
  icm->numberOfTap = &icm42688_numberOfTap;
  icm->axisOfTap = &icm42688_axisOfTap;
  icm->wakeOnMotionInit = &icm42688_wakeOnMotionInit;
  icm->setWOMTh = &icm42688_setWOMTh;
  icm->setWOMInterrupt = &icm42688_setWOMInterrupt;
  icm->enableSMDInterrupt = &icm42688_enableSMDInterrupt;
  icm->readInterruptStatus = &icm42688_readInterruptStatus;
  icm->setODRAndFSR = &icm42688_setODRAndFSR;
  icm->startFIFOMode = &icm42688_startFIFOMode;
  icm->stopFIFOMode = &icm42688_stopFIFOMode;
  icm->getFIFOData = &icm42688_getFIFOData;
  icm->setINTMode = &icm42688_setINTMode;
  icm->startGyroMeasure = &icm42688_startGyroMeasure;
  icm->startAccelMeasure = &icm42688_startAccelMeasure;
  icm->startTempMeasure = &icm42688_startTempMeasure;

  icm->writeReg = &icm42688_writeReg;
  icm->readReg = &icm42688_readReg;
  icm->setFIFODataMode = &icm42688_setFIFODataMode;

  gpio_pin_configure(gpio0_dev, LED_PIN, GPIO_OUTPUT);

  icm->spi_init(icm); // Init CS PIN and SPI dev

  icm->begin(icm);

  uint8_t id = 0;
  printk("ICM42688 begin success!!!");
  float accelDataX, accelDataY, accelDataZ, gyroDataX, gyroDataY, gyroDataZ, tempData;

  while (1)
  {

/*     tempData = icm->getTemperature(icm);
    accelDataX = icm->getAccelDataX(icm);
    accelDataY = icm->getAccelDataY(icm);
    accelDataZ = icm->getAccelDataZ(icm);
    gyroDataX = icm->getGyroDataX(icm);
    gyroDataY = icm->getGyroDataY(icm);
    gyroDataZ = icm->getGyroDataZ(icm);

    printk("Temperature:  %d ", tempData);

    printk("Accel_X: %d \r\n ", accelDataX);
    printk("Accel_Y: %d \r\n ", accelDataY);
    printk("Accel_Z: %d \r\n ", accelDataZ);

    printk("Gyro_X:  %d \r\n", gyroDataX);
    printk("Gyro_Y:  %d \r\n", gyroDataY);
    printk("Gyro_Z:  %d \r\n", gyroDataZ); */

    icm->readReg(icm, ICM42688_WHO_AM_I, &id, 1);
    printk("ID = %d \r\n", id);

    gpio_pin_toggle(gpio0_dev, LED_PIN);
    k_msleep(1000);
  }
}
