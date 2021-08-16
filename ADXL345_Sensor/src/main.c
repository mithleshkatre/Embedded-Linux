/*============================================================================
 Name        :ADXL345.c
 Author      : mithlesh
 Version     :
 Copyright   : Your copyright notice
 Description : This Application prints Accelerometer and gyroscope raw and G values using ADXL345 sensor
 TODOs for the students :
 ============================================================================*/


/*================================================
BBB_expansion_header_P9_pins     ADXL345 pins
===================================================
P9_19                              SCL
P9_20                              SDA
P9_3                               VCC 3.3v
P9_1                               GND
==================================================== */

/*
 * Datasheet refs
 * 1. MPU-6000 and MPU-6050 Product Specification Revision 3.4
 * 2. MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2
 */


#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>



/*ADXL345 register addresses */

#define ADXL345_REG_POWER               0x2D  //power_cntl measure and wake up 8hz

/*These are the addresses of ADXL345 from which you will fetch accelerometer x,y,z high and low values */
#define ADXL345_REG_ACC_X_HIGH          0x32
#define ADXL345_REG_ACC_X_LOW           0x33
#define ADXL345_REG_ACC_Y_HIGH          0x34
#define ADXL345_REG_ACC_Y_LOW           0x35
#define ADXL345_REG_ACC_Z_HIGH          0x36
#define ADXL345_REG_ACC_Z_LOW           0x37

///*These are the addresses of ADXL345 from which you will fetch gyro x,y,z high and low values */
//#define ADXL345_REG_GYRO_X_HIGH          0x43
//#define ADXL345_REG_GYRO_X_LOW           0x44
//#define ADXL345_REG_GYRO_Y_HIGH          0x45
//#define ADXL345_REG_GYRO_Y_LOW           0x46
//#define ADXL345_REG_GYRO_Z_HIGH          0x47
//#define ADXL345_REG_GYRO_Z_LOW           0x48

/*
 * Different full scale ranges for acc and gyro
 * refer table 6.2 and 6.3 in the document MPU-6000 and MPU-6050 Product Specification Revision 3.4
 *
 */
#define ACC_FS_SENSITIVITY_0					382
#define ACC_FS_SENSITIVITY_1		            128
#define ACC_FS_SENSITIVITY_2		            71
#define ACC_FS_SENSITIVITY_3		            35

/* This is the I2C slave address of ADXL345 sensor */
#define ADXL345_SLAVE_ADDR 0x53

#define MAX_VALUE 50

/* This is the linux OS device file for hte I2C3 controller of the SOC */
#define I2C_DEVICE_FILE   "/dev/i2c-2"

int fd;

/*write a 8bit "data" to the sensor at the address indicated by "addr" */
int ADXL345_write(uint8_t addr, uint8_t data)
{
  int ret;
  char buf[2];
  buf[0]=addr;
  buf[1]=data;
  ret = write(fd,buf,2);
  if (ret <= 0)
  {
      perror("write failed\n");
      return -1;
  }
  return 0;
}

/*read "len" many bytes from "addr" of the sensor in to the adresss indicated by "pBuffer" */
int ADXL345_read(uint8_t base_addr, char *pBuffer,uint32_t len)
{
  int ret;
  char buf[1];
  buf[0]=base_addr;
  ret = write(fd,buf,1);
  if (ret <= 0)
  {
      perror("write address failed!!!!!!!!!!!!!!!!!!!!!!\n");
      return -1;
  }

  ret = read(fd,pBuffer,len);
  if(ret <= 0)
  {
      perror("read failed\n");
  }
  return 0;
}


/* by default ADXL345 will in sleep mode, so disable its sleep mode and also configure
 * the full scale ranges for gyro and acc
 */
void ADXL345_init()
{
    // 1. disable sleep mode
    ADXL345_write(ADXL345_REG_POWER, 0x00);
    usleep(500);
    ADXL345_write (ADXL345_REG_POWER, 0x08);  // power_cntl measure and wake up 8hz
    ADXL345_write (0x31, 0x01);
}

/* read accelerometer values of x,y,z in to the buffer "pBuffer" */
void ADXL345_read_acc(short int *pBuffer)
{
    //each axis value is of 2byte, so we need a buffer of 6bytes. 
    char acc_buffer[6];
    
    //start reading from the base address of accelerometer values i.e ADXL345_REG_ACC_X_HIGH
    ADXL345_read(ADXL345_REG_ACC_X_HIGH,acc_buffer,6);
    
    /* pBuffer[0]= acc x axis value , pBuffer[1]= acc y axis value , pBuffer[2]= acc z axis value  */
    pBuffer[0] = (int) ( (acc_buffer[0] << 8) |  acc_buffer[1] );
    pBuffer[1] = (int) ( (acc_buffer[2] << 8) |  acc_buffer[3] );
    pBuffer[2] = (int) ( (acc_buffer[4] << 8) |  acc_buffer[5] );

}


int main(void)
{

     short acc_value[3];
     double accx,accy,accz,gyrox,gyroy,gyroz;

     /*first lets open the I2C device file */
    if ((fd = open(I2C_DEVICE_FILE,O_RDWR)) < 0) {
        perror("Failed to open I2C device file.\n");
        return -1;
    }

    /*set the I2C slave address using ioctl I2C_SLAVE command */
    if (ioctl(fd,I2C_SLAVE,ADXL345_SLAVE_ADDR) < 0) {
            perror("Failed to set I2C slave address.\n");
            close(fd);
            return -1;
    }

    ADXL345_init();


    while(1)
    {
        ADXL345_read_acc(acc_value);

        /*Convert acc raw values in to 'g' values*/
        accx = (double) acc_value[0]/ACC_FS_SENSITIVITY_1;
        accy = (double) acc_value[1]/ACC_FS_SENSITIVITY_1;
        accz = (double) acc_value[2]/ACC_FS_SENSITIVITY_1;



#if 0
        /* print just the raw values read */
       printf("Acc(raw)=> X:%d Y:%d Z:%d \n", \
                   acc_value[0],acc_value[1],acc_value[2]);
       
       /* print the 'g' and '°/s' values */
       printf("Acc(g)=> X:%.2f Y:%.2f Z:%.2f\n", \
               accx,accy,accz);
#endif

#if 1
       printf("%0.2f	%0.2f	%0.2f\n",accx,accy,accz);
#endif
      
      /*wait for 250000 micro seconds, thats 250ms before going for another round */
       usleep(50 * 1000);
    }

}


