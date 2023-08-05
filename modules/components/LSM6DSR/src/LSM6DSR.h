/**
 ******************************************************************************
 * @file    LSM6DSR.h
 * @author  SRA
 * @version V1.0.1
 * @date    December 2022
 * @brief   Abstract Class of an LSM6DSR Inertial Measurement Unit (IMU) 6 axes
 *          sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __LSM6DSR_H__
#define __LSM6DSR_H__

/* Includes ------------------------------------------------------------------*/

#include "I2Cdev.h"
#include "lsm6dsr_reg.h"

/* Defines -------------------------------------------------------------------*/

#define LSM6DSR_ACC_SENSITIVITY_FS_2G   0.061f
#define LSM6DSR_ACC_SENSITIVITY_FS_4G   0.122f
#define LSM6DSR_ACC_SENSITIVITY_FS_8G   0.244f
#define LSM6DSR_ACC_SENSITIVITY_FS_16G  0.488f

#define LSM6DSR_GYRO_SENSITIVITY_FS_125DPS    4.370f
#define LSM6DSR_GYRO_SENSITIVITY_FS_250DPS    8.750f
#define LSM6DSR_GYRO_SENSITIVITY_FS_500DPS   17.500f
#define LSM6DSR_GYRO_SENSITIVITY_FS_1000DPS  35.000f
#define LSM6DSR_GYRO_SENSITIVITY_FS_2000DPS  70.000f
#define LSM6DSR_GYRO_SENSITIVITY_FS_4000DPS 140.000f


/* Typedefs ------------------------------------------------------------------*/

typedef enum
{
  LSM6DSR_OK = 0,
  LSM6DSR_ERROR =-1
} LSM6DSRStatusTypeDef;

typedef enum
{
  LSM6DSR_ACC_HIGH_PERFORMANCE_MODE,
  LSM6DSR_ACC_LOW_POWER_NORMAL_MODE
} LSM6DSR_ACC_Operating_Mode_t;

typedef enum
{
  LSM6DSR_GYRO_HIGH_PERFORMANCE_MODE,
  LSM6DSR_GYRO_LOW_POWER_NORMAL_MODE
} LSM6DSR_GYRO_Operating_Mode_t;


/* Class Declaration ---------------------------------------------------------*/
   
/**
 * Abstract class of an LSM6DSR Inertial Measurement Unit (IMU) 3 axes
 * sensor.
 */
class LSM6DSR
{
  public:
    LSM6DSR(uint8_t slaveAddress);
    //LSM6DSR(TwoWire *i2c, uint8_t address=LSM6DSR_I2C_ADD_H);
    //LSM6DSR(SPIClass *spi, int cs_pin, uint32_t spi_speed=2000000);
    LSM6DSRStatusTypeDef begin();
    LSM6DSRStatusTypeDef end();
    LSM6DSRStatusTypeDef ReadID(uint8_t *Id);
    LSM6DSRStatusTypeDef Enable_X();
    LSM6DSRStatusTypeDef Disable_X();
    LSM6DSRStatusTypeDef Get_X_Sensitivity(float *Sensitivity);
    LSM6DSRStatusTypeDef Get_X_ODR(float *Odr);
    LSM6DSRStatusTypeDef Set_X_ODR(float Odr);
    LSM6DSRStatusTypeDef Set_X_ODR_With_Mode(float Odr, LSM6DSR_ACC_Operating_Mode_t Mode);
    LSM6DSRStatusTypeDef Get_X_FS(int32_t *FullScale);
    LSM6DSRStatusTypeDef Set_X_FS(int32_t FullScale);
    LSM6DSRStatusTypeDef Get_X_AxesRaw(int16_t *Value);
    LSM6DSRStatusTypeDef Get_X_Axes(int32_t *Acceleration);
    LSM6DSRStatusTypeDef Get_X_DRDY_Status(uint8_t *Status);
    
    LSM6DSRStatusTypeDef Enable_G();
    LSM6DSRStatusTypeDef Disable_G();
    LSM6DSRStatusTypeDef Get_G_Sensitivity(float *Sensitivity);
    LSM6DSRStatusTypeDef Get_G_ODR(float *Odr);
    LSM6DSRStatusTypeDef Set_G_ODR(float Odr);
    LSM6DSRStatusTypeDef Set_G_ODR_With_Mode(float Odr, LSM6DSR_GYRO_Operating_Mode_t Mode);
    LSM6DSRStatusTypeDef Get_G_FS(int32_t *FullScale);
    LSM6DSRStatusTypeDef Set_G_FS(int32_t FullScale);
    LSM6DSRStatusTypeDef Get_G_AxesRaw(int16_t *Value);
    LSM6DSRStatusTypeDef Get_G_Axes(int32_t *AngularRate);
    LSM6DSRStatusTypeDef Get_G_DRDY_Status(uint8_t *Status);
    
    LSM6DSRStatusTypeDef Read_Reg(uint8_t reg, uint8_t *Data);
    LSM6DSRStatusTypeDef Write_Reg(uint8_t reg, uint8_t Data);
    
    /**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t addr, uint8_t* data, size_t length)
    {        
      I2Cdev::readBytes(devAddr, addr, length, data);
      return 1;
    }
    
    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t addr, uint8_t value)
    {  
      I2Cdev::writeByte(devAddr, addr, value);
    return 1;
    }

  private:
    uint8_t devAddr;

    LSM6DSRStatusTypeDef Set_X_ODR_When_Enabled(float Odr);
    LSM6DSRStatusTypeDef Set_X_ODR_When_Disabled(float Odr);
    LSM6DSRStatusTypeDef Set_G_ODR_When_Enabled(float Odr);
    LSM6DSRStatusTypeDef Set_G_ODR_When_Disabled(float Odr);

    /* Helper classes. */
    /*TwoWire *dev_i2c;
    SPIClass *dev_spi;*/
    
    /* Configuration */
    uint8_t address;
    int cs_pin;
    uint32_t spi_speed;
    
    lsm6dsr_odr_xl_t acc_odr;
    lsm6dsr_odr_g_t gyro_odr;
    
    uint8_t acc_is_enabled;
    uint8_t gyro_is_enabled;
       
    lsm6dsr_ctx_t reg_ctx;  
};

#ifdef __cplusplus
 extern "C" {
#endif
int32_t LSM6DSR_io_write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
int32_t LSM6DSR_io_read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
#ifdef __cplusplus
  }
#endif

#endif
