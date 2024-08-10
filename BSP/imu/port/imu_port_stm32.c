
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "main.h"
#include "i2c.h"

#define TAG "imu"

#include "elog.h"

static int i2c_write_byte_to_mem(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
static int i2c_read_from_mem(uint8_t dev_addr, uint8_t reg_addr, uint32_t length, uint8_t *buf);
static int imu_test(void);

/**
 * @brief  imu task 入口函数
 *
 */

void imu_task_entry(void *argument) {
    elog_i(TAG, "imu task start");

    while (1) {
        imu_test();
        osDelay(1000);
    }
}

static int imu_test(void) {
    uint8_t data = 0;
    i2c_read_from_mem(0x68, 0x75, 1, &data);
    elog_i(TAG, "mpu6050 id: 0x%x", data);
    return 0;
}

static int i2c_write_byte_to_mem(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
	int value = 0;
	//进入中断级代码临界区
	uint32_t ret = taskENTER_CRITICAL_FROM_ISR(); 
	value = HAL_I2C_Mem_Write(&hi2c2, dev_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xFF);
	//退出中断级代码临界区
	taskEXIT_CRITICAL_FROM_ISR(ret);
    return value;
//	return HAL_I2C_Mem_Write(&hi2c2, self->dev_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xFF);
}

static int i2c_read_from_mem(uint8_t dev_addr, uint8_t reg_addr, uint32_t length, uint8_t *buf) {
// return HAL_I2C_Mem_Read(&hi2c2, self->dev_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, buf, length, 0xFF);
//    return HAL_I2C_Mem_Read_DMA(&hi2c2, self->dev_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, buf, length);
	int value = 0;
	//进入中断级代码临界区
	uint32_t ret = taskENTER_CRITICAL_FROM_ISR(); 
	value = HAL_I2C_Mem_Read(&hi2c2, dev_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, buf, length, 0xFF);
	//退出中断级代码临界区
	taskEXIT_CRITICAL_FROM_ISR(ret);
    return value;
}
