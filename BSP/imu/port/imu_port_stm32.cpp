
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "main.h"
#include "i2c.h"

#include "MPU6050.hpp"

#define LOG_LVL ELOG_LVL_DEBUG
#define LOG_TAG "imu"
#include "elog.h"
#define IMU_READY_EVENT 0x01U

static int i2c_write_byte_to_mem(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
static int i2c_read_from_mem(uint8_t dev_addr, uint8_t reg_addr, uint32_t length, uint8_t *buf);
static int imu_test(void);

extern osEventFlagsId_t imu_eventHandle;
MPU6050 mpu6050(&imu_hi2c);

/**
 * @brief  imu task 入口函数
 *
 */

extern "C" void imu_task_entry(void *argument) {
    log_i("imu task start");

    while (1) {
        imu_test();
        osDelay(1000);
    }
}

static int imu_test(void) {
    uint8_t data = 0;
    // i2c_read_from_mem(0x68, 0x75, 1, &data);
    // log_i("mpu6050 id: 0x%x", data);

	// ================================================================
	// ===                      INITIAL SETUP                       ===
	// ================================================================
    do
    {
        mpu6050.Init();
        osDelay(100);
    } while (!mpu6050.testConnection());
    mpu6050.InitFilter(200, 100, 50);
	// 开启中断
	mpu6050.setIntEnabled(0x00);
	mpu6050.setInterruptMode(0);
	mpu6050.setIntEnabled(0x01);

	for(;;) {
		// 等待事件标志触发，中断服务程序会设置该标志
		osEventFlagsWait(imu_eventHandle, IMU_READY_EVENT, osFlagsWaitAny, osWaitForever);

		mpu6050.Update(true);
		// mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		// mpu6050.getRotation(&roll,&yaw,&pitch);
		log_i("Accel: %f %f %f", mpu6050.data.ax, mpu6050.data.ay, mpu6050.data.az);
	}

    return 0;
}

// 中断处理函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == IMU_ITR_Pin) {
		osEventFlagsSet(imu_eventHandle, IMU_READY_EVENT);
    }
}

static int i2c_write_byte_to_mem(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
	int value = 0;
	//进入中断级代码临界区
	uint32_t ret = taskENTER_CRITICAL_FROM_ISR(); 
	value = HAL_I2C_Mem_Write(&hi2c2, dev_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xFF);
	//退出中断级代码临界区
	taskEXIT_CRITICAL_FROM_ISR(ret);
    return value;
}

static int i2c_read_from_mem(uint8_t dev_addr, uint8_t reg_addr, uint32_t length, uint8_t *buf) {
	int value = 0;
	//进入中断级代码临界区
	uint32_t ret = taskENTER_CRITICAL_FROM_ISR(); 
	value = HAL_I2C_Mem_Read(&hi2c2, dev_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, buf, length, 0xFF);
	//退出中断级代码临界区
	taskEXIT_CRITICAL_FROM_ISR(ret);
    return value;
}
