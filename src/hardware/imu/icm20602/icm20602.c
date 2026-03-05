/*
 * drv_icm20602.c
 *
 *  Created on: Jun 22, 2025
 *      Author: coder0908
 */

#include <stdio.h>
#include "platform/platform.h"
#include "platform/spi.h"
#include "drivers/icm20602_driver/icm20602_driver.h"
#include "lib/crsf/crsf.h"
#include "middleware/morb/morb.h"
#include "icm20602.h"
#include "lib/math.h"


static struct drv_icm20602 s_drv_icm20602;



bool icm20602_init()
{
	if (!drv_icm20602_init(&s_drv_icm20602, &hspi2, ICM20602_CS_GPIO_Port, ICM20602_CS_Pin, ICM20602_INT_GPIO_Port, ICM20602_INT_Pin, 4000.0/65536.0, 32.0/65536.0, 0.996)) {
		return false;
	}
	HAL_Delay(50);

	// PWR_MGMT_2 0x6C
	//	ret = drv_icm20602_write_spi_byte(&s_drv_icm20602, ICM20602_REG_PWR_MGMT_2, 0x38); // Disable Acc(bit5:3-111), Enable Gyro(bit2:0-000)
	if (!drv_icm20602_write_spi_byte(&s_drv_icm20602, ICM20602_REG_PWR_MGMT_2, 0x00)) {
		return false;
	}

	// set sample rate to 1000Hz and apply a software filter
	if (!drv_icm20602_write_spi_byte(&s_drv_icm20602, ICM20602_REG_SMPLRT_DIV, 0x00)) {
		return false;
	}
	HAL_Delay(50);

	// Gyro DLPF Config
	//drv_icm20602_write_spi_byte(CONFIG, 0x00); // Gyro LPF fc 250Hz(bit2:0-000)
	// Gyro LPF fc 20Hz(bit2:0-100) at 1kHz sample rate
	if (!drv_icm20602_write_spi_byte(&s_drv_icm20602, ICM20602_REG_CONFIG, 0x05)) {
		return false;
	}
	HAL_Delay(50);

	// GYRO_CONFIG 0x1B
	// +- 2000dps
	if (!drv_icm20602_write_spi_byte(&s_drv_icm20602, ICM20602_REG_GYRO_CONFIG, 0x18)) {
		return false;
	}
	HAL_Delay(50);

	// ACCEL_CONFIG 0x1C
	// Acc sensitivity +-16g
	if (!drv_icm20602_write_spi_byte(&s_drv_icm20602, ICM20602_REG_ACCEL_CONFIG, 0x18)) {
		return false;
	}
	HAL_Delay(50);

	// ACCEL_CONFIG2 0x1D
	// Acc FCHOICE 1kHz(bit3-0), DLPF fc 44.8Hz(bit2:0-011)
	if (!drv_icm20602_write_spi_byte(&s_drv_icm20602, ICM20602_REG_ACCEL_CONFIG2, 0x03)) {
		return false;
	}
	HAL_Delay(50);

	if (!drv_icm20602_calibrate_gyro(&s_drv_icm20602)) {
		return false;
	}
	HAL_Delay(50);


	return true;
}

static int16_t s_x_accel_lsb, s_y_accel_lsb, s_z_accel_lsb;
static int16_t s_x_gyro_lsb, s_y_gyro_lsb, s_z_gyro_lsb;

static float s_x_accel_g, s_y_accel_g, s_z_accel_g;
static float s_x_gyro_dps, s_y_gyro_dps, s_z_gyro_dps;

void icm20602_loop()
{
	if (drv_icm20602_is_data_ready(&s_drv_icm20602)) {

		drv_icm20602_get_accel_gyro_lsb(&s_drv_icm20602, &s_x_accel_lsb, &s_y_accel_lsb, &s_z_accel_lsb, &s_x_gyro_lsb, &s_y_gyro_lsb, &s_z_gyro_lsb);
		drv_icm20602_parse_accel(&s_drv_icm20602, s_x_accel_lsb, s_y_accel_lsb, s_z_accel_lsb, &s_x_accel_g, &s_y_accel_g, &s_z_accel_g);
		drv_icm20602_parse_gyro(&s_drv_icm20602, s_x_gyro_lsb, s_y_gyro_lsb, s_z_gyro_lsb, &s_x_gyro_dps, &s_y_gyro_dps, &s_z_gyro_dps);



		struct topic_icm20602 topic_icm20602 = {0,};

//		printf("ax: %ld, ay: %ld, az: %ld,       gx: %ld, gy: %ld, gz: %ld\n\r", (long)s_x_accel_g, (long)s_y_accel_g, (long)s_z_accel_g, (long)s_x_gyro_dps, (long)s_y_gyro_dps, (long)s_z_gyro_dps);

		topic_icm20602.x_accel_g = -s_x_accel_g;
		topic_icm20602.y_accel_g = s_y_accel_g;
		topic_icm20602.z_accel_g = s_z_accel_g;

		topic_icm20602.x_gyro_dps = -s_x_gyro_dps;
		topic_icm20602.y_gyro_dps = s_y_gyro_dps;
		topic_icm20602.z_gyro_dps = s_z_gyro_dps;

		morb_publish(TOPIC_NUM_ICM20602, (void*)&topic_icm20602);
	}
}

void icm20602_irq_handler()
{
	drv_icm20602_irq_handler(&s_drv_icm20602);
}

