/*
 * icm20602.h
 *
 *  Created on: Jun 22, 2025
 *      Author: coder0908
 */

#pragma once

#include <stdbool.h>




bool icm20602_init();
void icm20602_loop();
void icm20602_irq_handler();
