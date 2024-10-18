// SPDX-License-Identifier: GPL-2.0
/**
 *-----------------------------------------------------------------------------
 * @brief SPI module for the RTDM audio driver.
 * @copyright 2020-2023 Melbourne Instruments, Australia
 *-----------------------------------------------------------------------------
 */
#ifndef BCM2835_SPI_MELBINST_H
#define BCM2835_SPI_MELBINST_H

#include "audio-rtdm.h"

extern int bcm2835_spi_init(int audio_buffer_size, int audio_channels, char *audio_hat);
extern struct audio_rtdm_dev *bcm2835_spi_open(void);
extern bool bcm2835_spi_transfer(void); 
extern void bcm2835_spi_close(void);
extern int bcm2835_spi_exit(void);

#endif	// BCM2835_SPI_MELBINST_H
