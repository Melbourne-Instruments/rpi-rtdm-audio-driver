// SPDX-License-Identifier: GPL-2.0
/**
 *-----------------------------------------------------------------------------
 * @brief Based on the Audio RTDM driver by ELK, updated to use the SPI.
 * @copyright 2020-2022 Melbourne Instruments, Australia
 *-----------------------------------------------------------------------------
 */
#ifndef AUDIO_RTDM_H
#define AUDIO_RTDM_H

#include <linux/io.h>
#include <linux/ioctl.h>

#define RTDM_SUBCLASS_GPIO      0
#define DEVICE_NAME             "audio_rtdm"
#define RTAUDIO_PROFILE_VER     1
#define AUDIO_RTDM_VERSION_MAJ  0
#define AUDIO_RTDM_VERSION_MIN  2
#define AUDIO_RTDM_VERSION_VER  0
#define AUDIO_IOC_MAGIC         'r'

/* ioctl request to wait on dma callback */
#define AUDIO_IRQ_WAIT          _IO(AUDIO_IOC_MAGIC, 1)
/* This ioctl not used anymore but kept for backwards compatibility */
#define AUDIO_IMMEDIATE_SEND    _IOW(AUDIO_IOC_MAGIC, 2, int)
/* ioctl request to start receiving audio callbacks */
#define AUDIO_PROC_START        _IO(AUDIO_IOC_MAGIC, 3)
/* ioctl to inform the driver the user space process has completed */
#define AUDIO_USERPROC_FINISHED _IOW(AUDIO_IOC_MAGIC, 4, int)
/* ioctl to stop receiving audio callbacks */
#define AUDIO_PROC_STOP         _IO(AUDIO_IOC_MAGIC, 5)


enum codec_sample_format {
    INT24_LJ = 1,
    INT24_I2S,
    INT24_RJ,
    INT32_RJ
};

enum spi_id {
    SPI0,
    SPI4
};

struct audio_rtdm_spi_dev {
    enum spi_id      id;
    struct device    *dev;
    void __iomem     *base_addr;
    dma_addr_t       dma_reg_base;
    struct clk       *clk;
    unsigned long    clk_hz;
    bool             dma_pending;
    int              spi_submit_count;
    int              spi_transfer_count;
    struct dma_chan  *dma_tx;
    struct dma_chan  *dma_rx;
    struct dma_async_tx_descriptor *tx0_desc;
    struct dma_async_tx_descriptor *rx0_desc;
    struct dma_async_tx_descriptor *tx1_desc;
    struct dma_async_tx_descriptor *rx1_desc;
   
};

/* General audio rtdm device struct */
struct audio_rtdm_dev {
    struct audio_rtdm_spi_dev spi0_dev;
    struct audio_rtdm_spi_dev spi4_dev;
    rtdm_event_t              irq_event;
    unsigned                  wait_flag;
    unsigned                  buffer_idx;
    uint64_t                  kinterrupts;
    char                      *audio_hat;
    void                      *spi_buf;
    dma_addr_t                spi_dma_addr;
    unsigned int              rtdm_event_signalled;
};

#endif
