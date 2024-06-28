// SPDX-License-Identifier: GPL-2.0
/**
 *-----------------------------------------------------------------------------
 * @brief Based on the Audio RTDM driver by ELK, updated to use the SPI.
 * @copyright 2020-2022 Melbourne Instruments, Australia
 *-----------------------------------------------------------------------------
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/of_gpio.h>

/* RTDM headers */
#include <rtdm/driver.h>
#include <rtdm/rtdm.h>
#include "audio-rtdm.h"
#include "bcm2835-spi-nina.h"
#include "nina-pi-config.h"

MODULE_AUTHOR("Melbourne Instruments");
MODULE_DESCRIPTION("RTDM audio driver for NINA RPi");
MODULE_LICENSE("GPL");

#define DEFAULT_AUDIO_SAMPLING_RATE			48000
#define DEFAULT_AUDIO_NUM_INPUT_CHANNELS		8
#define DEFAULT_AUDIO_NUM_OUTPUT_CHANNELS		8
#define DEFAULT_AUDIO_NUM_CODEC_CHANNELS		8
#define DEFAULT_AUDIO_N_FRAMES_PER_BUFFER		128
#define DEFAULT_AUDIO_CODEC_FORMAT			INT24_LJ
#define NUM_INITIAL_BUFFER_SKIPS			1515	// ~1s @ 660us per buffer

static uint audio_ver_maj = AUDIO_RTDM_VERSION_MAJ;
module_param(audio_ver_maj, uint, 0644);

static uint audio_ver_min = AUDIO_RTDM_VERSION_MIN;
module_param(audio_ver_min, uint, 0644);

static uint audio_ver_rev = AUDIO_RTDM_VERSION_VER;
module_param(audio_ver_rev, uint, 0644);

static uint audio_buffer_size = DEFAULT_AUDIO_N_FRAMES_PER_BUFFER;
module_param(audio_buffer_size, uint, 0644);

static uint audio_input_channels = DEFAULT_AUDIO_NUM_INPUT_CHANNELS;
module_param(audio_input_channels, uint, 0444);

static uint audio_output_channels = DEFAULT_AUDIO_NUM_OUTPUT_CHANNELS;
module_param(audio_output_channels, uint, 0444);

static uint audio_sampling_rate = DEFAULT_AUDIO_SAMPLING_RATE;
module_param(audio_sampling_rate, uint, 0444);

static uint audio_format = DEFAULT_AUDIO_CODEC_FORMAT;
module_param(audio_format, uint, 0444);

static char *audio_hat = "nina-pi";
module_param(audio_hat, charp, 0660);

struct audio_dev_context {
	struct audio_rtdm_dev *rtdm_dev;
	uint64_t user_proc_calls;
};

static int initial_buffer_skips = 0;

static int audio_driver_open(struct rtdm_fd *fd, int oflags)
{
	struct audio_dev_context *dev_context;

	rtdm_printk(KERN_INFO "audio_rtdm: audio_open.\n");
    initial_buffer_skips = 0;
	dev_context = (struct audio_dev_context *)rtdm_fd_to_private(fd);
	dev_context->rtdm_dev = bcm2835_spi_open();
	dev_context->rtdm_dev->wait_flag = 0;
	dev_context->user_proc_calls = 0;
	rtdm_event_init(&dev_context->rtdm_dev->irq_event, 0);
    return 0;
}

static void audio_driver_close(struct rtdm_fd *fd)
{
	struct audio_dev_context *dev_context = (struct audio_dev_context *)
							rtdm_fd_to_private(fd);

	rtdm_printk(KERN_INFO "audio_rtdm: audio_close.\n");
	dev_context->rtdm_dev->wait_flag = 0;
	rtdm_event_destroy(&dev_context->rtdm_dev->irq_event);
	bcm2835_spi_close();

	// TODO - the code below has been commented out, do we need it?
    //rtdm_event_destroy(&dev_context->rtdm_dev->irq_event2);
	//if (dev_context->rtdm_dev->wait_flag) {
	//	for (i = 0; i < i2s_buffer->buffer_len/4; i++) {
	//		tx[i] = 0;
	////	}
	//	dev_context->rtdm_dev->wait_flag = 0;
	//}
}

static int audio_driver_mmap_nrt(struct rtdm_fd *fd, struct vm_area_struct *vma)
{
	struct audio_dev_context *dev_context = (struct audio_dev_context *)
							rtdm_fd_to_private(fd);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	return dma_mmap_coherent(dev_context->rtdm_dev->spi0_dev.dev,
		vma,
		dev_context->rtdm_dev->spi_buf, dev_context->rtdm_dev->spi_dma_addr,
		16 * PAGE_SIZE);
}

static int audio_driver_ioctl_rt(struct rtdm_fd *fd, unsigned int request,
								void __user *arg)
{
	int result;
	struct audio_dev_context *dev_context = (struct audio_dev_context *)
					rtdm_fd_to_private(fd);
	struct audio_rtdm_dev *dev = dev_context->rtdm_dev;	

	switch (request) {

	case AUDIO_IRQ_WAIT:
	{
		// Check if the event has already been signalled
		// This can happen if buffer processing is longer than the sample IRQ
		// time (normally due to cache hit issues)
		// If it has already been signalled then do not wait and return
		// immediately to process the next buffer
		//if (dev->rtdm_event_signalled == 0) {
			result = rtdm_event_wait(&dev->irq_event);
			if (result < 0) {
				// If the event wait has been interrupted (probably because
				// it has been destroyed on close), and the wait flag is 0, assume
				// this is the correct behaviour
				if ((result == -EINTR) && (dev->wait_flag == 0)) {
					rtdm_printk(KERN_INFO "rtdm_event_wait destroyed");
				}
				else {
					rtdm_printk(KERN_ERR "rtdm_event_wait failed: %d\n", result);
				}
				return result;
			}
		//}
		//dev->rtdm_event_signalled = 0;

        // Don't process the first n buffers while Sushi and the VST
        // are initialising and starting up
        if (initial_buffer_skips >= NUM_INITIAL_BUFFER_SKIPS) {
            if (bcm2835_spi_transfer()) {
                dev->buffer_idx = ~(dev->buffer_idx) & 0x1;
            }
        }
        else {
            initial_buffer_skips++;
			return -1;
        }
		dev_context->user_proc_calls = dev->kinterrupts;
		result = dev->buffer_idx;
		return !result;
	} 

	case AUDIO_PROC_START:
	{
		dev->wait_flag = 1;
		return 0;
	}

	case AUDIO_PROC_STOP:
	{
		dev->wait_flag = 0;
		return 0;
	}

	case AUDIO_USERPROC_FINISHED:
	{
		result = (dev->kinterrupts -
				dev_context->user_proc_calls);
		return result;
	}

	default:
		rtdm_printk(KERN_WARNING "audio_rtdm : audio_ioctl_rt: invalid value"
							" %d\n", request);
		return -EINVAL;
	}
}

static struct rtdm_driver audio_driver = {
	.profile_info		= RTDM_PROFILE_INFO(gpio,
						  RTDM_CLASS_EXPERIMENTAL,
						  RTDM_SUBCLASS_GENERIC,
						  RTAUDIO_PROFILE_VER),
	.device_flags		= RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.device_count		= 1,
	.context_size		= sizeof(struct audio_dev_context),
	.ops = {
		.open		= audio_driver_open,
		.close		= audio_driver_close,
		.mmap		= audio_driver_mmap_nrt,
		.ioctl_rt	= audio_driver_ioctl_rt,
	},
};

static struct rtdm_device rtdm_audio_device = {
	.driver = &audio_driver,
	.label = DEVICE_NAME,
};

int audio_rtdm_init(void)
{
	int ret, num_codec_channels = DEFAULT_AUDIO_NUM_CODEC_CHANNELS;

	if (!realtime_core_enabled()) {
		rtdm_printk(KERN_ERR "audio_rtdm: rt core not enabled\n");
		return -ENODEV;
	}
	msleep(100);
	rtdm_printk(KERN_INFO "audio_rtdm: SPI audio interface\n");
	if (!strcmp(audio_hat, "nina-pi")) {
		rtdm_printk(KERN_INFO "audio_rtdm: nina-pi hat\n");
		audio_input_channels = NINA_PI_NUM_INPUT_CHANNELS;
		audio_output_channels = NINA_PI_NUM_OUTPUT_CHANNELS;
		num_codec_channels = NINA_PI_NUM_CODEC_CHANNELS;
		audio_sampling_rate = NINA_PI_SAMPLING_RATE;
        audio_buffer_size = NINA_PI_BUFFER_SIZE;
		msleep(100);
		if (bcm2835_spi_init(audio_buffer_size, num_codec_channels, audio_hat)) {
			rtdm_printk(KERN_ERR "audio_rtdm: SPI init failed\n");
			return -1;
		}
	}
	else {
		rtdm_printk(KERN_ERR "audio_rtdm: Unsupported hat\n");
	}
	ret = rtdm_dev_register(&rtdm_audio_device);
	if (ret) {
		rtdm_dev_unregister(&rtdm_audio_device);
		rtdm_printk(KERN_ERR "audio_rtdm:driver init failed\n");
		return ret;
	}

	rtdm_printk(KERN_INFO "audio_rtdm: driver initialized\n");
	return 0;
}

void audio_rtdm_exit(void)
{
	rtdm_printk(KERN_INFO "audio_rtdm: driver exiting...\n");
	bcm2835_spi_exit();
	rtdm_dev_unregister(&rtdm_audio_device);
}

module_init(audio_rtdm_init)
module_exit(audio_rtdm_exit)
