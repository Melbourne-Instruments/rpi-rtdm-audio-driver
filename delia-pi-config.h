// SPDX-License-Identifier: GPL-2.0
/**
 *-----------------------------------------------------------------------------
 * @brief DELIA pi configuration settings.
 * @copyright 2023-2024 Melbourne Instruments, Australia
 *-----------------------------------------------------------------------------
 */
#ifndef DELIA_PI_CONFIG_H
#define DELIA_PI_CONFIG_H

// Number of FPGAs
#define MELBINST_NUM_FPGAS  1

/* Output channels */
#define DELIA_PI_NUM_HQ_AUDIO_OUTPUT_CHANNELS           2
#define DELIA_PI_NUM_VOICE_OUTPUT_CHANNELS_PER_FPGA     (6*3)
#define MELBINST_PI_NUM_OUTPUT_CHANNELS_PER_FPGA        (DELIA_PI_NUM_HQ_AUDIO_OUTPUT_CHANNELS + DELIA_PI_NUM_VOICE_OUTPUT_CHANNELS_PER_FPGA)
#define MELBINST_PI_NUM_OUTPUT_CHANNELS                 (DELIA_PI_NUM_HQ_AUDIO_OUTPUT_CHANNELS + (DELIA_PI_NUM_VOICE_OUTPUT_CHANNELS_PER_FPGA * MELBINST_NUM_FPGAS))

/* Input channels */
#define MELBINST_PI_NUM_AUDIO_INPUT_CHANNELS    8
#define MELBINST_PI_NUM_FPGA_STATUS_REGS        48
#define MELBINST_PI_NUM_INPUT_CHANNELS          9

/* Codec settings */
#define MELBINST_PI_NUM_CODEC_CHANNELS  MELBINST_PI_NUM_OUTPUT_CHANNELS
#define MELBINST_PI_SAMPLING_RATE	    96000
#define MELBINST_PI_BUFFER_SIZE         128

#endif  /* DELIA_PI_CONFIG_H */