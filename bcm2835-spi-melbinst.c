// SPDX-License-Identifier: GPL-2.0
/**
 *-----------------------------------------------------------------------------
 * @brief SPI module for the RTDM audio driver.
 *        NINA: This uses two SPI devices (SPI0 and SPI4) to transmit and receive
 *        data to the NINA FPGAs.
 *        DELIA: This uses a single SPI device (SPI0) to transmit and receive
 *        data to the DELIA FPGA.
 *	      A lot of this code is based on the Xenomai SPI module by Philippe
 *        Gerum, and the standard linux Broadcom BCM2835 SPI module.
 * @copyright 2020-2024 Melbourne Instruments, Australia
 *-----------------------------------------------------------------------------
 */
#include <linux/module.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/dmaengine.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <rtdm/driver.h>
#include <rtdm/uapi/spi.h>
#include "bcm2835-spi-melbinst.h"
#if MELBINST_HAT == 0
#include "nina-pi-config.h"
#elif MELBINST_HAT == 1
#include "delia-pi-config.h"
#else
#error Unknown Melbourne Instruments RPi target device
#endif

/*-----------------------------------------------------------------------------
 * Constants
 *---------------------------------------------------------------------------*/

/* Define to simulate the Melbourne Instruments Hat sample rate interrupt */
#ifndef SIMULATE_MELBINST_HAT
#define SIMULATE_MELBINST_HAT       0
#endif

/* PACTL_CS register address */
#define BCM2711_PACTL_CS_ADDR       0xfe204e00

/* SPI register offsets */
#define BCM2835_SPI_CS	            0x00
#define BCM2835_SPI_FIFO            0x04
#define BCM2835_SPI_CLK             0x08
#define BCM2835_SPI_DLEN            0x0c
#define BCM2835_SPI_LTOH            0x10
#define BCM2835_SPI_DC              0x14

/* Bitfields in CS */
#define BCM2835_SPI_CS_LEN_LONG     0x02000000
#define BCM2835_SPI_CS_DMA_LEN      0x01000000
#define BCM2835_SPI_CS_CSPOL2       0x00800000
#define BCM2835_SPI_CS_CSPOL1       0x00400000
#define BCM2835_SPI_CS_CSPOL0       0x00200000
#define BCM2835_SPI_CS_RXF          0x00100000
#define BCM2835_SPI_CS_RXR          0x00080000
#define BCM2835_SPI_CS_TXD          0x00040000
#define BCM2835_SPI_CS_RXD          0x00020000
#define BCM2835_SPI_CS_DONE         0x00010000
#define BCM2835_SPI_CS_LEN          0x00002000
#define BCM2835_SPI_CS_REN          0x00001000
#define BCM2835_SPI_CS_ADCS         0x00000800
#define BCM2835_SPI_CS_INTR         0x00000400
#define BCM2835_SPI_CS_INTD         0x00000200
#define BCM2835_SPI_CS_DMAEN        0x00000100
#define BCM2835_SPI_CS_TA           0x00000080
#define BCM2835_SPI_CS_CSPOL        0x00000040
#define BCM2835_SPI_CS_CLEAR_RX     0x00000020
#define BCM2835_SPI_CS_CLEAR_TX     0x00000010
#define BCM2835_SPI_CS_CPOL         0x00000008
#define BCM2835_SPI_CS_CPHA         0x00000004
#define BCM2835_SPI_CS_CS_10        0x00000002
#define BCM2835_SPI_CS_CS_01        0x00000001

/* SPI mode bits */
#define BCM2835_SPI_MODE_BITS       (SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_NO_CS | SPI_3WIRE)

/* 
** Size of the SPI buffer to reserve in pages
** The SPI buffer consists of audio, and is double buffered
** Note: Page size is assumed to be 4096 bytes
*/
#define SPI_OUTPUT_BUFFER_AUDIO_SAMPLES_SIZE        (MELBINST_PI_NUM_OUTPUT_CHANNELS_PER_FPGA * MELBINST_PI_BUFFER_SIZE * 24 / 8)
#define SPI_INPUT_BUFFER_AUDIO_SAMPLES_SIZE         (MELBINST_PI_NUM_AUDIO_INPUT_CHANNELS * MELBINST_PI_BUFFER_SIZE) * 4
#define SPI_OUTPUT_BUFFER_TRANSFER_SIZE             (SPI_OUTPUT_BUFFER_AUDIO_SAMPLES_SIZE + (2*4))
#define SPI_INPUT_BUFFER_TRANSFER_SIZE              (SPI_INPUT_BUFFER_AUDIO_SAMPLES_SIZE + ((MELBINST_PI_NUM_FPGA_STATUS_REGS + 7) * 4))
#define SPI_BUFFER_TRANSFER_SIZE                    SPI_OUTPUT_BUFFER_TRANSFER_SIZE
#define SPI_BUFFER_TRANSFER_SIZE_IN_PAGES           2
#define RESERVED_SPI_BUFFER_SIZE_IN_PAGES           (SPI_BUFFER_TRANSFER_SIZE_IN_PAGES * 4 * MELBINST_NUM_FPGAS)

/* GPIO lines used for the FPGA interrupt/reset */
#define FPGA_INTERRUPT_GPIO             18
#define FPGA_RESET_GPIO                 24

/* GPO line used to simulate the FPGA interrupt */
#if SIMULATE_MELBINST_HAT == 1
#define SIMULATE_FPGA_INTERUPT_GPIO     17
#endif


/*-----------------------------------------------------------------------------
 * Function Prototypes
 *---------------------------------------------------------------------------*/

static inline u32 bcm2835_rd(void *base_addr, unsigned int reg);
static inline void bcm2835_wr(void *base_addr, unsigned int reg, u32 val);
static void bcm2835_spi_reset_hw(void *base_addr);
static void submit_next_spi_transfer(struct audio_rtdm_spi_dev *spi_dev);
static void bcm2835_spi_dma_init(struct audio_rtdm_spi_dev *spi_dev);
static void bcm2835_spi_dma_release(struct audio_rtdm_spi_dev *spi_dev);
static struct dma_async_tx_descriptor *bcm2835_spi_prep_slave(struct audio_rtdm_spi_dev *spi_dev,
                                  dma_addr_t buf, size_t len,
                                  bool is_tx);
static bool bcm2835_spi_dma_check_last_transfer(struct audio_rtdm_spi_dev *spi_dev);
static void bcm2835_spi_dma_transfer(struct audio_rtdm_spi_dev *spi_dev);                                  
static void bcm2835_spi_dma_done(void *data);
static int bcm2835_spi_probe(struct platform_device *pdev);
static int bcm2835_spi_remove(struct platform_device *pdev);
static int bcm2835_spi_configure(struct rtdm_spi_config *config, struct audio_rtdm_spi_dev *spi_dev);
static int bcm2835_gpio_init(struct device *pdev);
static int bcm2835_dma_init(void);
static void bcm2835_dma_release(void);                      
static int fpga_irq_handler(rtdm_irq_t *irq_handle);

#if SIMULATE_MELBINST_HAT == 1
static void simulate_fpga_sample_rate(rtdm_timer_t *timer);
#endif


/*-----------------------------------------------------------------------------
 * Module Variables
 *---------------------------------------------------------------------------*/

static struct audio_rtdm_dev *audio_static_dev;
static rtdm_irq_t rtdm_fpga_irq;
#if SIMULATE_MELBINST_HAT == 1
static rtdm_timer_t rtdm_fpga_timer;
#endif


static inline u32 bcm2835_rd(void *base_addr, unsigned int reg)
{
    /* Read the SPI register */
    return readl(base_addr + reg);
}

static inline void bcm2835_wr(void *base_addr, unsigned int reg, u32 val)
{
    /* Write the SPI register */
    writel(val, base_addr + reg);
}

static void bcm2835_spi_reset_hw(void *base_addr)
{
    u32 cs = bcm2835_rd(base_addr, BCM2835_SPI_CS);

    /* Reset the SPI block */
    cs &= ~(BCM2835_SPI_CS_INTR |
            BCM2835_SPI_CS_INTD |
            BCM2835_SPI_CS_DMAEN |
            BCM2835_SPI_CS_TA);
    cs |= BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX;
    bcm2835_wr(base_addr, BCM2835_SPI_CS, cs);
    bcm2835_wr(base_addr, BCM2835_SPI_DLEN, 0);
}

static void submit_next_spi_transfer(struct audio_rtdm_spi_dev *spi_dev)
{
    struct audio_rtdm_dev *audio_dev = audio_static_dev;

    /* Submit the next transfer depending on the buffer index */
    if (audio_dev->buffer_idx == 0) {
        /* Submit transfer 0 */
        dmaengine_submit(spi_dev->tx1_desc);
        dmaengine_submit(spi_dev->rx1_desc);
    }
    else {
        /* Submit transfer 1 */
        dmaengine_submit(spi_dev->tx0_desc);    
        dmaengine_submit(spi_dev->rx0_desc);             
    }
    spi_dev->spi_submit_count++;     
}

static void bcm2835_spi_dma_init(struct audio_rtdm_spi_dev *spi_dev)
{
    struct dma_slave_config slave_config;
    int ret;

    /* 
     ** Note: This dmaengine_resume is a way to enter the dma backend
     ** and get rtdm irqs. The initialized string in the private element is 
     ** used as an identifier to recognize which channels need to be 
     ** real-time safe 
     */

    /* Get the tx/rx dma */
    spi_dev->dma_tx = dma_request_slave_channel(spi_dev->dev, "tx");
    if (!spi_dev->dma_tx) {
        dev_err(spi_dev->dev, "No tx-dma configuration found - not using dma mode\n");
        goto err;
    }
    spi_dev->dma_rx = dma_request_slave_channel(spi_dev->dev, "rx");
    if (!spi_dev->dma_rx) {
        dev_err(spi_dev->dev, "No rx-dma configuration found - not using dma mode\n");
        goto err_release;
    }
    if (spi_dev->id == SPI0) {
	    spi_dev->dma_rx->private = "rtdm-rx1-irq";
    }
    else {
        spi_dev->dma_rx->private = "rtdm-rx2-irq";
    }
    dmaengine_resume(spi_dev->dma_rx);

    /* Configure the tx DMA */
    slave_config.direction = DMA_MEM_TO_DEV;
    slave_config.dst_addr = (u32)(spi_dev->dma_reg_base + BCM2835_SPI_FIFO);
    slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
    ret = dmaengine_slave_config(spi_dev->dma_tx, &slave_config);
    if (ret)
        goto err_config;

    /* Configure the rx DMA */
    slave_config.direction = DMA_DEV_TO_MEM;
    slave_config.src_addr = (u32)(spi_dev->dma_reg_base + BCM2835_SPI_FIFO);
    slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
    ret = dmaengine_slave_config(spi_dev->dma_rx, &slave_config);
    if (ret)
        goto err_config;
    return;

err_config:
    dev_err(spi_dev->dev, "Issue configuring dma: %d - not using DMA mode\n", ret);
err_release:
    bcm2835_spi_dma_release(spi_dev);
err:
    return;
}

static void bcm2835_spi_dma_release(struct audio_rtdm_spi_dev *spi_dev)
{
    rtdm_printk(KERN_INFO "bcm2835_dma_release");

    /* Terminate and release the tx DMA */
    if (spi_dev->dma_tx) {
        dmaengine_terminate_all(spi_dev->dma_tx);
        dma_release_channel(spi_dev->dma_tx);
        spi_dev->dma_tx = NULL;
    }

    /* Terminate and release the rx DMA */
    if (spi_dev->dma_rx) {
        dmaengine_terminate_all(spi_dev->dma_rx);
        dma_release_channel(spi_dev->dma_rx);
        spi_dev->dma_rx = NULL;
    }
}

static struct dma_async_tx_descriptor *bcm2835_spi_prep_slave(
                                struct audio_rtdm_spi_dev *spi_dev,
                                dma_addr_t buf, size_t len,
                                bool is_tx)
{
    struct dma_chan *chan;
    enum dma_transfer_direction dir;
    unsigned long flags;
    struct dma_async_tx_descriptor *desc;

    /* Preparing tx/rx? */
    if (is_tx) {
        /* Setup tx details */
        dir   = DMA_MEM_TO_DEV;
        chan  = spi_dev->dma_tx;
        flags = DMA_CTRL_REUSE; /* no  tx interrupt */
    } 
    else {
        /* Setup rx details */
        dir   = DMA_DEV_TO_MEM;
        chan  = spi_dev->dma_rx;
        flags = DMA_CTRL_REUSE | DMA_PREP_INTERRUPT;
    }

    /* Prepare the channel */
    desc = dmaengine_prep_slave_single(chan, buf, len, dir, flags);
    if (!desc)
        return NULL;

    /* Set callback for rx complete */
    if (!is_tx) {
        desc->callback = bcm2835_spi_dma_done;
        desc->callback_param = spi_dev;
    }
    return desc;
}

static bool bcm2835_spi_dma_check_last_transfer(struct audio_rtdm_spi_dev *spi_dev)
{
    // Are we trying to start a new transfer before the previous has
    // finished?
    if (spi_dev->spi_transfer_count != spi_dev->spi_submit_count) {
        // Log a kernel error and *do not* initiate the transfer
        rtdm_printk(KERN_ERR "SPI transfer count %d != SPI submit count %d\n", spi_dev->spi_transfer_count, spi_dev->spi_submit_count);
        return false;
    }
    return true;
}

static void bcm2835_spi_dma_transfer(struct audio_rtdm_spi_dev *spi_dev)
{
    u32 cs;

    /* Start the SPI DMA transfer */
    spi_dev->spi_transfer_count++;
    submit_next_spi_transfer(spi_dev);
    dma_async_issue_pending(spi_dev->dma_tx);

    /* Mark as dma pending */
    spi_dev->dma_pending = true;

    /* Set the DMA length */
    bcm2835_wr(spi_dev->base_addr, BCM2835_SPI_DLEN, SPI_BUFFER_TRANSFER_SIZE);

    /* Start the HW */
    cs = bcm2835_rd(spi_dev->base_addr, BCM2835_SPI_CS);
    cs |= BCM2835_SPI_CS_TA | BCM2835_SPI_CS_DMAEN;
    bcm2835_wr(spi_dev->base_addr, BCM2835_SPI_CS, cs);

    /* Start rx-DMA late */
    dma_async_issue_pending(spi_dev->dma_rx);
}

static void bcm2835_spi_dma_done(void *data)
{
    struct audio_rtdm_spi_dev *spi_dev = (struct audio_rtdm_spi_dev *)data;

    // Has this DMA done processing occurred late - that is,
    // the next transfer has been triggered before this DMA
    // done processing was received?
    if (spi_dev->spi_submit_count < spi_dev->spi_transfer_count) {
        // Log a kernel error
        rtdm_printk(KERN_ERR "SPI submit count < SPI transfer count\n");
    }

    bcm2835_spi_reset_hw(spi_dev->base_addr);

    /* Terminate tx-dma as we do not have an irq for it
     * because when the rx dma will terminate and this callback
     * is called the tx-dma must have finished - can't get to this
     * situation otherwise...
     */
    if (cmpxchg(&spi_dev->dma_pending, true, false)) {
        dmaengine_terminate_async(spi_dev->dma_tx);
    }
}

static int bcm2835_spi_probe(struct platform_device *pdev)
{
    struct resource *res;
    struct audio_rtdm_dev *audio_dev = audio_static_dev;
#if MELBINST_HAT == 0
    void __iomem *regs;
#endif
    const __be32 *addr;
    int len;
    int ret;

    rtdm_printk(KERN_INFO "bcm2835_spi_probe");

    /* Allocate the audio device buffer if not allocated */
    if (!audio_dev)
    {
        audio_dev = devm_kzalloc(&pdev->dev, sizeof(struct audio_rtdm_dev), GFP_KERNEL);
        if (!audio_dev)
            return -ENOMEM;
        audio_static_dev = audio_dev; 
    }

    /* Is this SPI0? */
    if (of_find_property(pdev->dev.of_node, "id-spi0", &len))
    {
        /* Get the SPI device base address */
        audio_dev->spi0_dev.id = SPI0;
        audio_dev->spi0_dev.dev = &pdev->dev;
        res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        audio_dev->spi0_dev.base_addr = devm_ioremap_resource(&pdev->dev, res);
        if (IS_ERR(audio_dev->spi0_dev.base_addr)) {
            dev_err(&pdev->dev, "%s: cannot map I/O memory\n", __func__);
            ret = PTR_ERR(audio_dev->spi0_dev.base_addr);
            return ret;
        }
        
        /* Get the SPI clock */
        audio_dev->spi0_dev.clk = devm_clk_get(&pdev->dev, NULL);
        if (IS_ERR(audio_dev->spi0_dev.clk)) {
            ret = PTR_ERR(audio_dev->spi0_dev.clk);
            return ret;
        }
        audio_dev->spi0_dev.clk_hz = clk_get_rate(audio_dev->spi0_dev.clk);
        clk_prepare_enable(audio_dev->spi0_dev.clk);

        /* Get the DMA register base address for this SPI */
        addr = of_get_address(pdev->dev.of_node, 0, NULL, NULL);
        if (!addr) {
            dev_err(&pdev->dev, "Could not get DMA-register address - not using dma mode\n");
            return -EINVAL;
        }
        audio_dev->spi0_dev.dma_reg_base = be32_to_cpup(addr);

        /* Initialise the hardware with the default polarities */
        bcm2835_wr(audio_dev->spi0_dev.base_addr, BCM2835_SPI_CS,
                   BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX);
    }
#if MELBINST_HAT == 0
    /* Is this SPI4? */
    else if (of_find_property(pdev->dev.of_node, "id-spi4", &len))
    {
        /* 
         ** RX DMA - assumes SPI4 is for the receive SPI
         ** Enable SPI4 DMA in PACTL_CS
         */
        regs = ioremap(BCM2711_PACTL_CS_ADDR, 4);
        writel(readl(regs) | 0x9000000, regs);
        iounmap(regs);

        /* Get the SPI device base address */
        audio_dev->spi4_dev.id = SPI4;
        audio_dev->spi4_dev.dev = &pdev->dev;
        res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        audio_dev->spi4_dev.base_addr = devm_ioremap_resource(&pdev->dev, res);
        if (IS_ERR(audio_dev->spi4_dev.base_addr)) {
            dev_err(&pdev->dev, "%s: cannot map I/O memory\n", __func__);
            ret = PTR_ERR(audio_dev->spi4_dev.base_addr);
            return ret;
        }

        /* Get the SPI clock */
        audio_dev->spi4_dev.clk = devm_clk_get(&pdev->dev, NULL);
        if (IS_ERR(audio_dev->spi4_dev.clk)) {
            ret = PTR_ERR(audio_dev->spi4_dev.clk);
            return ret;
        }
        audio_dev->spi4_dev.clk_hz = clk_get_rate(audio_dev->spi4_dev.clk);
        clk_prepare_enable(audio_dev->spi4_dev.clk);

        /* Get the DMA register base address for this SPI */
        addr = of_get_address(pdev->dev.of_node, 0, NULL, NULL);
        if (!addr) {
            dev_err(&pdev->dev, "Could not get DMA-register address - not using dma mode\n");
            return -EINVAL;
        }
        audio_dev->spi4_dev.dma_reg_base = be32_to_cpup(addr);

        /* Initialise the hardware with the default polarities */
        bcm2835_wr(audio_dev->spi4_dev.base_addr, BCM2835_SPI_CS,
                   BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX);     
    }
#endif
    else
    {
        /* Unknown SPI device */
        dev_err(&pdev->dev, "Unknown SPI mapped to RTDM\n");
        return -EINVAL;
    }

    /* SPI Device probe OK */
    dev_info(&pdev->dev, "bcm2835_spi_probe OK\n");
	return 0;
}

static int bcm2835_spi_remove(struct platform_device *pdev)
{
    struct audio_rtdm_dev *audio_dev = audio_static_dev;

    rtdm_printk(KERN_INFO "bcm2835_spi_remove");

    /* Clear FIFOs, and disable the HW block */
    bcm2835_wr(audio_dev->spi0_dev.base_addr, BCM2835_SPI_CS,
               BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX);
#if MELBINST_HAT == 0
    bcm2835_wr(audio_dev->spi4_dev.base_addr, BCM2835_SPI_CS,
               BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX);
#endif

    /* Free the RTDM interrupt and disable the clock */
    rtdm_irq_free(&rtdm_fpga_irq);
    clk_disable_unprepare(audio_dev->spi0_dev.clk);
#if MELBINST_HAT == 0    
    clk_disable_unprepare(audio_dev->spi4_dev.clk);
#endif
    return 0;
}

int bcm2835_spi_configure(struct rtdm_spi_config *config, struct audio_rtdm_spi_dev *spi_dev)
{
    unsigned long spi_hz, cdiv;
    u32 cs;

    /* Set clock polarity and phase */
    cs = bcm2835_rd(spi_dev->base_addr, BCM2835_SPI_CS);
    cs &= ~(BCM2835_SPI_CS_CPOL | BCM2835_SPI_CS_CPHA);
    if (config->mode & SPI_CPOL)
        cs |= BCM2835_SPI_CS_CPOL;
    if (config->mode & SPI_CPHA)
        cs |= BCM2835_SPI_CS_CPHA;
    bcm2835_wr(spi_dev->base_addr, BCM2835_SPI_CS, cs);
	
    /* Set the clock frequency */
    spi_hz = config->speed_hz;

    /*
     * Fastest clock rate is of the APB clock, which is close to
     * clk_hz / 2
     */
    if (spi_hz >= spi_dev->clk_hz / 2)
        cdiv = 2;
    else if (spi_hz) {
        cdiv = DIV_ROUND_UP(spi_dev->clk_hz, spi_hz); /* Multiple of 2 */
        cdiv += (cdiv % 2);
        if (cdiv >= 65536)
            cdiv = 0;
    } else
	    cdiv = 0;
    bcm2835_wr(spi_dev->base_addr, BCM2835_SPI_CLK, cdiv);
    return 0;
}

static int bcm2835_gpio_init(struct device *dev)
{
    int ret;
    int gpio_irq;

    /* Initialise the FPGA GPIO interrupt line */
    ret = gpio_request(FPGA_INTERRUPT_GPIO, "FPGA_IRQ");
    if (ret) {
        dev_err(dev, "gpio_request: FAILED\n");
        return ret;
    }
    ret = gpio_direction_input(FPGA_INTERRUPT_GPIO);
    if (ret) {
        dev_err(dev, "gpio_direction_input: FAILED\n");
        return ret;
    }
    gpio_irq = gpio_to_irq(FPGA_INTERRUPT_GPIO);
    if (gpio_irq < 0) {
        dev_err(dev, "gpio_to_irq: FAILED\n");
        return gpio_irq;
    }
    ret = irq_set_irq_type(gpio_irq,  IRQF_TRIGGER_RISING);
    if (ret) {
        dev_err(dev, "gpio_to_irq: FAILED\n");
        return ret;
    }
    ret = rtdm_irq_request(&rtdm_fpga_irq, gpio_irq, fpga_irq_handler, RTDM_IRQTYPE_EDGE, "FPGA_IRQ", NULL);
    if (ret) {
        dev_err(dev, "rtdm_irq_request: FAILED\n");
        return ret;
    }

    /* Initialise the FPGA GPIO reset line */
    ret = gpio_request(FPGA_RESET_GPIO, "FPGA_RESET");
    if (ret) {
        dev_err(dev, "gpio_request: FAILED\n");
        return ret;
    }	
    ret = gpio_direction_output(FPGA_RESET_GPIO, 1);
    if (ret) {
        dev_err(dev, "gpio_direction_output: FAILED\n");
        return ret;
    }

#if SIMULATE_MELBINST_HAT == 1
    /* Initialise the GPIO line used to simulate the FPGA interrupt */
    ret = gpio_request(SIMULATE_FPGA_INTERUPT_GPIO, "Simulate_FPGA_IRQ");
    if (ret) {
        dev_err(dev, "gpio_request: FAILED\n");
        return ret;
    }	
    ret = gpio_direction_output(SIMULATE_FPGA_INTERUPT_GPIO, 0);
    if (ret) {
        dev_err(dev, "gpio_direction_output: FAILED\n");
        return ret;
    }		
#endif
    return 0;
}

static int bcm2835_dma_init()
{
    struct audio_rtdm_dev *audio_dev = audio_static_dev;
    dma_addr_t dma_phys_addr = audio_dev->spi_dma_addr;

    /* Initialise the DMA for each SPI */
    bcm2835_spi_dma_init(&audio_dev->spi0_dev);
#if MELBINST_HAT == 0
    bcm2835_spi_dma_init(&audio_dev->spi4_dev);
#endif

    /* Prepare the SPI slave transfer 0 */
    audio_dev->spi0_dev.tx0_desc = bcm2835_spi_prep_slave(&audio_dev->spi0_dev, 
                                dma_phys_addr, 
                                SPI_BUFFER_TRANSFER_SIZE,
                                true);
    if (!audio_dev->spi0_dev.tx0_desc) {
        dev_info(audio_dev->spi0_dev.dev,
                "dma: bcm2835_spi_prep_slave tx0\n");
        return -EINVAL;
    }
    dma_phys_addr += SPI_BUFFER_TRANSFER_SIZE;
#if MELBINST_HAT == 0
    audio_dev->spi4_dev.tx0_desc = bcm2835_spi_prep_slave(&audio_dev->spi4_dev, 
                                dma_phys_addr, 
                                SPI_BUFFER_TRANSFER_SIZE,
                                true);
    if (!audio_dev->spi4_dev.tx0_desc) {
        dev_info(audio_dev->spi4_dev.dev,
                "dma: bcm2835_spi_prep_slave tx0\n");
        return -EINVAL;
    }
    dma_phys_addr += SPI_BUFFER_TRANSFER_SIZE;
#endif
    audio_dev->spi0_dev.rx0_desc = bcm2835_spi_prep_slave(&audio_dev->spi0_dev, 
                                dma_phys_addr,
                                SPI_BUFFER_TRANSFER_SIZE,
                                false);
    if (!audio_dev->spi0_dev.rx0_desc) {
        dev_info(audio_dev->spi0_dev.dev,
                "dma: bcm2835_spi_prep_slave rx0\n");
        return -EINVAL;
    }
    dma_phys_addr += SPI_BUFFER_TRANSFER_SIZE;
#if MELBINST_HAT == 0
    audio_dev->spi4_dev.rx0_desc = bcm2835_spi_prep_slave(&audio_dev->spi4_dev, 
                                dma_phys_addr,
                                SPI_BUFFER_TRANSFER_SIZE,
                                false);
    if (!audio_dev->spi4_dev.rx0_desc) {
        dev_info(audio_dev->spi4_dev.dev,
                "dma: bcm2835_spi_prep_slave rx0\n");
        return -EINVAL;
    }
    dma_phys_addr += SPI_BUFFER_TRANSFER_SIZE;
#endif

    /* Prepare the SPI slave transfer 1 */
    audio_dev->spi0_dev.tx1_desc = bcm2835_spi_prep_slave(&audio_dev->spi0_dev, 
                                dma_phys_addr, 
                                SPI_BUFFER_TRANSFER_SIZE,
                                true);
    if (!audio_dev->spi0_dev.tx1_desc) {
        dev_info(audio_dev->spi0_dev.dev,
                "dma: bcm2835_spi_prep_slave tx1\n");
        return -EINVAL;
    }
    dma_phys_addr += SPI_BUFFER_TRANSFER_SIZE;
#if MELBINST_HAT == 0
    audio_dev->spi4_dev.tx1_desc = bcm2835_spi_prep_slave(&audio_dev->spi4_dev, 
                                dma_phys_addr, 
                                SPI_BUFFER_TRANSFER_SIZE,
                                true);
    if (!audio_dev->spi4_dev.tx1_desc) {
        dev_info(audio_dev->spi4_dev.dev,
                "dma: bcm2835_spi_prep_slave tx1\n");
        return -EINVAL;
    }
    dma_phys_addr += SPI_BUFFER_TRANSFER_SIZE;
#endif            
    audio_dev->spi0_dev.rx1_desc = bcm2835_spi_prep_slave(&audio_dev->spi0_dev, 
                                dma_phys_addr,
                                SPI_BUFFER_TRANSFER_SIZE,
                                false);
    if (!audio_dev->spi0_dev.rx1_desc) {
        dev_info(audio_dev->spi0_dev.dev,
                "dma: bcm2835_spi_prep_slave rx1\n");
        return -EINVAL;
    }
#if MELBINST_HAT == 0
    dma_phys_addr += SPI_BUFFER_TRANSFER_SIZE;        
    audio_dev->spi4_dev.rx1_desc = bcm2835_spi_prep_slave(&audio_dev->spi4_dev, 
                                dma_phys_addr,
                                SPI_BUFFER_TRANSFER_SIZE,
                                false);
    if (!audio_dev->spi4_dev.rx1_desc) {
        dev_info(audio_dev->spi4_dev.dev,
                "dma: bcm2835_spi_prep_slave rx1\n");
        return -EINVAL;
    }
#endif
    return 0;
}

static void bcm2835_dma_release(void)
{
    struct audio_rtdm_dev *audio_dev = audio_static_dev;

    /* Release the DMA channels */
    bcm2835_spi_dma_release(&audio_dev->spi0_dev);
#if MELBINST_HAT == 0    
    bcm2835_spi_dma_release(&audio_dev->spi4_dev);
#endif
}

static int fpga_irq_handler(rtdm_irq_t *irq_handle)
{
    struct audio_rtdm_dev *audio_dev = audio_static_dev;

    /* Increment the interrupt count and set the new audio buffer index */
    audio_dev->kinterrupts++;
    if (audio_dev->wait_flag) {
        /* Signal Sushi via Raspa that an IRQ has occurred */
        audio_dev->rtdm_event_signalled = 1;
        rtdm_event_pulse(&audio_dev->irq_event);
    }
    return RTDM_IRQ_HANDLED;
}

#if SIMULATE_MELBINST_HAT == 1
static void simulate_fpga_sample_rate(rtdm_timer_t *timer) {
    // The simulate FPGA GPIO line is used to generate an interrupt
    // on the actual FPGA GPIO interrupt line
    // These must connected to each other with a jumper
    gpio_set_value(SIMULATE_FPGA_INTERUPT_GPIO, 1);
    udelay(1);
    gpio_set_value(SIMULATE_FPGA_INTERUPT_GPIO, 0);
}
#endif

int bcm2835_spi_init(int audio_buffer_size, int audio_channels, char *audio_hat)
{
	struct audio_rtdm_dev *audio_dev = audio_static_dev;
	struct rtdm_spi_config config;
	dma_addr_t dma_phys_addr;
    void *p;
    int ret;

    // Log the ELK pi hat being used
	audio_dev->audio_hat = audio_hat;
	rtdm_printk(KERN_INFO "Melbourne Instruments hat: %s\n", audio_dev->audio_hat);

    /* Initialise the IRQ event used to signal the audio rtdm */
    rtdm_event_init(&audio_dev->irq_event, 0);

    /* Initialise GPIO use */
    ret = bcm2835_gpio_init(audio_dev->spi0_dev.dev);
    if (ret < 0) {
        return -EINVAL;
    }

    /* 
     ** Has the SPI buffer been allocated?
     ** If it has, we can assume the SPI has been intialised, but
     ** the SPI exit routine not called - this can happen if Sushi is
     ** stopped with pkill/kill
     ** In this case there is no need to initialise the SPI
     */
    if (audio_dev->spi_buf == 0) {
        /* Allocate memory for the SPI transfers */
        p = dma_alloc_coherent(audio_dev->spi0_dev.dev, 
                               (RESERVED_SPI_BUFFER_SIZE_IN_PAGES * PAGE_SIZE),
                               &dma_phys_addr,
                               GFP_KERNEL);
        if (!p) {
            rtdm_printk(KERN_ERR "bcm2835-spi: couldn't allocate dma mem\n");
            return -ENOMEM;
        }
        audio_dev->spi_buf = p;
        audio_dev->spi_dma_addr = dma_phys_addr;

        /* Note: SPI DMA initialisation is done on device open */

        // Configure the SPIs
        config.mode = SPI_MODE_2;
        config.bits_per_word = 8;
        config.speed_hz = 62500000;
        bcm2835_spi_configure(&config, &audio_dev->spi0_dev);
#if MELBINST_HAT == 0
        bcm2835_spi_configure(&config, &audio_dev->spi4_dev);
#endif

#if SIMULATE_MELBINST_HAT == 1
        /* Create the sample rate timer, used to generate an interrupt using GPIO */
        rtdm_timer_init(&rtdm_fpga_timer, simulate_fpga_sample_rate, "simulate_fpga_sample_rate");
        rtdm_timer_start(&rtdm_fpga_timer, 1000000000, (1000000000 / (MELBINST_PI_SAMPLING_RATE / MELBINST_PI_BUFFER_SIZE)), RTDM_TIMERMODE_REALTIME);
        rtdm_printk(KERN_ERR "bcm2835-spi: SIMULATE_MELBINST_HAT\n");
#endif
    }
    else {
        // Ensure the DMA memory is zeroed on initialisation, even if already allocated
        memset(audio_dev->spi_buf, 0, RESERVED_SPI_BUFFER_SIZE_IN_PAGES * PAGE_SIZE);
    }
	return 0;
}
EXPORT_SYMBOL_GPL(bcm2835_spi_init);

struct audio_rtdm_dev *bcm2835_spi_open(void)
{
    struct audio_rtdm_dev *audio_dev = audio_static_dev;

    /* Reset fifo and HW */
    bcm2835_spi_reset_hw(audio_dev->spi0_dev.base_addr);
#if MELBINST_HAT == 0
    bcm2835_spi_reset_hw(audio_dev->spi4_dev.base_addr);
#endif

    // Ensure the DMA memory is zero'd on initialisation, even if already allocated
    memset(audio_dev->spi_buf, 0, RESERVED_SPI_BUFFER_SIZE_IN_PAGES * PAGE_SIZE);

    /* Initialise the SPI DMAs */
    bcm2835_dma_init();

    /* Reset the FPGA */
    gpio_set_value(FPGA_RESET_GPIO, 0);
    udelay(1);
    gpio_set_value(FPGA_RESET_GPIO, 1);

    /* Return the device descriptor */
    return audio_static_dev;
}
EXPORT_SYMBOL_GPL(bcm2835_spi_open);

bool bcm2835_spi_transfer()
{
    struct audio_rtdm_dev *audio_dev = audio_static_dev;
    spl_t s;

    /* Reset the FPGA */
    cobalt_atomic_enter(s);
    gpio_set_value(FPGA_RESET_GPIO, 0);
    udelay(1);
    gpio_set_value(FPGA_RESET_GPIO, 1);
    cobalt_atomic_leave(s);

    /* 
     ** Check both previous transfers were OK
     ** If not, do not initiate the next transfer
     */
#if MELBINST_HAT == 0    
    if (!bcm2835_spi_dma_check_last_transfer(&audio_dev->spi0_dev) ||
        !bcm2835_spi_dma_check_last_transfer(&audio_dev->spi4_dev)) {
#else
    if (!bcm2835_spi_dma_check_last_transfer(&audio_dev->spi0_dev)) {
#endif
        return false;
    }

    /* Start the SPI DMA transfers */
    bcm2835_spi_dma_transfer(&audio_dev->spi0_dev);
#if MELBINST_HAT == 0    
    bcm2835_spi_dma_transfer(&audio_dev->spi4_dev);
#endif
    return true;
}
EXPORT_SYMBOL_GPL(bcm2835_spi_transfer);

void bcm2835_spi_close(void)
{
    /* Release the DMA channels */
    bcm2835_dma_release(); 
}
EXPORT_SYMBOL_GPL(bcm2835_spi_close);

int bcm2835_spi_exit(void)
{
    struct audio_rtdm_dev *audio_dev = audio_static_dev;

    /* Release the DMA channels */
    bcm2835_dma_release();

    /* Free the allocated SPI buffer */
    dma_free_coherent(audio_dev->spi0_dev.dev,
                      (RESERVED_SPI_BUFFER_SIZE_IN_PAGES * PAGE_SIZE),
                      audio_dev->spi_buf,
                      audio_dev->spi_dma_addr);

#if SIMULATE_MELBINST_HAT == 1
    /* Stop and destroy the FPGA timer */
	rtdm_timer_stop(&rtdm_fpga_timer);
	rtdm_timer_destroy(&rtdm_fpga_timer);
#endif
	return 0;
}
EXPORT_SYMBOL_GPL(bcm2835_spi_exit);

static const struct of_device_id bcm2835_spi_match[] = {
    { .compatible = "brcm,rtdm-bcm2835-spi", },
    { /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, bcm2835_spi_match);

static struct platform_driver bcm2835_spi_driver = {
    .driver = {
        .name           = "bcm2835-spi-melbinst",
        .of_match_table	= bcm2835_spi_match,
    },
    .probe  = bcm2835_spi_probe,
    .remove = bcm2835_spi_remove,
};

module_platform_driver(bcm2835_spi_driver);
#if MELBINST_HAT == 0
MODULE_DESCRIPTION("BCM2835 SPI interface for NINA RPi");
#else
MODULE_DESCRIPTION("BCM2835 SPI interface for DELIA RPi");
#endif
MODULE_AUTHOR("Melbourne Instruments");
MODULE_LICENSE("GPL");
