/*
 * LitePCIe driver
 *
 * Copyright (C) 2018-2020 / EnjoyDigital  / florent@enjoy-digital.fr
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/mmtimer.h>
#include <linux/miscdevice.h>
#include <linux/posix-timers.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/math64.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/log2.h>
#include <linux/cdev.h>
#include <linux/poll.h>

/* V4L2 */
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#include "litepcie.h"
#include "csr.h"
#include "config.h"
#include "flags.h"

#define PCIE_CHANNEL 0

//#define DEBUG_CSR
//#define DEBUG_MSI
#define DEBUG_POLL
//#define DEBUG_READ
//#define DEBUG_WRITE
//#define DEBUG_FNCTS
//#define DEBUG_PHASE
//#define DEBUG_RES
#define DEBUG_HDMI_STATUS

#define LITEPCIE_NAME "litepcie"
#define LITEPCIE_MINOR_COUNT 32

#define HDMI2PCIE_DIR_IN 0
#define HDMI2PCIE_DIR_OUT 1

#define PCIE_BUF_SIZE 0x400000
#define READ_BUF_OFF (7*PCIE_BUF_SIZE)

#define SLOT_EMPTY      0
#define SLOT_LOADED     1
#define SLOT_PENDING    2

#define CAP_DLY_CTL_OFF		0
#define CAP_PHASE_OFF		4
#define CAP_PHASE_RES_OFF	8

#define DELAY_RST			0x01
#define DELAY_MASTER_INC	0x02
#define DELAY_MASTER_DEC	0x04
#define DELAY_SLAVE_INC		0x08
#define DELAY_SLAVE_DEC		0x10

#define DELAY_TOO_LATE		1
#define DELAY_TOO_EARLY		2

struct litepcie_dma_chan {
	uint32_t base;
	uint32_t writer_interrupt;
	uint32_t reader_interrupt;
	dma_addr_t reader_handle[DMA_BUFFER_COUNT];
	dma_addr_t writer_handle[DMA_BUFFER_COUNT];
	uint32_t *reader_addr[DMA_BUFFER_COUNT];
	uint32_t *writer_addr[DMA_BUFFER_COUNT];
	int64_t reader_hw_count;
	int64_t reader_hw_count_last;
	int64_t reader_sw_count;
	int64_t writer_hw_count;
	int64_t writer_hw_count_last;
	int64_t writer_sw_count;
	uint8_t writer_enable;
	uint8_t reader_enable;
	uint8_t writer_lock;
	uint8_t reader_lock;
};

struct litepcie_chan {
	struct litepcie_device *litepcie_dev;
	struct litepcie_dma_chan dma;
	struct cdev *cdev;
	uint32_t block_size;
	uint32_t core_base;
	wait_queue_head_t wait_rd; /* to wait for an ongoing read */
	wait_queue_head_t wait_wr; /* to wait for an ongoing write */

	int index;
	int minor;
};

struct litepcie_hdmi_rx {
	int lock_status;
    int locked;
	u64 lock_timestamp;
	u64 service_timestamp;

	int data_dly[3];
};

struct litepcie_device {
	struct pci_dev  *dev;
	resource_size_t bar0_size;
	phys_addr_t bar0_phys_addr;
	uint8_t *bar0_addr; /* virtual address of BAR0 */
	struct litepcie_chan chan[DMA_CHANNEL_COUNT];
    struct litepcie_hdmi_rx rx;
	struct list_head list;
	spinlock_t lock;
	int minor_base;
	int channels_count;
	struct vid_channel *channels;
};

struct litepcie_chan_priv {
	struct litepcie_chan *chan;
	bool reader;
	bool writer;
};

struct vid_channel {
	uint8_t dir;
	uint32_t sequence;
	struct litepcie_device *common;

	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct mutex lock;

	struct v4l2_dv_timings timings;
	struct v4l2_pix_format format;

	unsigned int *fb_idx;

	spinlock_t qlock;
	struct vb2_queue queue;
	struct list_head buf_list;
	uint8_t streaming;
};

static const struct v4l2_dv_timings_cap hdmi2pcie_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	.reserved = { 0 },
	V4L2_INIT_BT_TIMINGS(
		640, 1920,
		480, 1080,
		25000000, 148500000,
		V4L2_DV_BT_STD_CEA861,
		V4L2_DV_BT_CAP_PROGRESSIVE
	)
};

struct hdmi2pcie_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

static LIST_HEAD(litepcie_list);

static int litepcie_major;
static int litepcie_minor_idx;
static struct class* litepcie_class;
static dev_t litepcie_dev_t;

static inline uint32_t litepcie_readl(struct litepcie_device  *s, uint32_t addr)
{
	uint32_t val;
	val = readl(s->bar0_addr + addr - CSR_BASE);

#ifdef DEBUG_CSR
	printk("csr_read: 0x%08x @ 0x%08x", val, addr);
#endif

	return val;
}

static inline void litepcie_writel(struct litepcie_device *s, uint32_t addr, uint32_t val)
{
#ifdef DEBUG_CSR
	printk("csr_write: 0x%08x @ 0x%08x", val, addr);
#endif

	return writel(val, s->bar0_addr + addr - CSR_BASE);
}

static void litepcie_enable_interrupt(struct litepcie_device *s, int irq_num)
{
	uint32_t v;

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	v = litepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
	v |= (1 << irq_num);
	litepcie_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

static void litepcie_disable_interrupt(struct litepcie_device *s, int irq_num)
{
	uint32_t v;

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	v = litepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
	v &= ~(1 << irq_num);
	litepcie_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

static int litepcie_hdmi_rx_mmcm_locked_filtered(struct litepcie_device *s)
{
	if(litepcie_readl(s, CSR_HDMI_IN0_CLOCKING_LOCKED_ADDR)) {
		switch(s->rx.lock_status) {
			case 0:
				s->rx.lock_timestamp = ktime_get_ns();
				s->rx.lock_status = 1;
				break;
			case 1:
				if((s->rx.lock_timestamp + 250*1000*1000) < ktime_get_ns())
					s->rx.lock_status = 2;
				break;
			case 2:
				return 1;
		}
	} else {
		s->rx.lock_status = 0;
	}
	return 0;
}

static void litepcie_hdmi_rx_adjust_phase(struct litepcie_device *s)
{
	switch(litepcie_readl(s, CSR_HDMI_IN0_DATA0_CAP_PHASE_ADDR)) {
		case DELAY_TOO_LATE:
			litepcie_writel(s, CSR_HDMI_IN0_DATA0_CAP_DLY_CTL_ADDR, DELAY_MASTER_DEC | DELAY_SLAVE_DEC);
			s->rx.data_dly[0]--;
			litepcie_writel(s, CSR_HDMI_IN0_DATA0_CAP_PHASE_RESET_ADDR, 1);
			break;
		case DELAY_TOO_EARLY:
			litepcie_writel(s, CSR_HDMI_IN0_DATA0_CAP_DLY_CTL_ADDR, DELAY_MASTER_INC | DELAY_SLAVE_INC);
			s->rx.data_dly[0]++;
			litepcie_writel(s, CSR_HDMI_IN0_DATA0_CAP_PHASE_RESET_ADDR, 1);
			break;
	}

	switch(litepcie_readl(s, CSR_HDMI_IN0_DATA1_CAP_PHASE_ADDR)) {
		case DELAY_TOO_LATE:
			litepcie_writel(s, CSR_HDMI_IN0_DATA1_CAP_DLY_CTL_ADDR, DELAY_MASTER_DEC | DELAY_SLAVE_DEC);
			s->rx.data_dly[1]--;
			litepcie_writel(s, CSR_HDMI_IN0_DATA1_CAP_PHASE_RESET_ADDR, 1);
			break;
		case DELAY_TOO_EARLY:
			litepcie_writel(s, CSR_HDMI_IN0_DATA1_CAP_DLY_CTL_ADDR, DELAY_MASTER_INC | DELAY_SLAVE_INC);
			s->rx.data_dly[1]++;
			litepcie_writel(s, CSR_HDMI_IN0_DATA1_CAP_PHASE_RESET_ADDR, 1);
			break;
	}

	switch(litepcie_readl(s, CSR_HDMI_IN0_DATA2_CAP_PHASE_ADDR)) {
		case DELAY_TOO_LATE:
			litepcie_writel(s, CSR_HDMI_IN0_DATA2_CAP_DLY_CTL_ADDR, DELAY_MASTER_DEC | DELAY_SLAVE_DEC);
			s->rx.data_dly[2]--;
			litepcie_writel(s, CSR_HDMI_IN0_DATA2_CAP_PHASE_RESET_ADDR, 1);
			break;
		case DELAY_TOO_EARLY:
			litepcie_writel(s, CSR_HDMI_IN0_DATA2_CAP_DLY_CTL_ADDR, DELAY_MASTER_INC | DELAY_SLAVE_INC);
			s->rx.data_dly[2]++;
			litepcie_writel(s, CSR_HDMI_IN0_DATA2_CAP_PHASE_RESET_ADDR, 1);
			break;
	}
}

static int litepcie_hdmi_rx_init_phase(struct litepcie_device *s)
{
	int data_dly[3];
	int i, j, k;

	for (i = 0; i < 100; i++) {
		for (j = 0; j < 3; j++)
			data_dly[j] = s->rx.data_dly[j];

		for (k = 0; k < 1000; k++) {
			litepcie_hdmi_rx_adjust_phase(s);
			udelay(10);
		}

		if ((abs(s->rx.data_dly[0] - data_dly[0]) < 4) &&
			(abs(s->rx.data_dly[1] - data_dly[1]) < 4) &&
			(abs(s->rx.data_dly[2] - data_dly[2]) < 4)) {
			return 1;
		}
	}
	return 0;
}

static void litepcie_hdmi_rx_calibrate_delays(struct litepcie_device *s)
{
	int i, delay, freq = 7425;
	litepcie_writel(s, CSR_HDMI_IN0_DATA0_CAP_DLY_CTL_ADDR, DELAY_RST);
	litepcie_writel(s, CSR_HDMI_IN0_DATA1_CAP_DLY_CTL_ADDR, DELAY_RST);
	litepcie_writel(s, CSR_HDMI_IN0_DATA2_CAP_DLY_CTL_ADDR, DELAY_RST);
	litepcie_writel(s, CSR_HDMI_IN0_DATA0_CAP_PHASE_RESET_ADDR, 1);
	litepcie_writel(s, CSR_HDMI_IN0_DATA1_CAP_PHASE_RESET_ADDR, 1);
	litepcie_writel(s, CSR_HDMI_IN0_DATA2_CAP_PHASE_RESET_ADDR, 1);
	s->rx.data_dly[0] = s->rx.data_dly[1] = s->rx.data_dly[2] = 0;

#if 0
	for (i = 0; i < 16; i++) {
		litepcie_writel(s, CSR_HDMI_IN0_DATA0_CAP_DLY_CTL_ADDR, DELAY_SLAVE_INC | DELAY_MASTER_INC);
		litepcie_writel(s, CSR_HDMI_IN0_DATA1_CAP_DLY_CTL_ADDR, DELAY_SLAVE_INC | DELAY_MASTER_INC);
		litepcie_writel(s, CSR_HDMI_IN0_DATA2_CAP_DLY_CTL_ADDR, DELAY_SLAVE_INC | DELAY_MASTER_INC);
	}

	s->rx.data_dly[0] = s->rx.data_dly[1] = s->rx.data_dly[2] = 15;
#endif

	/* preload slave phase detector idelay with 90° phase shift
	  (78 ps taps on 7-series) */
	delay = 10000000/(4*freq*78);
	for (i = 0; i < delay; i++) {
		litepcie_writel(s, CSR_HDMI_IN0_DATA0_CAP_DLY_CTL_ADDR, DELAY_SLAVE_INC);
		litepcie_writel(s, CSR_HDMI_IN0_DATA1_CAP_DLY_CTL_ADDR, DELAY_SLAVE_INC);
		litepcie_writel(s, CSR_HDMI_IN0_DATA2_CAP_DLY_CTL_ADDR, DELAY_SLAVE_INC);
	}
}

static int litepcie_hdmi_rx_phase_startup(struct litepcie_device *s)
{
	int attempts = 0;
	while(1) {
		attempts++;
		litepcie_hdmi_rx_calibrate_delays(s);
		if(litepcie_hdmi_rx_init_phase(s)) {
			printk(KERN_ERR LITEPCIE_NAME " phase init OK\n");
			return 1;
		} else if (attempts > 3) {
			printk(KERN_ERR LITEPCIE_NAME " phase init failed, giving up\n");
			litepcie_hdmi_rx_calibrate_delays(s);
			return 0;
		} else {
			printk(KERN_ERR LITEPCIE_NAME " phase init failed\n");
		}
	}
}

static void litepcie_hdmi_rx_print_status(struct litepcie_device *s)
{
	int delay[3], charsync[3], charsync_ctl[3], wer[3], chansync, hres, vres;

	litepcie_writel(s, CSR_HDMI_IN0_DATA0_WER_UPDATE_ADDR, 1);
	litepcie_writel(s, CSR_HDMI_IN0_DATA1_WER_UPDATE_ADDR, 1);
	litepcie_writel(s, CSR_HDMI_IN0_DATA2_WER_UPDATE_ADDR, 1);

	delay[0] = s->rx.data_dly[0];
	delay[1] = s->rx.data_dly[1];
	delay[2] = s->rx.data_dly[2];

	charsync[0] = litepcie_readl(s, CSR_HDMI_IN0_DATA0_CHARSYNC_CHAR_SYNCED_ADDR);
	charsync[1] = litepcie_readl(s, CSR_HDMI_IN0_DATA1_CHARSYNC_CHAR_SYNCED_ADDR);
	charsync[2] = litepcie_readl(s, CSR_HDMI_IN0_DATA2_CHARSYNC_CHAR_SYNCED_ADDR);

	charsync_ctl[0] = litepcie_readl(s, CSR_HDMI_IN0_DATA0_CHARSYNC_CTL_POS_ADDR);
	charsync_ctl[1] = litepcie_readl(s, CSR_HDMI_IN0_DATA1_CHARSYNC_CTL_POS_ADDR);
	charsync_ctl[2] = litepcie_readl(s, CSR_HDMI_IN0_DATA2_CHARSYNC_CTL_POS_ADDR);

	wer[0] = litepcie_readl(s, CSR_HDMI_IN0_DATA0_WER_VALUE_ADDR);
	wer[1] = litepcie_readl(s, CSR_HDMI_IN0_DATA1_WER_VALUE_ADDR);
	wer[2] = litepcie_readl(s, CSR_HDMI_IN0_DATA2_WER_VALUE_ADDR);

	chansync = litepcie_readl(s, CSR_HDMI_IN0_CHANSYNC_CHANNELS_SYNCED_ADDR);
	
	hres = litepcie_readl(s, CSR_HDMI_IN0_RESDETECTION_HRES_ADDR);
	vres = litepcie_readl(s, CSR_HDMI_IN0_RESDETECTION_VRES_ADDR);

	printk(KERN_ERR LITEPCIE_NAME " ph %4d %4d %4d // charsync %d%d%d [%d %d %d] // WER %3d %3d %3d // chansync %d // res %dx%d\n",
		delay[0], delay[1], delay[2],
		charsync[0], charsync[1], charsync[2],
		charsync_ctl[0], charsync_ctl[1], charsync_ctl[2],
		wer[0], wer[1], wer[2],
		chansync, hres, vres);
}

static void litepcie_hdmi_rx_service(struct litepcie_device *s)
{
    int ret;
    if(s->rx.locked) {
		if(litepcie_hdmi_rx_mmcm_locked_filtered(s)) {
			if((s->rx.service_timestamp + 500*1000*1000) < ktime_get_ns()) {
				s->rx.service_timestamp = ktime_get_ns();
				litepcie_hdmi_rx_adjust_phase(s);
#ifdef DEBUG_HDMI_STATUS
				litepcie_hdmi_rx_print_status(s);
#endif
			}
		} else {
			printk(KERN_ERR LITEPCIE_NAME " lost PLL lock\n");
			s->rx.locked = 0;
		}
    } else {
		if(litepcie_hdmi_rx_mmcm_locked_filtered(s)) {
			printk(KERN_ERR LITEPCIE_NAME " PLL locked\n");
			litepcie_hdmi_rx_phase_startup(s);
			s->rx.locked = 1;
		}
	}
	if(litepcie_readl(s, CSR_HDMI_IN0_FRAME_OVERFLOW_ADDR)) {
		printk(KERN_ERR LITEPCIE_NAME " HDMI IN FIFO overflow\n");
		litepcie_writel(s, CSR_HDMI_IN0_FRAME_OVERFLOW_ADDR, 1);
	}
#ifdef DEBUG_RES
#endif
}

static void litepcie_hdmi_rx_mmcm_write(struct litepcie_device *s, int adr, int data)
{
	litepcie_writel(s, CSR_HDMI_IN0_CLOCKING_MMCM_ADR_ADDR, adr);
	litepcie_writel(s, CSR_HDMI_IN0_CLOCKING_MMCM_DAT_W_ADDR, data);
	litepcie_writel(s, CSR_HDMI_IN0_CLOCKING_MMCM_WRITE_ADDR, 1);
	while(!litepcie_readl(s, CSR_HDMI_IN0_CLOCKING_MMCM_DRDY_ADDR));
}

int hdmi_rx_mmcm_settings [3][5] = {
    /* 30 to 60 MHz */
	{0x1000 | (10<<6) | 10, 0x1000 | (10<<6) | 10, 0x1000 |  (8<<6) |  8, 0x1000 |  (2<<6) |  2, 0},
    /* 60 to 120 MHz */
	{0x1000 |  (5<<6) | 5, 0x1000 |  (5<<6) | 5, 0x1000 |  (4<<6) | 4, 0x1000 |  (1<<6) | 1, 0},
    /* 120 to 240 MHz */
	{0x1000 |  (2<<6) | 3, 0x1000 |  (2<<6) | 3, 0x1000 |  (2<<6) | 2, 0x1000 |  (0<<6) | 0, (1<<6)},
};

int hdmi_rx_mmcm_regs [5] = {0x14, 0x08, 0x0a, 0x0c, 0x0d};

static void litepcie_hdmi_rx_mmcm_init(struct litepcie_device *s)
{
	int i;
	/* 1280p60 settings */
	for (i = 0; i < 5; i++) {
		printk(KERN_ERR LITEPCIE_NAME " MMCM write 0x%x to 0x%x\n", hdmi_rx_mmcm_settings[1][i], hdmi_rx_mmcm_regs[i]);
		litepcie_hdmi_rx_mmcm_write(s, hdmi_rx_mmcm_regs[i], hdmi_rx_mmcm_settings[1][i]);
	}
}

static void litepcie_dma_writer_start_addr(struct litepcie_device *s, int chan_num, dma_addr_t addr)
{
	struct litepcie_dma_chan *dmachan;
	int i;

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	dmachan = &s->chan[chan_num].dma;

	/* fill dma writer descriptors */
	litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 0);
	litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_FLUSH_OFFSET, 1);
	litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_OFFSET, 0);
	for(i = 0; i < DMA_BUFFER_COUNT; i++) {
		litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_VALUE_OFFSET,
#ifndef DMA_BUFFER_ALIGNED
				DMA_LAST_DISABLE |
#endif
				(!(i%DMA_BUFFER_PER_IRQ == 0)) * DMA_IRQ_DISABLE | /* generate an msi */
				DMA_BUFFER_SIZE);                                  /* every n buffers */
		litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_VALUE_OFFSET + 4,
				addr);
		litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_WE_OFFSET, 1);
	}

	/* clear counters */
	dmachan->writer_hw_count = 0;
	dmachan->writer_hw_count_last = 0;

	/* start dma writer */
	litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 1);
}

static void litepcie_dma_writer_stop(struct litepcie_device *s, int chan_num)
{
	struct litepcie_dma_chan *dmachan;

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	dmachan = &s->chan[chan_num].dma;

	/* flush and stop dma writer */
	litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_OFFSET, 0);
	litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_FLUSH_OFFSET, 1);
	udelay(1000);
	litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 0);
	litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_FLUSH_OFFSET, 1);

	/* clear counters */
	dmachan->writer_hw_count = 0;
	dmachan->writer_hw_count_last = 0;
	dmachan->writer_sw_count = 0;
}

static void litepcie_dma_reader_start_addr(struct litepcie_device *s, int chan_num, dma_addr_t addr)
{
	struct litepcie_dma_chan *dmachan;
	int i;

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	dmachan = &s->chan[chan_num].dma;

	/* fill dma reader descriptors */
	litepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET , 0);
	litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_FLUSH_OFFSET, 1);
	litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_LOOP_PROG_N_OFFSET, 0);
	for(i = 0; i < DMA_BUFFER_COUNT; i++) {
		litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_VALUE_OFFSET,
#ifndef DMA_BUFFER_ALIGNED
				DMA_LAST_DISABLE |
#endif
				(!(i%DMA_BUFFER_PER_IRQ == 0)) * DMA_IRQ_DISABLE | /* generate an msi */
				DMA_BUFFER_SIZE);                                  /* every n buffers */
		litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_VALUE_OFFSET + 4,
				addr);
		litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_WE_OFFSET, 1);
	}
	//litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_LOOP_PROG_N_OFFSET, 1);

	/* clear counters */
	dmachan->reader_hw_count = 0;
	dmachan->reader_hw_count_last = 0;

	/* start dma reader */
	litepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 1);
}

static void litepcie_dma_reader_stop(struct litepcie_device *s, int chan_num)
{
	struct litepcie_dma_chan *dmachan;

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	dmachan = &s->chan[chan_num].dma;

	/* flush and stop dma reader */
	litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_LOOP_PROG_N_OFFSET, 0);
	litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_FLUSH_OFFSET, 1);
	udelay(1000);
	litepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 0);
	litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_FLUSH_OFFSET, 1);

	/* clear counters */
	dmachan->reader_hw_count = 0;
	dmachan->reader_hw_count_last = 0;
	dmachan->reader_sw_count = 0;
}

void litepcie_stop_dma(struct litepcie_device *s)
{
	struct litepcie_dma_chan *dmachan;
	int i;

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	for(i = 0; i < s->channels_count; i++) {
		dmachan = &s->chan[i].dma;
		litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 0);
		litepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 0);
	}
}

static irqreturn_t litepcie_interrupt(int irq, void *data)
{
	struct litepcie_device *s = (struct litepcie_device*) data;
	struct litepcie_chan *chan;
	uint32_t clear_mask, irq_vector, irq_enable;
	int i;

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	irq_vector = litepcie_readl(s, CSR_PCIE_MSI_VECTOR_ADDR);
	irq_enable = litepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
#ifdef DEBUG_MSI
	printk(KERN_INFO LITEPCIE_NAME " MSI: 0x%x 0x%x\n", irq_vector, irq_enable);
#endif
	irq_vector &= irq_enable;
	clear_mask = 0;

	for(i = 0; i < s->channels_count; i++) {
		chan = &s->chan[i];
		/* dma reader interrupt handling */
		if(irq_vector & (1 << chan->dma.reader_interrupt)) {
			chan->dma.reader_hw_count ++;
#ifdef DEBUG_MSI
			printk(KERN_INFO LITEPCIE_NAME " MSI DMA%d Reader buf: %lld\n", i, chan->dma.reader_hw_count);
#endif
			wake_up_interruptible(&chan->wait_wr);
			clear_mask |= (1 << chan->dma.reader_interrupt);
		}
		/* dma writer interrupt handling */
		if(irq_vector & (1 << chan->dma.writer_interrupt)) {
			chan->dma.writer_hw_count ++;
#ifdef DEBUG_MSI
			printk(KERN_INFO LITEPCIE_NAME " MSI DMA%d WWriter buf: %lld\n", i, chan->dma.writer_hw_count);
#endif
			wake_up_interruptible(&chan->wait_rd);
			clear_mask |= (1 << chan->dma.writer_interrupt);
		}
	}

	if (irq_vector & (1 << HDMI_IN0_DMA_INTERRUPT)) {
		printk(KERN_INFO LITEPCIE_NAME " HDMI_IN0_DMA_IRQ\n");

#if 1
        if (litepcie_readl(s, CSR_HDMI_IN0_DMA_SLOT0_STATUS_ADDR) == SLOT_PENDING){
            printk(KERN_ERR LITEPCIE_NAME " SLOT0\n");
            litepcie_writel(s, CSR_HDMI_IN0_DMA_SLOT0_ADDRESS_ADDR, 0x03000000);
            litepcie_writel(s, CSR_HDMI_IN0_DMA_SLOT0_STATUS_ADDR, SLOT_LOADED);
        }
        if (litepcie_readl(s, CSR_HDMI_IN0_DMA_SLOT1_STATUS_ADDR) == SLOT_PENDING){
            printk(KERN_ERR LITEPCIE_NAME " SLOT1\n");
            litepcie_writel(s, CSR_HDMI_IN0_DMA_SLOT1_ADDRESS_ADDR, 0x04000000);
            litepcie_writel(s, CSR_HDMI_IN0_DMA_SLOT1_STATUS_ADDR, SLOT_LOADED);
        }
#endif
		litepcie_writel(s, CSR_HDMI_IN0_DMA_EV_PENDING_ADDR,
			litepcie_readl(s, CSR_HDMI_IN0_DMA_EV_PENDING_ADDR));
		clear_mask |= (1 << HDMI_IN0_DMA_INTERRUPT);
	}

	litepcie_writel(s, CSR_PCIE_MSI_CLEAR_ADDR, clear_mask);

	return IRQ_HANDLED;
}

/* V4L2 fncts */

static inline struct hdmi2pcie_buffer *to_hdmi2pcie_buffer(struct vb2_buffer *vb2)
{
	return container_of(vb2, struct hdmi2pcie_buffer, vb);
}

static int queue_setup(struct vb2_queue *vq,
		unsigned int *nbuffers, unsigned int *nplanes,
		unsigned int sizes[], struct device *alloc_devs[])
{
	struct vid_channel *chan = vb2_get_drv_priv(vq);

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	if (vq->num_buffers + *nbuffers < 3)
		*nbuffers = 3 - vq->num_buffers;

	printk("%d %d %d\n", *nplanes, vq->num_buffers, *nbuffers);

	if (*nplanes)
		return sizes[0] < chan->format.sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = chan->format.sizeimage;
	return 0;
}

static int buffer_prepare(struct vb2_buffer *vb)
{
	struct vid_channel *chan = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = chan->format.sizeimage;

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(&chan->common->dev->dev, "buffer too small (%lu < %lu)\n",
				vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);
	return 0;
}

static void buffer_queue(struct vb2_buffer *vb)
{
	struct vid_channel *chan = vb2_get_drv_priv(vb->vb2_queue);
	struct litepcie_device *litepcie_dev = chan->common;
	struct litepcie_chan *pcie_chan = &litepcie_dev->chan[PCIE_CHANNEL];
	//struct hdmi2pcie_buffer *hbuf = to_hdmi2pcie_buffer(vb);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	dma_addr_t dma_buf = vb2_dma_contig_plane_dma_addr(vb, 0);
	unsigned long flags;

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	if (!chan->streaming) {
		litepcie_enable_interrupt(litepcie_dev, pcie_chan->dma.writer_interrupt);


#ifdef DMA_LOOP
		//Prepare LiteDRAMDMAReader
		litepcie_writel(litepcie_dev, CSR_DMA_READER_BASE_ADDR, 0x03000000);
		litepcie_writel(litepcie_dev, CSR_DMA_READER_LENGTH_ADDR, DMA_BUFFER_SIZE);
		litepcie_writel(litepcie_dev, CSR_DMA_READER_LOOP_ADDR, 1);
		litepcie_writel(litepcie_dev, CSR_DMA_READER_START_ADDR, 1);
#endif

		chan->streaming = 1;
	}

	spin_lock_irqsave(&chan->qlock, flags);
	//list_add_tail(&hbuf->list, &priv->buf_list);
	if (chan->dir == HDMI2PCIE_DIR_IN) {

		litepcie_dma_writer_start_addr(litepcie_dev, PCIE_CHANNEL, dma_buf);
#if 0
        if (litepcie_readl(litepcie_dev, CSR_HDMI_IN0_DMA_SLOT0_STATUS_ADDR) == SLOT_PENDING){
            printk(KERN_ERR LITEPCIE_NAME " SLOT0\n");
            litepcie_writel(litepcie_dev, CSR_HDMI_IN0_DMA_SLOT0_ADDRESS_ADDR, 0x03000000);
            litepcie_writel(litepcie_dev, CSR_HDMI_IN0_DMA_SLOT0_STATUS_ADDR, SLOT_LOADED);
        }
        if (litepcie_readl(litepcie_dev, CSR_HDMI_IN0_DMA_SLOT1_STATUS_ADDR) == SLOT_PENDING){
            printk(KERN_ERR LITEPCIE_NAME " SLOT1\n");
            litepcie_writel(litepcie_dev, CSR_HDMI_IN0_DMA_SLOT1_ADDRESS_ADDR, 0x04000000);
            litepcie_writel(litepcie_dev, CSR_HDMI_IN0_DMA_SLOT1_STATUS_ADDR, SLOT_LOADED);
        }
#endif

#ifndef DMA_LOOP
		//Prepare LiteDRAMDMAReader
		litepcie_writel(litepcie_dev, CSR_DMA_READER_BASE_ADDR, 0x03000000);
		litepcie_writel(litepcie_dev, CSR_DMA_READER_LENGTH_ADDR, DMA_BUFFER_SIZE);
		litepcie_writel(litepcie_dev, CSR_DMA_READER_LOOP_ADDR, 0);
		litepcie_writel(litepcie_dev, CSR_DMA_READER_START_ADDR, 1);
#endif
	} else {
		//litepcie_dma_reader_start_addr(litepcie_dev, PCIE_CHANNEL, dma_buf);
		//litepcie_enable_interrupt(litepcie_dev, pcie_chan->dma.reader_interrupt);
		//litepcie_disable_interrupt(litepcie_dev, pcie_chan->dma.reader_interrupt);
		//litepcie_dma_reader_stop(litepcie_dev, PCIE_CHANNEL);
	}

	litepcie_hdmi_rx_service(litepcie_dev);

	spin_unlock_irqrestore(&chan->qlock, flags);

	if (chan->dir == HDMI2PCIE_DIR_IN) {
		wait_event_interruptible(pcie_chan->wait_rd,
				pcie_chan->dma.writer_hw_count > 0);
	} else {
		//wait_event_interruptible(pcie_chan->wait_wr,
		//		pcie_chan->dma.reader_hw_count > 0);
	}

	vb->timestamp = ktime_get_ns();
	vbuf->sequence = chan->sequence++;
	vbuf->field = V4L2_FIELD_NONE;

	vb2_buffer_done(vb, VB2_BUF_STATE_DONE);

}

static void return_all_buffers(struct vid_channel *chan,
		enum vb2_buffer_state state)
{
	struct hdmi2pcie_buffer *buf, *node;
	unsigned long flags;

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	spin_lock_irqsave(&chan->qlock, flags);
	list_for_each_entry_safe(buf, node, &chan->buf_list, list) {
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		list_del(&buf->list);
	}
	spin_unlock_irqrestore(&chan->qlock, flags);
}

static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct vid_channel *chan = vb2_get_drv_priv(vq);
	int ret = 0;

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	chan->sequence = 0;

	if (ret) {
		return_all_buffers(chan, VB2_BUF_STATE_QUEUED);
	}
	return ret;
}

static void stop_streaming(struct vb2_queue *vq)
{
	struct vid_channel *chan = vb2_get_drv_priv(vq);
	struct litepcie_device *litepcie_dev = chan->common;
	struct litepcie_chan *pcie_chan = &litepcie_dev->chan[PCIE_CHANNEL];

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	//litepcie_writel(litepcie_dev, CSR_DMA_READER_INITIATOR_ENABLE_ADDR, 0);
	chan->streaming = 0;
	litepcie_disable_interrupt(litepcie_dev, pcie_chan->dma.writer_interrupt);
	litepcie_dma_writer_stop(litepcie_dev, PCIE_CHANNEL);

	return_all_buffers(chan, VB2_BUF_STATE_ERROR);
}

static struct vb2_ops hdmi2pcie_qops = {
	.queue_setup		= queue_setup,
	.buf_prepare		= buffer_prepare,
	.buf_queue		= buffer_queue,
	.start_streaming	= start_streaming,
	.stop_streaming		= stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static int hdmi2pcie_querycap(struct file *file, void *private,
		struct v4l2_capability *cap)
{
	struct vid_channel *chan = video_drvdata(file);

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	strlcpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	strlcpy(cap->card, "HDMI2PCIe", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI:%s",
			pci_name(chan->common->dev));
	return 0;
}

static void hdmi2pcie_fill_pix_format(struct vid_channel *chan,
		struct v4l2_pix_format *pix)
{
#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	pix->pixelformat = V4L2_PIX_FMT_UYVY;
	pix->width = chan->timings.bt.width;
	pix->height = chan->timings.bt.height;
	if (chan->timings.bt.interlaced) {
		pix->field = V4L2_FIELD_ALTERNATE;
		pix->height /= 2;
	} else {
		pix->field = V4L2_FIELD_NONE;
	}
	pix->colorspace = V4L2_COLORSPACE_REC709;

	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
}

static int hdmi2pcie_try_fmt_vid(struct file *file, void *private,
		struct v4l2_format *f)
{
	struct vid_channel *chan = video_drvdata(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	if (pix->pixelformat != V4L2_PIX_FMT_UYVY) {
		dev_info(&chan->common->dev->dev, "%s: wrong format\n", __PRETTY_FUNCTION__);
	}
	hdmi2pcie_fill_pix_format(chan, pix);
	return 0;
}

static int hdmi2pcie_s_fmt_vid(struct file *file, void *private,
		struct v4l2_format *f)
{
	struct vid_channel *chan = video_drvdata(file);
	int ret;

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	ret = hdmi2pcie_try_fmt_vid(file, private, f);
	if (ret)
		return ret;

	if (vb2_is_busy(&chan->queue))
		return -EBUSY;

	// TODO: set format on the device
	chan->format = f->fmt.pix;
	return 0;
}

static int hdmi2pcie_g_fmt_vid(struct file *file, void *private,
		struct v4l2_format *f)
{
	struct vid_channel *chan = video_drvdata(file);

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	f->fmt.pix = chan->format;
	return 0;
}

static int hdmi2pcie_enum_fmt_vid(struct file *file, void *private,
		struct v4l2_fmtdesc *f)
{
#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	if (f->index > 0)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_UYVY;
	return 0;
}

static int hdmi2pcie_s_dv_timings(struct file *file, void *_fh,
		struct v4l2_dv_timings *timings)
{
	struct vid_channel *chan = video_drvdata(file);

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	if (!v4l2_valid_dv_timings(timings, &hdmi2pcie_timings_cap, NULL, NULL))
		return -EINVAL;

	if (!v4l2_find_dv_timings_cap(timings, &hdmi2pcie_timings_cap,
				0, NULL, NULL))
		return -EINVAL;

	if (v4l2_match_dv_timings(timings, &chan->timings, 0, false))
		return 0;

	if (vb2_is_busy(&chan->queue))
		return -EBUSY;

	chan->timings = *timings;

	hdmi2pcie_fill_pix_format(chan, &chan->format);
	return 0;
}

static int hdmi2pcie_g_dv_timings(struct file *file, void *_fh,
		struct v4l2_dv_timings *timings)
{
	struct vid_channel *chan = video_drvdata(file);

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	*timings = chan->timings;
	return 0;
}

static int hdmi2pcie_enum_dv_timings(struct file *file, void *_fh,
		struct v4l2_enum_dv_timings *timings)
{
#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	return v4l2_enum_dv_timings_cap(timings, &hdmi2pcie_timings_cap,
			NULL, NULL);
}

static int hdmi2pcie_query_dv_timings(struct file *file, void *_fh,
		struct v4l2_dv_timings *timings)
{
	// TODO: detect current timings/signal state (out of range, disconnected, ...)

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	return 0;
}

static int hdmi2pcie_dv_timings_cap(struct file *file, void *fh,
		struct v4l2_dv_timings_cap *cap)
{
#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	*cap = hdmi2pcie_timings_cap;
	return 0;
}

static int hdmi2pcie_enum_input(struct file *file, void *private,
		struct v4l2_input *i)
{
#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	if (i->index > 0)
		return -EINVAL;

	i->type = V4L2_INPUT_TYPE_CAMERA;
	strlcpy(i->name, "HDMI In", sizeof(i->name));
	i->capabilities = V4L2_IN_CAP_DV_TIMINGS;
	return 0;
}

static int hdmi2pcie_s_input(struct file *file, void *private, unsigned int i)
{
	struct vid_channel *chan = video_drvdata(file);

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	if (i > 0)
		return -EINVAL;

	if (vb2_is_busy(&chan->queue))
		return -EBUSY;

	chan->vdev.tvnorms = 0;

	hdmi2pcie_fill_pix_format(chan, &chan->format);
	return 0;
}

static int hdmi2pcie_g_input(struct file *file, void *private, unsigned int *i)
{
#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	*i = 0;
	return 0;
}

static int hdmi2pcie_enum_output(struct file *file, void *private,
		struct v4l2_output *i)
{
#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	if (i->index > 0)
		return -EINVAL;

	i->type = V4L2_OUTPUT_TYPE_ANALOG;
	strlcpy(i->name, "HDMI Out", sizeof(i->name));
	i->capabilities = V4L2_OUT_CAP_DV_TIMINGS;
	return 0;
}

static int hdmi2pcie_s_output(struct file *file, void *private, unsigned int i)
{
	struct vid_channel *chan = video_drvdata(file);

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	if (i > 0)
		return -EINVAL;

	if (vb2_is_busy(&chan->queue))
		return -EBUSY;

	chan->vdev.tvnorms = 0;

	hdmi2pcie_fill_pix_format(chan, &chan->format);
	return 0;
}

static int hdmi2pcie_g_output(struct file *file, void *private, unsigned int *i)
{
#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	*i = 0;
	return 0;
}

static const struct v4l2_ioctl_ops hdmi2pcie_ioctl_in_ops = {
	.vidioc_querycap = hdmi2pcie_querycap,

	.vidioc_try_fmt_vid_cap = hdmi2pcie_try_fmt_vid,
	.vidioc_s_fmt_vid_cap = hdmi2pcie_s_fmt_vid,
	.vidioc_g_fmt_vid_cap = hdmi2pcie_g_fmt_vid,
	.vidioc_enum_fmt_vid_cap = hdmi2pcie_enum_fmt_vid,

	.vidioc_s_dv_timings = hdmi2pcie_s_dv_timings,
	.vidioc_g_dv_timings = hdmi2pcie_g_dv_timings,
	.vidioc_enum_dv_timings = hdmi2pcie_enum_dv_timings,
	.vidioc_query_dv_timings = hdmi2pcie_query_dv_timings,
	.vidioc_dv_timings_cap = hdmi2pcie_dv_timings_cap,

	.vidioc_enum_input = hdmi2pcie_enum_input,
	.vidioc_g_input = hdmi2pcie_g_input,
	.vidioc_s_input = hdmi2pcie_s_input,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,

	.vidioc_log_status = v4l2_ctrl_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_ioctl_ops hdmi2pcie_ioctl_out_ops = {
	.vidioc_querycap = hdmi2pcie_querycap,

	.vidioc_try_fmt_vid_out = hdmi2pcie_try_fmt_vid,
	.vidioc_s_fmt_vid_out = hdmi2pcie_s_fmt_vid,
	.vidioc_g_fmt_vid_out = hdmi2pcie_g_fmt_vid,
	.vidioc_enum_fmt_vid_out = hdmi2pcie_enum_fmt_vid,

	.vidioc_s_dv_timings = hdmi2pcie_s_dv_timings,
	.vidioc_g_dv_timings = hdmi2pcie_g_dv_timings,
	.vidioc_enum_dv_timings = hdmi2pcie_enum_dv_timings,
	.vidioc_query_dv_timings = hdmi2pcie_query_dv_timings,
	.vidioc_dv_timings_cap = hdmi2pcie_dv_timings_cap,

	.vidioc_enum_output = hdmi2pcie_enum_output,
	.vidioc_g_output = hdmi2pcie_g_output,
	.vidioc_s_output = hdmi2pcie_s_output,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,

	.vidioc_log_status = v4l2_ctrl_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations hdmi2pcie_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.read = vb2_fop_read,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

static int hdmi2pcie_register_video_dev(struct pci_dev *pdev, struct vid_channel *chan, uint8_t dir)
{
	static const struct v4l2_dv_timings timings_def = V4L2_DV_BT_CEA_1280X720P60;//V4L2_DV_BT_CEA_1920X1080P60;
	struct video_device *vdev = &chan->vdev;
	struct vb2_queue *q = &chan->queue;
	int ret;

#ifdef DEBUG_FNCTS
	printk(KERN_INFO LITEPCIE_NAME " %s\n", __func__);
#endif

	chan->dir = dir;

	ret = v4l2_device_register(&pdev->dev, &chan->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "v4l2_device_register failed, ret=%d\n", ret);
		return ret;
	}

	mutex_init(&chan->lock);

	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->dev = &pdev->dev;
	q->drv_priv = chan;
	q->buf_struct_size = sizeof(struct hdmi2pcie_buffer);
	q->ops = &hdmi2pcie_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed = 2;
	q->lock = &chan->lock;
	q->gfp_flags = GFP_DMA32;

	if (dir == HDMI2PCIE_DIR_IN) {
		q->io_modes |= VB2_READ;
		q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	} else {
		q->io_modes |= VB2_WRITE;
		q->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	}

	ret = vb2_queue_init(q);
	if (ret) {
		dev_err(&pdev->dev, "vb2_queue_init failed, ret=%d\n", ret);
		goto v4l2_unregister;
	}

	INIT_LIST_HEAD(&chan->buf_list);
	spin_lock_init(&chan->qlock);

	strlcpy(vdev->name, LITEPCIE_NAME, sizeof(vdev->name));
	vdev->release = video_device_release_empty;

	vdev->fops = &hdmi2pcie_fops;
	vdev->device_caps = V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;

	if (dir == HDMI2PCIE_DIR_IN) {
		vdev->ioctl_ops = &hdmi2pcie_ioctl_in_ops;
		vdev->device_caps |= V4L2_CAP_VIDEO_CAPTURE;
	} else {
		vdev->ioctl_ops = &hdmi2pcie_ioctl_out_ops;
		vdev->device_caps |= V4L2_CAP_VIDEO_OUTPUT;
	}


	vdev->lock = &chan->lock;
	vdev->queue = q;
	vdev->vfl_dir = dir ? VFL_DIR_TX : VFL_DIR_RX;
	vdev->v4l2_dev = &chan->v4l2_dev;
	video_set_drvdata(vdev, chan);

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret) {
		dev_err(&pdev->dev, "video_register_device failed, ret=%d\n", ret);
		goto v4l2_unregister;
	}

	chan->timings = timings_def;

	dev_info(&pdev->dev, "V4L2 HDMI2PCIe driver loaded");
	return 0;

v4l2_unregister:
	v4l2_device_unregister(&chan->v4l2_dev);
	return ret;
}


/* from stackoverflow */
void sfind( char *string, char *format, ... )
{
	va_list arglist;

	va_start( arglist, format );
	vsscanf( string, format, arglist );
	va_end( arglist );
}

struct revision
{
	int yy;
	int mm;
	int dd;
};

int compare_revisions(struct revision d1, struct revision d2)
{
	if (d1.yy < d2.yy)
		return -1;
	else if (d1.yy > d2.yy)
		return 1;
	else {
		if (d1.mm < d2.mm)
			return -1;
		else if (d1.mm > d2.mm)
			return 1;
		else if (d1.dd < d2.dd)
			return -1;
		else if(d1.dd > d2.dd)
			return 1;
		else
			return 0;
	}
}
/* from stackoverflow */

static int litepcie_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int ret=0;
	uint8_t rev_id;
	int i;
	char fpga_identifier[256];
	struct revision minimal_gateware_revision;
	struct revision current_gateware_revision;

	struct litepcie_device *litepcie_dev = NULL;

	printk(KERN_INFO LITEPCIE_NAME " \e[1m[lalaalProbing device]\e[0m\n");

	litepcie_dev = kzalloc(sizeof(struct litepcie_device), GFP_KERNEL);
	if(!litepcie_dev) {
		printk(KERN_ERR LITEPCIE_NAME " Cannot allocate memory\n");
		ret = -ENOMEM;
		goto fail1;
	}

	pci_set_drvdata(dev, litepcie_dev);
	litepcie_dev->dev = dev;
	spin_lock_init(&litepcie_dev->lock);
	list_add_tail(&(litepcie_dev->list), &(litepcie_list));

	ret = pci_enable_device(dev);
	if (ret != 0) {
		printk(KERN_ERR LITEPCIE_NAME " Cannot enable device\n");
		goto fail1;
	}

	/* check device version */
	pci_read_config_byte(dev, PCI_REVISION_ID, &rev_id);
	if (rev_id != 0) {
		printk(KERN_ERR LITEPCIE_NAME " Unsupported device version %d\n", rev_id);
		goto fail2;
	}

	if (pci_request_regions(dev, LITEPCIE_NAME) < 0) {
		printk(KERN_ERR LITEPCIE_NAME " Could not request regions\n");
		goto fail2;
	}

	/* check bar0 config */
	if (!(pci_resource_flags(dev, 0) & IORESOURCE_MEM)) {
		printk(KERN_ERR LITEPCIE_NAME " Invalid BAR0 configuration\n");
		goto fail3;
	}

	litepcie_dev->bar0_addr = pci_ioremap_bar(dev, 0);
	litepcie_dev->bar0_size = pci_resource_len(dev, 0);
	litepcie_dev->bar0_phys_addr = pci_resource_start(dev, 0);
	if (!litepcie_dev->bar0_addr) {
		printk(KERN_ERR LITEPCIE_NAME " Could not map BAR0\n");
		goto fail3;
	}

	/* show identifier */
	for(i=0; i < 256; i++)
		fpga_identifier[i] = litepcie_readl(litepcie_dev, CSR_IDENTIFIER_MEM_BASE + i*4);
	printk(KERN_INFO LITEPCIE_NAME " Version %s\n", fpga_identifier);

	pci_set_master(dev);
	ret = pci_set_dma_mask(dev, DMA_BIT_MASK(32));
	if (ret) {
		printk(KERN_ERR LITEPCIE_NAME " Failed to set DMA mask\n");
		goto fail4;
	};

	ret = pci_enable_msi(dev);
	if (ret) {
		printk(KERN_ERR LITEPCIE_NAME " Failed to enable MSI\n");
		goto fail4;
	}

	if (request_irq(dev->irq, litepcie_interrupt, IRQF_SHARED, LITEPCIE_NAME, litepcie_dev) < 0) {
		printk(KERN_ERR LITEPCIE_NAME " Failed to allocate IRQ %d\n", dev->irq);
		goto fail5;
	}

	/* soft reset */
	litepcie_writel(litepcie_dev, CSR_CRG_RESET_ADDR, 1);
	udelay(1000);

	litepcie_dev->channels_count = DMA_CHANNELS;

	litepcie_dev->channels = kzalloc(sizeof(*litepcie_dev->channels) * litepcie_dev->channels_count, GFP_KERNEL);
	if(!litepcie_dev->channels) {
		dev_err(&dev->dev, "Cannot allocate memory\n");
		ret = -ENOMEM;
		goto fail1;
	}

	for (i = 0; i < litepcie_dev->channels_count; i++) {
		litepcie_dev->channels[i].common = litepcie_dev;
		litepcie_dev->channels[i].streaming = 0;
	}

	for(i = 0; i < litepcie_dev->channels_count; i++) {
		litepcie_dev->chan[i].index = i;
		litepcie_dev->chan[i].block_size = DMA_BUFFER_SIZE;
		litepcie_dev->chan[i].minor = litepcie_dev->minor_base + i;
		litepcie_dev->chan[i].litepcie_dev = litepcie_dev;
		litepcie_dev->chan[i].dma.writer_lock = 0;
		litepcie_dev->chan[i].dma.reader_lock = 0;
		init_waitqueue_head(&litepcie_dev->chan[i].wait_rd);
		init_waitqueue_head(&litepcie_dev->chan[i].wait_wr);
		switch(i) {
			case 1: {
					litepcie_dev->chan[i].dma.base = CSR_PCIE_DMA1_BASE;
					litepcie_dev->chan[i].dma.writer_interrupt = PCIE_DMA1_WRITER_INTERRUPT;
					litepcie_dev->chan[i].dma.reader_interrupt = PCIE_DMA1_READER_INTERRUPT;
				}
				break;
			default:
				{
					litepcie_dev->chan[i].dma.base = CSR_PCIE_DMA0_BASE;
					litepcie_dev->chan[i].dma.writer_interrupt = PCIE_DMA0_WRITER_INTERRUPT;
					litepcie_dev->chan[i].dma.reader_interrupt = PCIE_DMA0_READER_INTERRUPT;
				}
				break;
		}
	}

    /* reset hdmi rx0 mmcm */
    litepcie_writel(litepcie_dev, CSR_HDMI_IN0_CLOCKING_MMCM_RESET_ADDR, 1);
    litepcie_writel(litepcie_dev, CSR_HDMI_IN0_CLOCKING_MMCM_RESET_ADDR, 0);

	litepcie_hdmi_rx_mmcm_init(litepcie_dev);

	/* enable hdmi rx0 dma */
	litepcie_writel(litepcie_dev, CSR_HDMI_IN0_DMA_FRAME_SIZE_ADDR, 1280 * 720 * 2);
	litepcie_writel(litepcie_dev, CSR_HDMI_IN0_DMA_SLOT0_ADDRESS_ADDR, 0x03000000);
	litepcie_writel(litepcie_dev, CSR_HDMI_IN0_DMA_SLOT1_ADDRESS_ADDR, 0x04000000);
	litepcie_writel(litepcie_dev, CSR_HDMI_IN0_DMA_SLOT0_STATUS_ADDR, SLOT_LOADED);
	litepcie_writel(litepcie_dev, CSR_HDMI_IN0_DMA_SLOT1_STATUS_ADDR, SLOT_LOADED);
	litepcie_writel(litepcie_dev, CSR_HDMI_IN0_DMA_EV_PENDING_ADDR,
        litepcie_readl(litepcie_dev, CSR_HDMI_IN0_DMA_EV_PENDING_ADDR));
	litepcie_writel(litepcie_dev, CSR_HDMI_IN0_DMA_EV_ENABLE_ADDR, 0x3);

	litepcie_enable_interrupt(litepcie_dev, HDMI_IN0_DMA_INTERRUPT);

	/* check minimal gateware revision */
	sfind(MINIMAL_GATEWARE_REVISION, "%d-%d-%d",
			&minimal_gateware_revision.yy,
			&minimal_gateware_revision.mm,
			&minimal_gateware_revision.dd);
	for(i=0; i < 256; i++) {
		if (fpga_identifier[i] == '-') {
			i -= 5;
			break;
		}
	}
	sfind(fpga_identifier + i, "%d-%d-%d",
			&current_gateware_revision.yy,
			&current_gateware_revision.mm,
			&current_gateware_revision.dd);

	if (compare_revisions(minimal_gateware_revision, current_gateware_revision) > 0) {
		printk(KERN_INFO LITEPCIE_NAME " \e[1mHW needs update to gateware revision %04d-%02d-%02d\e[0m\n",
				minimal_gateware_revision.yy,
				minimal_gateware_revision.mm,
				minimal_gateware_revision.dd);
	}

	ret = hdmi2pcie_register_video_dev(dev, &litepcie_dev->channels[0], HDMI2PCIE_DIR_IN);
	ret += hdmi2pcie_register_video_dev(dev, &litepcie_dev->channels[1], HDMI2PCIE_DIR_OUT);
	if (ret) {
		dev_err(&dev->dev, "Failed to register V4L2 device");
		goto fail5;
	}

	return 0;

fail5:
	pci_disable_msi(dev);
fail4:
	pci_iounmap(dev, litepcie_dev->bar0_addr);
fail3:
	pci_release_regions(dev);
fail2:
	pci_disable_device(dev);
fail1:
	if(litepcie_dev){
		list_del(&litepcie_dev->list);
		kfree(litepcie_dev);
	}
	return ret;
}

static void litepcie_pci_remove(struct pci_dev *dev)
{
	struct litepcie_device *litepcie_dev;
	int i;

	litepcie_dev = pci_get_drvdata(dev);

	printk(KERN_INFO LITEPCIE_NAME " \e[1m[Removing device]\e[0m\n");

	if(litepcie_dev){
		litepcie_stop_dma(litepcie_dev);
		free_irq(dev->irq, litepcie_dev);
	}

	/* disable all interrupts */
	litepcie_writel(litepcie_dev, CSR_PCIE_MSI_ENABLE_ADDR, 0);

	pci_disable_msi(dev);
	if(litepcie_dev)
		pci_iounmap(dev, litepcie_dev->bar0_addr);
	pci_disable_device(dev);
	pci_release_regions(dev);
	for (i=0; i < litepcie_dev->channels_count; i++) {
		video_unregister_device(&litepcie_dev->channels[i].vdev);
		v4l2_device_unregister(&litepcie_dev->channels[i].v4l2_dev);
	}
	if(litepcie_dev){
		kfree(litepcie_dev);
	}
}

static const struct pci_device_id litepcie_pci_ids[] = {
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID ), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, litepcie_pci_ids);

static struct pci_driver litepcie_pci_driver = {
	.name = LITEPCIE_NAME,
	.id_table = litepcie_pci_ids,
	.probe = litepcie_pci_probe,
	.remove = litepcie_pci_remove,
};


static int __init litepcie_module_init(void)
{
	int ret;

	litepcie_class = class_create(THIS_MODULE, LITEPCIE_NAME);
	if(!litepcie_class) {
		ret = -EEXIST;
		printk(KERN_ERR LITEPCIE_NAME " Failed to create class\n");
		goto fail_create_class;
	}

	ret = alloc_chrdev_region(&litepcie_dev_t, 0, LITEPCIE_MINOR_COUNT, LITEPCIE_NAME);
	if(ret < 0) {
		printk(KERN_ERR LITEPCIE_NAME " Could not allocate char device\n");
		goto fail_alloc_chrdev_region;
	}
	litepcie_major = MAJOR(litepcie_dev_t);
	litepcie_minor_idx = MINOR(litepcie_dev_t);

	ret = pci_register_driver(&litepcie_pci_driver);
	if (ret < 0) {
		printk(KERN_ERR LITEPCIE_NAME LITEPCIE_NAME " Error while registering PCI driver\n");
		goto fail_register;
	}

	return 0;

fail_register:
	unregister_chrdev_region(litepcie_dev_t, LITEPCIE_MINOR_COUNT);
fail_alloc_chrdev_region:
	class_destroy(litepcie_class);
fail_create_class:
	return ret;
}

static void __exit litepcie_module_exit(void)
{
	pci_unregister_driver(&litepcie_pci_driver);
	unregister_chrdev_region(litepcie_dev_t, LITEPCIE_MINOR_COUNT);
	class_destroy(litepcie_class);
}


module_init(litepcie_module_init);
module_exit(litepcie_module_exit);

MODULE_LICENSE("GPL");
