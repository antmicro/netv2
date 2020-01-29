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

//#define DEBUG_CSR
//#define DEBUG_MSI
//#define DEBUG_POLL
//#define DEBUG_READ
//#define DEBUG_WRITE

#define LITEPCIE_NAME "litepcie"
#define LITEPCIE_MINOR_COUNT 32

#define HDMI2PCIE_DIR_IN 0
#define HDMI2PCIE_DIR_OUT 1

#define PCIE_BUF_SIZE 0x400000
#define READ_BUF_OFF (7*PCIE_BUF_SIZE)

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

struct litepcie_device {
    struct pci_dev  *dev;
    resource_size_t bar0_size;
    phys_addr_t bar0_phys_addr;
    uint8_t *bar0_addr; /* virtual address of BAR0 */
    struct litepcie_chan chan[DMA_CHANNEL_COUNT];
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
    v = litepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
    v |= (1 << irq_num);
    litepcie_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

static void litepcie_disable_interrupt(struct litepcie_device *s, int irq_num)
{
    uint32_t v;
    v = litepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
    v &= ~(1 << irq_num);
    litepcie_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

static int litepcie_dma_free(struct litepcie_device *s)
{
    int i, j;
    struct litepcie_dma_chan *dmachan;

    if(!s)
        return -ENODEV;

    /* for each dma channel */
    for(i = 0; i < s->channels_count; i++) {
        dmachan = &s->chan[i].dma;
        /* for each dma buffer */
        for(j = 0; j < DMA_BUFFER_COUNT; j++) {
            /* free rd */
            if(dmachan->reader_addr[j]) {
                dma_free_coherent(&s->dev->dev, DMA_BUFFER_SIZE,
                                  dmachan->reader_addr[j],
                                  dmachan->reader_handle[j]);
                dmachan->reader_addr[j] = NULL;
                dmachan->reader_handle[j] = 0;
            }
            /* free wr */
            if(dmachan->writer_addr[j]) {
                dma_free_coherent(&s->dev->dev, DMA_BUFFER_SIZE,
                                  dmachan->writer_addr[j],
                                  dmachan->writer_handle[j]);
                dmachan->writer_addr[j] = NULL;
                dmachan->writer_handle[j] = 0;
            }
        }
    }

    return 0;
}

static int litepcie_dma_init(struct litepcie_device *s)
{

    int i, j;
    struct litepcie_dma_chan *dmachan;
    int ret;

    if(!s)
        return -ENODEV;

    /* for each dma channel */
    for(i = 0; i < s->channels_count; i++) {
        dmachan = &s->chan[i].dma;
        /* for each dma buffer */
        for(j = 0; j < DMA_BUFFER_COUNT; j++) {
            /* allocate rd */
            dmachan->reader_addr[j] = dma_alloc_coherent(
                &s->dev->dev,
                DMA_BUFFER_SIZE,
                &dmachan->reader_handle[j],
                GFP_KERNEL | GFP_DMA32);
            /* allocate wr */
            dmachan->writer_addr[j] = dma_alloc_coherent(
                &s->dev->dev,
                DMA_BUFFER_SIZE,
                &dmachan->writer_handle[j],
                GFP_KERNEL | GFP_DMA32);
            /* check */
            if(!dmachan->writer_addr[j]
               || !dmachan->reader_addr[j]) {
                printk(KERN_ERR LITEPCIE_NAME " Failed to allocate dma buffers\n");
                ret = -ENOMEM;
                goto fail;
            }
        }
    }

    return 0;
fail:
    litepcie_dma_free(s);
    return ret;
}

static void litepcie_dma_writer_start(struct litepcie_device *s, int chan_num)
{
    struct litepcie_dma_chan *dmachan;
    int i;

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
                    dmachan->writer_handle[i]);
        litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_WE_OFFSET, 1);
    }
    litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_OFFSET, 1);

    /* clear counters */
    dmachan->writer_hw_count = 0;
    dmachan->writer_hw_count_last = 0;

    /* start dma writer */
    litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 1);
}

static void litepcie_dma_writer_stop(struct litepcie_device *s, int chan_num)
{
    struct litepcie_dma_chan *dmachan;

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

static void litepcie_dma_reader_start(struct litepcie_device *s, int chan_num)
{
    struct litepcie_dma_chan *dmachan;
    int i;

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
            dmachan->reader_handle[i]);
        litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_WE_OFFSET, 1);
    }
    litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_LOOP_PROG_N_OFFSET, 1);

    /* clear counters */
    dmachan->reader_hw_count = 0;
    dmachan->reader_hw_count_last = 0;

    /* start dma reader */
    litepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 1);
}

static void litepcie_dma_reader_stop(struct litepcie_device *s, int chan_num)
{
    struct litepcie_dma_chan *dmachan;

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
    uint32_t loop_status;
    uint32_t clear_mask, irq_vector, irq_enable;
    int i;

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
            loop_status = litepcie_readl(s, chan->dma.base +
                PCIE_DMA_READER_TABLE_LOOP_STATUS_OFFSET);
            chan->dma.reader_hw_count &= ((~(DMA_BUFFER_COUNT - 1) << 16) & 0xffffffffffff0000);
            chan->dma.reader_hw_count |= (loop_status >> 16) * DMA_BUFFER_COUNT + (loop_status & 0xffff);
            if (chan->dma.reader_hw_count_last > chan->dma.reader_hw_count)
                chan->dma.reader_hw_count += (1 << (ilog2(DMA_BUFFER_COUNT) + 16));
            chan->dma.reader_hw_count_last = chan->dma.reader_hw_count;
#ifdef DEBUG_MSI
            printk(KERN_INFO LITEPCIE_NAME " MSI DMA%d Reader buf: %lld\n", i, chan->dma.reader_hw_count);
#endif
            wake_up_interruptible(&chan->wait_wr);
            clear_mask |= (1 << chan->dma.reader_interrupt);
        }
        /* dma writer interrupt handling */
        if(irq_vector & (1 << chan->dma.writer_interrupt)) {
            loop_status = litepcie_readl(s, chan->dma.base +
                PCIE_DMA_WRITER_TABLE_LOOP_STATUS_OFFSET);
            chan->dma.writer_hw_count &= ((~(DMA_BUFFER_COUNT - 1) << 16) & 0xffffffffffff0000);
            chan->dma.writer_hw_count |= (loop_status >> 16) * DMA_BUFFER_COUNT + (loop_status & 0xffff);
            if (chan->dma.writer_hw_count_last > chan->dma.writer_hw_count)
                chan->dma.writer_hw_count += (1 << (ilog2(DMA_BUFFER_COUNT) + 16));
            chan->dma.writer_hw_count_last = chan->dma.writer_hw_count;
#ifdef DEBUG_MSI
            printk(KERN_INFO LITEPCIE_NAME " MSI DMA%d Writer buf: %lld\n", i, chan->dma.writer_hw_count);
#endif
            wake_up_interruptible(&chan->wait_rd);
            clear_mask |= (1 << chan->dma.writer_interrupt);
        }
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

	if (vq->num_buffers + *nbuffers < 3)
		*nbuffers = 3 - vq->num_buffers;

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
	//struct hdmi2pcie_buffer *hbuf = to_hdmi2pcie_buffer(vb);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	unsigned long size = vb2_get_plane_payload(vb, 0);
	void *buf = vb2_plane_vaddr(vb, 0);
	unsigned long flags;
	unsigned long fb_idx = (readl(chan->fb_idx) + 1) % 4;

	spin_lock_irqsave(&chan->qlock, flags);
	//list_add_tail(&hbuf->list, &priv->buf_list);
	if (chan->dir == HDMI2PCIE_DIR_IN)
		memcpy_fromio(buf, chan->common->bar0_addr + READ_BUF_OFF, size);
	else
		memcpy_toio(chan->common->bar0_addr + fb_idx * PCIE_BUF_SIZE, buf, size);

	writel(fb_idx, chan->fb_idx);

	vb->timestamp = ktime_get_ns();
	vbuf->sequence = chan->sequence++;
	vbuf->field = V4L2_FIELD_NONE;

	vb2_buffer_done(vb, VB2_BUF_STATE_DONE);

	spin_unlock_irqrestore(&chan->qlock, flags);
}

static void return_all_buffers(struct vid_channel *chan,
			       enum vb2_buffer_state state)
{
	struct hdmi2pcie_buffer *buf, *node;
	unsigned long flags;

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

	chan->sequence = 0;

	//TODO: Start DMA

	if (ret) {
		return_all_buffers(chan, VB2_BUF_STATE_QUEUED);
	}
	return ret;
}

static void stop_streaming(struct vb2_queue *vq)
{
	struct vid_channel *chan = vb2_get_drv_priv(vq);

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

	strlcpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	strlcpy(cap->card, "HDMI2PCIe", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI:%s",
		 pci_name(chan->common->dev));
	return 0;
}

static void hdmi2pcie_fill_pix_format(struct vid_channel *chan,
				     struct v4l2_pix_format *pix)
{
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

	f->fmt.pix = chan->format;
	return 0;
}

static int hdmi2pcie_enum_fmt_vid(struct file *file, void *private,
				     struct v4l2_fmtdesc *f)
{
	if (f->index > 0)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_UYVY;
	return 0;
}

static int hdmi2pcie_s_dv_timings(struct file *file, void *_fh,
				 struct v4l2_dv_timings *timings)
{
	struct vid_channel *chan = video_drvdata(file);

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

	*timings = chan->timings;
	return 0;
}

static int hdmi2pcie_enum_dv_timings(struct file *file, void *_fh,
				    struct v4l2_enum_dv_timings *timings)
{
	return v4l2_enum_dv_timings_cap(timings, &hdmi2pcie_timings_cap,
					NULL, NULL);
}

static int hdmi2pcie_query_dv_timings(struct file *file, void *_fh,
				     struct v4l2_dv_timings *timings)
{
	// TODO: detect current timings/signal state (out of range, disconnected, ...)

	return 0;
}

static int hdmi2pcie_dv_timings_cap(struct file *file, void *fh,
				   struct v4l2_dv_timings_cap *cap)
{
	*cap = hdmi2pcie_timings_cap;
	return 0;
}

static int hdmi2pcie_enum_input(struct file *file, void *private,
			       struct v4l2_input *i)
{
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
	*i = 0;
	return 0;
}

static int hdmi2pcie_enum_output(struct file *file, void *private,
			       struct v4l2_output *i)
{
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
	static const struct v4l2_dv_timings timings_def = V4L2_DV_BT_CEA_1920X1080P60;
	struct video_device *vdev = &chan->vdev;
	struct vb2_queue *q = &chan->queue;
	int ret;

	chan->dir = dir;

	ret = v4l2_device_register(&pdev->dev, &chan->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "v4l2_device_register failed, ret=%d\n", ret);
		return ret;
	}

	// Point to last 4 bytes of a 16MiB area
	if (dir == HDMI2PCIE_DIR_IN)
		chan->fb_idx = (unsigned int*)(chan->common->bar0_addr + 8*PCIE_BUF_SIZE - sizeof(unsigned int));
	else
		chan->fb_idx = (unsigned int*)(chan->common->bar0_addr + 4*PCIE_BUF_SIZE - sizeof(unsigned int));

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
/* cdev fcns */
static int litepcie_open(struct inode *inode, struct file *file)
{
    int subminor;
    struct litepcie_device *litepcie;
    struct litepcie_chan *chan;

    struct litepcie_chan_priv *chan_priv = kzalloc(sizeof(*chan_priv),
            GFP_KERNEL);
    if (!chan_priv)
        return -ENOMEM;

    subminor = iminor(inode);

    list_for_each_entry(litepcie, &litepcie_list, list) {
        if( (litepcie->minor_base <= subminor) &&
            (subminor < litepcie->minor_base + litepcie->channels_count) ) {

            chan = &litepcie->chan[subminor - litepcie->minor_base];
            chan_priv->chan = chan;
            file->private_data = chan_priv;

            if (chan->dma.reader_enable == 0) { /* clear only if disabled */
                chan->dma.reader_hw_count = 0;
                chan->dma.reader_hw_count_last = 0;
                chan->dma.reader_sw_count = 0;
            }

            if (chan->dma.writer_enable == 0) { /* clear only if disabled */
                chan->dma.writer_hw_count = 0;
                chan->dma.writer_hw_count_last = 0;
                chan->dma.writer_sw_count = 0;
            }

            return 0;
        }
    }

    kfree(chan_priv);
    return -1;
}

static int litepcie_release(struct inode *inode, struct file *file)
{
    struct litepcie_chan_priv *chan_priv = file->private_data;
    struct litepcie_chan *chan = chan_priv->chan;

    if (chan_priv->reader) {
        /* disable interrupt */
        litepcie_disable_interrupt(chan->litepcie_dev, chan->dma.reader_interrupt);
        /* disable DMA */
        litepcie_dma_reader_stop(chan->litepcie_dev, chan->index);
        chan->dma.reader_lock = 0;
        chan->dma.reader_enable = 0;
    }

    if (chan_priv->writer) {
        /* disable interrupt */
        litepcie_disable_interrupt(chan->litepcie_dev, chan->dma.writer_interrupt);
        /* disable DMA */
        litepcie_dma_writer_stop(chan->litepcie_dev, chan->index);
        chan->dma.writer_lock = 0;
        chan->dma.writer_enable = 0;
    }

    kfree(chan_priv);

    return 0;
}

static ssize_t litepcie_read(struct file *file, char __user *data, size_t size, loff_t *offset)
{
    size_t len;
    int i, ret;
    int overflows;

    struct litepcie_chan_priv *chan_priv = file->private_data;
    struct litepcie_chan *chan = chan_priv->chan;

    if (file->f_flags & O_NONBLOCK) {
        if (chan->dma.writer_hw_count == chan->dma.writer_sw_count)
            ret = -EAGAIN;
        else
            ret = 0;
    } else {
        ret = wait_event_interruptible(chan->wait_rd,
                (chan->dma.writer_hw_count - chan->dma.writer_sw_count) > 0);
    }

    if(ret < 0)
        return ret;

    i = 0;
    overflows = 0;
    len = size;
    while (len >= DMA_BUFFER_SIZE) {
        if ((chan->dma.writer_hw_count - chan->dma.writer_sw_count) > 0) {
            if ((chan->dma.writer_hw_count - chan->dma.writer_sw_count) > DMA_BUFFER_COUNT/2) {
                overflows++;
            } else {
                ret = copy_to_user(data + (chan->block_size * i),
                           chan->dma.writer_addr[chan->dma.writer_sw_count%DMA_BUFFER_COUNT],
                           DMA_BUFFER_SIZE);
                if(ret)
                    return -EFAULT;
            }
            len -= DMA_BUFFER_SIZE;
            chan->dma.writer_sw_count += 1;
            i++;
        } else {
            break;
        }
    }

    if (overflows)
        printk(KERN_INFO LITEPCIE_NAME " Reading too late, %d buffers lost\n", overflows);

#ifdef DEBUG_READ
    printk(KERN_INFO LITEPCIE_NAME " read: read %ld bytes out of %ld\n", size - len, size);
#endif

    return size - len;
}

static ssize_t litepcie_write(struct file *file, const char __user *data, size_t size, loff_t *offset)
{
    size_t len;
    int i, ret;
    int underflows;

    struct litepcie_chan_priv *chan_priv = file->private_data;
    struct litepcie_chan *chan = chan_priv->chan;

    if (file->f_flags & O_NONBLOCK) {
        if (chan->dma.reader_hw_count == chan->dma.reader_sw_count)
            ret = -EAGAIN;
        else
            ret = 0;
    } else {
        ret = wait_event_interruptible(chan->wait_wr,
                (chan->dma.reader_sw_count - chan->dma.reader_hw_count) < DMA_BUFFER_COUNT/2);
    }

    i = 0;
    underflows = 0;
    len = size;
    while (len >= DMA_BUFFER_SIZE) {
        if ((chan->dma.reader_sw_count - chan->dma.reader_hw_count) < DMA_BUFFER_COUNT/2) {
            if ((chan->dma.reader_sw_count - chan->dma.reader_hw_count) < 0) {
                underflows++;
            } else {
                ret = copy_from_user(chan->dma.reader_addr[chan->dma.reader_sw_count%DMA_BUFFER_COUNT],
                                     data + (chan->block_size * i), DMA_BUFFER_SIZE);
                if(ret)
                    return -EFAULT;
            }
            len -= DMA_BUFFER_SIZE;
            chan->dma.reader_sw_count += 1;
            i++;
        } else {
            break;
        }
    }

    if (underflows)
        printk(KERN_INFO LITEPCIE_NAME " Writing too late, %d buffers lost\n", underflows);

#ifdef DEBUG_WRITE
    printk(KERN_INFO LITEPCIE_NAME " write: write %ld bytes out of %ld\n", size - len, size);
#endif

    return size - len;
}

static int litepcie_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct litepcie_chan_priv *chan_priv = file->private_data;
    struct litepcie_chan *chan = chan_priv->chan;
    unsigned long pfn;
    int is_tx, i;

    if (vma->vm_end - vma->vm_start != DMA_BUFFER_TOTAL_SIZE)
        return -EINVAL;

    if (vma->vm_pgoff == 0) {
        is_tx = 1;
    } else if (vma->vm_pgoff == (DMA_BUFFER_TOTAL_SIZE >> PAGE_SHIFT)) {
        is_tx = 0;
    } else {
        return -EINVAL;
    }

    for (i=0; i < DMA_BUFFER_COUNT; i++) {
        if (is_tx) {
            pfn = __pa(chan->dma.reader_addr[i]) >> PAGE_SHIFT;
        } else {
            pfn = __pa(chan->dma.writer_addr[i]) >> PAGE_SHIFT;
        }
        /* Note: the memory is cached, so the user must explicitly
           flush the CPU caches on architectures which require it. */
        if (remap_pfn_range(vma, vma->vm_start + i * DMA_BUFFER_SIZE, pfn,
            DMA_BUFFER_SIZE, vma->vm_page_prot)) {
            printk(KERN_ERR LITEPCIE_NAME " mmap remap_pfn_range failed\n");
            return -EAGAIN;
        }
    }

    return 0;
}

static unsigned int litepcie_poll(struct file *file, poll_table *wait)
{
    unsigned int mask = 0;

    struct litepcie_chan_priv *chan_priv = file->private_data;
    struct litepcie_chan *chan = chan_priv->chan;
    poll_wait(file, &chan->wait_rd, wait);
    poll_wait(file, &chan->wait_wr, wait);

#ifdef DEBUG_POLL
    printk(KERN_INFO LITEPCIE_NAME " poll: writer hw_count: %10lld / sw_count %10lld \n",
        chan->dma.writer_hw_count, chan->dma.writer_sw_count);
    printk(KERN_INFO LITEPCIE_NAME " poll: reader hw_count: %10lld / sw_count %10lld \n",
        chan->dma.reader_hw_count, chan->dma.reader_sw_count);
#endif

    if((chan->dma.writer_hw_count - chan->dma.writer_sw_count) > 2)
        mask |= POLLIN | POLLRDNORM;

    if ((chan->dma.reader_sw_count - chan->dma.reader_hw_count) < DMA_BUFFER_COUNT/2)
        mask |= POLLOUT | POLLWRNORM;

    return mask;
}

/* SPI */

#define SPI_TIMEOUT 100000 /* in us */

static int litepcie_flash_spi(struct litepcie_device *s, struct litepcie_ioctl_flash *m)
{
    int i;

    if (m->tx_len < 8 || m->tx_len > 40)
        return -EINVAL;

    litepcie_writel(s, CSR_FLASH_SPI_MOSI_ADDR, m->tx_data >> 32);
    litepcie_writel(s, CSR_FLASH_SPI_MOSI_ADDR + 4, m->tx_data);
    litepcie_writel(s, CSR_FLASH_SPI_CONTROL_ADDR,
               SPI_CTRL_START | (m->tx_len * SPI_CTRL_LENGTH));
    udelay(16);
    for(i = 0; i < SPI_TIMEOUT; i++) {
        if (litepcie_readl(s, CSR_FLASH_SPI_STATUS_ADDR) & SPI_STATUS_DONE)
            break;
        udelay(1);
    }
    m->rx_data = ((uint64_t)litepcie_readl(s, CSR_FLASH_SPI_MISO_ADDR) << 32) |
        litepcie_readl(s, CSR_FLASH_SPI_MISO_ADDR + 4);
    return 0;
}

static long litepcie_ioctl(struct file *file, unsigned int cmd,
                      unsigned long arg)
{
    long ret = 0;

    struct litepcie_chan_priv *chan_priv = file->private_data;
    struct litepcie_chan *chan = chan_priv->chan;

    switch(cmd) {
    case LITEPCIE_IOCTL_REG:
        {
            struct litepcie_ioctl_reg m;

            if (copy_from_user(&m, (void *)arg, sizeof(m))) {
                ret = -EFAULT;
                break;
            }
            if (m.is_write)
                litepcie_writel(chan->litepcie_dev, m.addr, m.val);
            else
                m.val = litepcie_readl(chan->litepcie_dev, m.addr);

            if (copy_to_user((void *)arg, &m, sizeof(m))) {
                ret = -EFAULT;
                break;
            }
        }
        break;
    case LITEPCIE_IOCTL_FLASH:
        {
            struct litepcie_ioctl_flash m;

            if (copy_from_user(&m, (void *)arg, sizeof(m))) {
                ret = -EFAULT;
                break;
            }
            ret = litepcie_flash_spi(chan->litepcie_dev, &m);
            if (ret == 0) {
                if (copy_to_user((void *)arg, &m, sizeof(m))) {
                    ret = -EFAULT;
                    break;
                }
            }
        }
        break;
    case LITEPCIE_IOCTL_ICAP:
        {
            struct litepcie_ioctl_icap m;

            if (copy_from_user(&m, (void *)arg, sizeof(m))) {
                ret = -EFAULT;
                break;
            }

            litepcie_writel(chan->litepcie_dev, CSR_ICAP_ADDR_ADDR, m.addr);
			litepcie_writel(chan->litepcie_dev, CSR_ICAP_DATA_ADDR, m.data);
			litepcie_writel(chan->litepcie_dev, CSR_ICAP_SEND_ADDR, 1);
        }
        break;
    case LITEPCIE_IOCTL_DMA:
        {
            struct litepcie_ioctl_dma m;

            if (copy_from_user(&m, (void *)arg, sizeof(m))) {
                ret = -EFAULT;
                break;
            }

            /* loopback */
            litepcie_writel(chan->litepcie_dev, chan->dma.base + PCIE_DMA_LOOPBACK_ENABLE_OFFSET, m.loopback_enable);
        }
        break;
    case LITEPCIE_IOCTL_DMA_WRITER:
        {
            struct litepcie_ioctl_dma_writer m;

            if (copy_from_user(&m, (void *)arg, sizeof(m))) {
                ret = -EFAULT;
                break;
            }

            if (m.enable != chan->dma.writer_enable) {
                /* enable / disable DMA */
                if (m.enable) {
                    litepcie_dma_writer_start(chan->litepcie_dev, chan->index);
                    litepcie_enable_interrupt(chan->litepcie_dev, chan->dma.writer_interrupt);
                } else {
                    litepcie_disable_interrupt(chan->litepcie_dev, chan->dma.writer_interrupt);
                    litepcie_dma_writer_stop(chan->litepcie_dev, chan->index);
                }

            }

            chan->dma.writer_enable = m.enable;

            m.hw_count = chan->dma.writer_hw_count;
            m.sw_count = chan->dma.writer_sw_count;

            if (copy_to_user((void *)arg, &m, sizeof(m))) {
                ret = -EFAULT;
                break;
            }

        }
        break;
    case LITEPCIE_IOCTL_DMA_READER:
        {
            struct litepcie_ioctl_dma_reader m;

            if (copy_from_user(&m, (void *)arg, sizeof(m))) {
                ret = -EFAULT;
                break;
            }

            if (m.enable != chan->dma.reader_enable) {
                /* enable / disable DMA */
                if (m.enable) {
                    litepcie_dma_reader_start(chan->litepcie_dev, chan->index);
                    litepcie_enable_interrupt(chan->litepcie_dev, chan->dma.reader_interrupt);
                } else {
                    litepcie_disable_interrupt(chan->litepcie_dev, chan->dma.reader_interrupt);
                    litepcie_dma_reader_stop(chan->litepcie_dev, chan->index);
                }
            }

            chan->dma.reader_enable = m.enable;

            m.hw_count = chan->dma.reader_hw_count;
            m.sw_count = chan->dma.reader_sw_count;

            if (copy_to_user((void *)arg, &m, sizeof(m))) {
                ret = -EFAULT;
                break;
            }

        }
        break;
    case LITEPCIE_IOCTL_MMAP_DMA_INFO:
        {
            struct litepcie_ioctl_mmap_dma_info m;

            m.dma_tx_buf_offset = 0;
            m.dma_tx_buf_size = DMA_BUFFER_SIZE;
            m.dma_tx_buf_count = DMA_BUFFER_COUNT;

            m.dma_rx_buf_offset = DMA_BUFFER_TOTAL_SIZE;
            m.dma_rx_buf_size = DMA_BUFFER_SIZE;
            m.dma_rx_buf_count = DMA_BUFFER_COUNT;

            if (copy_to_user((void *)arg, &m, sizeof(m))) {
                ret = -EFAULT;
                break;
            }
        }
        break;
    case LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE:
        {
            struct litepcie_ioctl_mmap_dma_update m;

            if (copy_from_user(&m, (void *)arg, sizeof(m))) {
                ret = -EFAULT;
                break;
            }

            chan->dma.writer_sw_count = m.sw_count;
        }
        break;
    case LITEPCIE_IOCTL_MMAP_DMA_READER_UPDATE:
        {
            struct litepcie_ioctl_mmap_dma_update m;

            if (copy_from_user(&m, (void *)arg, sizeof(m))) {
                ret = -EFAULT;
                break;
            }

            chan->dma.reader_sw_count = m.sw_count;
        }
        break;
    case LITEPCIE_IOCTL_LOCK:
        {
            struct litepcie_ioctl_lock m;


            if (copy_from_user(&m, (void *)arg, sizeof(m))) {
                ret = -EFAULT;
                break;
            }

            m.dma_reader_status = 1;
            if (m.dma_reader_request) {
                if (chan->dma.reader_lock) {
                    m.dma_reader_status = 0;
                } else {
                    chan->dma.reader_lock = 1;
                    chan_priv->reader = 1;
                }
            }
            if (m.dma_reader_release) {
                chan->dma.reader_lock = 0;
                chan_priv->reader = 0;
            }

            m.dma_writer_status = 1;
            if (m.dma_writer_request) {
                if (chan->dma.writer_lock) {
                    m.dma_writer_status = 0;
                } else {
                    chan->dma.writer_lock = 1;
                    chan_priv->writer = 1;
                }
            }
            if (m.dma_writer_release) {
                chan->dma.writer_lock = 0;
                chan_priv->writer = 0;
            }

            if (copy_to_user((void *)arg, &m, sizeof(m))) {
                ret = -EFAULT;
                break;
            }

        }
        break;
    default:
        ret = -ENOIOCTLCMD;
        break;
    }
    return ret;
}

static struct file_operations litepcie_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = litepcie_ioctl,
    .open = litepcie_open,
    .release = litepcie_release,
    .read = litepcie_read,
    .poll = litepcie_poll,
    .write = litepcie_write,
    .mmap = litepcie_mmap,
};

static int litepcie_alloc_chdev(struct litepcie_device *s)
{
    int i, j;
    int ret;
    int index;

    index = litepcie_minor_idx;
    s->minor_base = litepcie_minor_idx;
    for(i = 0; i < s->channels_count; i++) {
        s->chan[i].cdev = cdev_alloc();
        if(!s->chan[i].cdev) {
            ret = -ENOMEM;
            printk(KERN_ERR LITEPCIE_NAME " Failed to allocate cdev\n");
            goto fail_alloc;
        }

        cdev_init(s->chan[i].cdev, &litepcie_fops);
        ret = cdev_add(s->chan[i].cdev, MKDEV(litepcie_major, index), 1);
        if(ret < 0) {
            printk(KERN_ERR LITEPCIE_NAME " Failed to allocate cdev\n");
            goto fail_alloc;
        }
        index++;
    }

    index = litepcie_minor_idx;
    for(i = 0; i < s->channels_count; i++) {
        printk(KERN_INFO LITEPCIE_NAME " Creating /dev/litepcie%d\n", index);
        if(!device_create(litepcie_class, NULL, MKDEV(litepcie_major, index), NULL, "litepcie%d", index)) {
            ret = -EINVAL;
            printk(KERN_ERR LITEPCIE_NAME " Failed to create device\n");
            goto fail_create;
        }
        index++;

    }

    litepcie_minor_idx = index;
    return 0;

fail_create:
    index = litepcie_minor_idx;
    for(j = 0; j < i ; j++){
        device_destroy(litepcie_class, MKDEV(litepcie_major, index++));
    }

fail_alloc:
    for(i = 0; i < s->channels_count;i++) {
        if(s->chan[i].cdev) {
            cdev_del(s->chan[i].cdev);
            s->chan[i].cdev=NULL;
        }
    }

    return ret;
}

static void litepcie_free_chdev(struct litepcie_device *s)
{
    int i;

    for(i = 0; i < s->channels_count; i++){
        device_destroy(litepcie_class, MKDEV(litepcie_major, s->minor_base + i));
        if(s->chan[i].cdev) {
            cdev_del(s->chan[i].cdev);
            s->chan[i].cdev=NULL;
        }
    }
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

    printk(KERN_INFO LITEPCIE_NAME " \e[1m[Probing device]\e[0m\n");

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

    /* create all chardev in /dev */
    ret = litepcie_alloc_chdev(litepcie_dev);
    if(ret){
        printk(KERN_ERR LITEPCIE_NAME "Failed to allocate character device\n");
        goto fail5;
    }

    litepcie_dev->channels = kzalloc(sizeof(*litepcie_dev->channels) * litepcie_dev->channels_count, GFP_KERNEL);
    if(!litepcie_dev->channels) {
        dev_err(&dev->dev, "Cannot allocate memory\n");
        ret = -ENOMEM;
        goto fail1;
    }

    for (i = 0; i < litepcie_dev->channels_count; i++)
        litepcie_dev->channels[i].common = litepcie_dev;

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

    /* allocate all dma buffers */
    ret = litepcie_dma_init(litepcie_dev);
    if(ret){
        printk(KERN_ERR LITEPCIE_NAME "Failed to allocate DMA\n");
        goto fail6;
    }

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
		goto fail6;
	}

    return 0;

fail6:
    litepcie_free_chdev(litepcie_dev);
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
        litepcie_free_chdev(litepcie_dev);
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
        litepcie_dma_free(litepcie_dev);
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
