/*
 * Copyright (c) 2007 Atmel Corporation
 *
 * Based on the bttv driver for Bt848 with respective copyright holders
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
//#define DEBUG			12
//#define VERBOSE		12
//#define VERBOSE_DEBUG		12

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/videodev2.h>
#include <linux/wait.h>

#include <linux/kfifo.h>

#include <asm/io.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>

#include <mach/board.h>
#include <mach/cpu.h>

#include <media/atmel-isi.h>

#define ATMEL_ISI_VERSION	KERNEL_VERSION(0, 1, 0)
#define ISI_CODEC 1

/* Default ISI capture buffer size */
#define ISI_CAPTURE_BUFFER_SIZE 800*600*2
/* Default ISI video frame size ie qvga */
#define ISI_VIDEO_BUFFER_SIZE 320*240*2
/* Default number of ISI video buffers */
#define ISI_VIDEO_BUFFERS 4/*4*2*/
/* Maximum number of video buffers */
#define ISI_VIDEO_BUFFERS_MAX 8*4
/* Interrupt mask for a single capture */
#define ISI_CAPTURE_MASK (ISI_BIT(SOF) | ISI_BIT(FO_C_EMP))
#define ISI_V2_CAPTURE_MASK (ISI_BIT(V2_VSYNC) | ISI_BIT(V2_CXFR_DONE) | ISI_BIT(V2_PXFR_DONE))
/* ISI capture buffer size */
static int capture_buffer_size = ISI_CAPTURE_BUFFER_SIZE;
/* Number of buffers used for streaming video */
static int video_buffers = ISI_VIDEO_BUFFERS;
static int video_buffer_size = ISI_VIDEO_BUFFER_SIZE;

static int input_format = ATMEL_ISI_PIXFMT_CbYCrY;
static u8 has_emb_sync = 0;
static u8 emb_crc_sync = 0;
static u8 hsync_act_low = 0;
static u8 vsync_act_low = 0;
static u8 pclk_act_falling = 0;
static u8 isi_full_mode = 0;
static u8  gs_mode = 0;
/* Preview path horizontal size */
static int prev_hsize = 320;
/* Preview path vertical size */
static int prev_vsize = 240;
/* Scaling factor of the preview path */
static int prev_decimation_factor = 16;

/* Input image horizontal size */
static int image_hsize = 320;
/* Input image vertical size */
static int image_vsize = 240;

/* Frame rate scaler 
 * 1 = capture every second frame
 * 2 = capture every third frame
 * ...
 * */
static int frame_rate_scaler = 0;

/* Set this value if we want to pretend a specific V4L2 output format 
 *  This format is for the capturing interface
 */ 
static int capture_v4l2_fmt = V4L2_PIX_FMT_YUYV;

/* Set this value if we want to pretend a specific V4L2 output format 
 *  This format is for the streaming interface
 */
static int streaming_v4l2_fmt = V4L2_PIX_FMT_YUYV;

/* Declare static vars that will be used as parameters */
static int video_nr = -1;	/* 0 <-> dev/video0, 1 <-> dev/video1, -1 <-> first free */ 

MODULE_PARM_DESC(video_buffers,"Number of frame buffers used for streaming");
module_param(video_buffers, int, 0664);
MODULE_PARM_DESC(capture_buffer_size,"Capture buffer size");
module_param(capture_buffer_size, int, 0664);
MODULE_PARM_DESC(image_hsize,"Horizontal size of input image");
module_param(image_hsize, int, 0664);
MODULE_PARM_DESC(image_vsize,"Vertical size of input image");
module_param(image_vsize, int, 0664);
MODULE_PARM_DESC(frame_rate_scaler, "Frame rate scaler");
module_param(frame_rate_scaler, int, 0664);
MODULE_PARM_DESC(prev_hsize, "Horizontal image size of preview path output");
module_param(prev_hsize, int, 0664);
MODULE_PARM_DESC(prev_vsize, "Vertical image size of preview path output");
module_param(prev_vsize, int, 0664);
MODULE_PARM_DESC(prev_decimation_factor, "Preview path decimaion factor");
module_param(prev_decimation_factor, int, 0664);
module_param(video_nr,          int, 0444);

/* Single frame capturing states */
enum {
	STATE_IDLE = 0,
	STATE_CAPTURE_READY,
	STATE_CAPTURE_WAIT_SOF,
	STATE_CAPTURE_IN_PROGRESS,
	STATE_CAPTURE_DONE,
	STATE_CAPTURE_ERROR,
};

/* Frame buffer states
 *  FRAME_UNUSED Frame(buffer) is not used by the ISI module -> an application
 *  can usually read out data in this state
 *  FRAME_QUEUED An application has queued the buffer in the incoming queue 
 *  FRAME_DONE The ISI module has filled the buffer with data and placed is on
 *  the outgoing queue
 *  FRAME_ERROR Not used at the moment
 *  */
enum frame_status {
	FRAME_UNUSED,
	FRAME_QUEUED,
	FRAME_DONE,
	FRAME_ERROR,
};
/* Frame buffer descriptor 
 *  Used by the ISI module as a linked list for the DMA controller.
 */
struct fbd {
	/* Physical address of the frame buffer */
	dma_addr_t fb_address;
#if defined(CONFIG_ARCH_AT91SAM9G45)
	/* DMA Control Register(new: only in HISI2) */
	u32 dma_ctrl;
#endif
	/* Physical address of the next fbd */
	dma_addr_t next_fbd_address;
};

/* Frame buffer data
 */
struct frame_buffer {
	/*  Frame buffer descriptor
	 *  Used by the ISI DMA controller to provide linked list DMA operation
	 */
	struct fbd fb_desc;
	/* Pointer to the start of the frame buffer */
	void *frame_buffer;
	/* Timestamp of the captured frame */
	struct timeval timestamp;
	/* Frame number of the frame  */
	unsigned long sequence;
	/* Buffer number*/
	int index;
	/* Bytes used in the buffer for data, needed as buffers are always
	 *  aligned to pages and thus may be bigger than the amount of data*/
	int bytes_used;
	/* Mmap count
	 *  Counter to measure how often this buffer is mmapped 
	 */
	int mmap_count;
	/* Buffer status */
	enum frame_status status;
};

struct atmel_isi {
	/* ISI module spin lock. Protects against concurrent access of variables
	 * that are shared with the ISR */
	spinlock_t			lock;
	void __iomem			*regs;
	/* Pointer to the start of the fbd list */
	dma_addr_t			fbd_list_start;
	/* Frame buffers */
	struct frame_buffer 		video_buffer[ISI_VIDEO_BUFFERS_MAX];
	/* Frame buffer currently used by the ISI module */
	struct frame_buffer		*current_buffer;
	/* Size of a frame buffer */
	size_t				capture_buffer_size;
	/* Streaming status 
	 *  If set ISI is in streaming mode */
	int				streaming;
	/* Queue for incoming buffers
	 *  The buffer number (index) is stored in the fifo as reference 
	 */
	struct kfifo 			*grabq;
	/* Spinlock for the incoming queue */
	spinlock_t 			grabq_lock;
	/* Queue for outgoing buffers
	 *  Buffer number is stored in the fifo as reference
	 */
	struct kfifo			*doneq;
	/* Spinlock for the incoming queue */
	spinlock_t			doneq_lock;

	/* State of the ISI module in capturing mode */
	int				state;
	/* Pointer to ISI buffer */
	void				*capture_buf;
	/* Physical address of the capture buffer */
	dma_addr_t			capture_phys;
	/* Size of the ISI buffer */
	size_t				capture_buf_size;
	/* Capture/streaming wait queue */
	wait_queue_head_t		capture_wq;

	struct atmel_isi_camera		*camera;
	struct atmel_isi_format		format;
	struct atmel_isi_format		streaming_format;

	struct mutex			mutex;
	/* User counter for the streaming interface */
	int				stream_users;
	/* User counter of the capture interface */
	int				capture_users;
	/* Video device for capturing (Codec path) */
	struct video_device		cdev;
	/* Video device for streaming (Preview path) */
	struct video_device		vdev;
	struct completion		reset_complete;
	struct clk			*pclk;
	struct clk			*hclk;
	struct platform_device		*pdev;
	unsigned int			irq;
};

#define to_atmel_isi(vdev) container_of(vdev, struct atmel_isi, vdev)

struct atmel_isi_fh {
	struct atmel_isi		*isi;
	unsigned int			read_off;
};

/*
 * The new ISI_V2 IP isn't 100% compatible with the old ISI IP,
 * and it has a few nice features which we want to use...
 */
static inline bool atmelisi_is_isi_v2(void)
{
	if (cpu_is_at91sam9g45())
		return true;

	return false;
}
/*-----------------------------------------------------------------------------
 * Interface to the actual camera.
 */
static LIST_HEAD(camera_list);
static DEFINE_MUTEX(camera_list_mutex);

static void atmel_isi_release_camera(struct atmel_isi *isi,
				     struct atmel_isi_camera *cam)
{
	mutex_lock(&camera_list_mutex);
	cam->isi = NULL;
	isi->camera = NULL;
	module_put(cam->owner);
	mutex_unlock(&camera_list_mutex);
}

int atmel_isi_register_camera(struct atmel_isi_camera *cam)
{
	pr_debug("atmel_isi: register camera %s\n", cam->name);

	mutex_lock(&camera_list_mutex);
	list_add_tail(&cam->list, &camera_list);
	mutex_unlock(&camera_list_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(atmel_isi_register_camera);

void atmel_isi_unregister_camera(struct atmel_isi_camera *cam)
{
	pr_debug("atmel_isi: unregister camera %s\n", cam->name);

	mutex_lock(&camera_list_mutex);
	if (cam->isi)
		cam->isi->camera = NULL;
	list_del(&cam->list);
	mutex_unlock(&camera_list_mutex);
}
EXPORT_SYMBOL_GPL(atmel_isi_unregister_camera);

static struct atmel_isi_camera * atmel_isi_grab_camera(struct atmel_isi *isi)
{
	struct atmel_isi_camera *entry, *cam = NULL;

	mutex_lock(&camera_list_mutex);
	list_for_each_entry(entry, &camera_list, list) {
		/* Just grab the first camera available */
		if (!entry->isi) {
			if (!try_module_get(entry->owner))
				continue;

			cam = entry;
			cam->isi = isi;
			pr_debug("%s: got camera: %s\n",
				 isi->vdev.name, cam->name);
			break;
		}
	}
	mutex_unlock(&camera_list_mutex);

	return cam;
}

static int atmel_isi_set_camera_input(struct atmel_isi *isi)
{
	struct atmel_isi_camera *cam = isi->camera;
	int ret;
	ret = cam->set_format(cam, &isi->format);
	if (ret)
		return ret;

	return 0;
}
static void atmel_isi_set_default_format(struct atmel_isi *isi)
{

	isi->format.pix.width = (u32)min((u32)2048l, (u32)image_hsize);
	isi->format.pix.height = (u32)min((u32)2048l, (u32)image_vsize);

	/* Set capture format if we have explicitely specified one */
	if(capture_v4l2_fmt){
		isi->format.pix.pixelformat = capture_v4l2_fmt;
	}
	else {
		/* Codec path output format */
		isi->format.pix.pixelformat = V4L2_PIX_FMT_YVYU;
	}

	/* The ISI module codec path tries to output YUV 4:2:2
	 * Therefore two pixels will be in a 32bit word */
	isi->format.pix.bytesperline = ALIGN(isi->format.pix.width * 2, 4);
	isi->format.pix.sizeimage = isi->format.pix.bytesperline * 
		isi->format.pix.height;

	pr_debug("set default format: width=%d height=%d\n",
		isi->format.pix.width, isi->format.pix.height);

#ifdef ISI_CODEC
	isi->streaming_format.pix.width = isi->format.pix.width;
	isi->streaming_format.pix.height = isi->format.pix.height;
	isi->streaming_format.pix.bytesperline = isi->format.pix.bytesperline;
	isi->streaming_format.pix.sizeimage = isi->format.pix.sizeimage;
#else
	isi->streaming_format.pix.width = min(640U, prev_hsize);
	isi->streaming_format.pix.height = min(480U, prev_vsize);

	/* The ISI module preview path outputs either RGB 5:5:5
	 * or grayscale mode. Normally 2 pixels are stored in one word. 
	 * But since the grayscale mode offers the possibility to store 1 pixel
	 * in one word we have to adjust the size here.
	 */
	if(input_format == ATMEL_ISI_PIXFMT_GREY 
		&& gs_mode == ISI_GS_1PIX_PER_WORD) {

		isi->streaming_format.pix.bytesperline =
			ALIGN(isi->streaming_format.pix.width *4, 4);
	}
	else {
		isi->streaming_format.pix.bytesperline = 
			ALIGN(isi->streaming_format.pix.width * 2, 4);
	}

	isi->streaming_format.pix.sizeimage = 
		isi->streaming_format.pix.bytesperline * 
		isi->streaming_format.pix.height;
#endif
	/* Set streaming format if we have explicitely specified one */
	if(streaming_v4l2_fmt){
		isi->streaming_format.pix.pixelformat = streaming_v4l2_fmt;
	}
	else {
		/* Preview path output format
		 * Would be logically V4L2_PIX_FMT_BGR555X 
		 * but this format does not exist in the specification
		 * So for now we pretend V4L2_PIX_FMT_RGB555X 
		 * Also the Greyscale format does not fit on top of the V4L2
		 * format but for now we just return it.
		 */
		if(input_format == ATMEL_ISI_PIXFMT_GREY)
			isi->streaming_format.pix.pixelformat = V4L2_PIX_FMT_GREY;
		else
			isi->streaming_format.pix.pixelformat = V4L2_PIX_FMT_RGB555X;
	}
	
	if(input_format){
		isi->format.input_format = input_format;
		/* Not needed but for completeness*/
		isi->streaming_format.input_format = input_format;
	}

}
#if defined(CONFIG_ARCH_AT91SAM9G45)	/* ISI_V2 */
static int atmel_isi_init_hardware(struct atmel_isi *isi)
{
	u32 cfg2, cfg1, cr, ctrl;

	cr = 0;
	switch (isi->format.input_format) {
	case ATMEL_ISI_PIXFMT_GREY:
		cr = ISI_BIT(GRAYSCALE);
		break;
	case ATMEL_ISI_PIXFMT_YCrYCb:
		cr = ISI_BF(V2_YCC_SWAP, 0);
		break;
	case ATMEL_ISI_PIXFMT_YCbYCr:
		cr = ISI_BF(V2_YCC_SWAP, 1);
		break;
	case ATMEL_ISI_PIXFMT_CrYCbY:
		cr = ISI_BF(V2_YCC_SWAP, 2);
		break;
	case ATMEL_ISI_PIXFMT_CbYCrY:
		cr = ISI_BF(YCC_SWAP, 3);
		break;
	case ATMEL_ISI_PIXFMT_RGB24:
		cr = ISI_BIT(V2_COL_SPACE) | ISI_BF(V2_RGB_CFG, 0);
		break;
	case ATMEL_ISI_PIXFMT_BGR24:
		cr = ISI_BIT(V2_COL_SPACE) | ISI_BF(V2_RGB_CFG, 1);
		break;
	case ATMEL_ISI_PIXFMT_RGB16:
		cr = (ISI_BIT(V2_COL_SPACE) | ISI_BIT(V2_RGB_MODE)
		       | ISI_BF(V2_RGB_CFG, 0));
		break;
	case ATMEL_ISI_PIXFMT_BGR16:
		cr = (ISI_BIT(V2_COL_SPACE) | ISI_BIT(V2_RGB_MODE)
		       | ISI_BF(V2_RGB_CFG, 1));
		break;
	case ATMEL_ISI_PIXFMT_GRB16:
		cr = (ISI_BIT(V2_COL_SPACE) | ISI_BIT(V2_RGB_MODE)
		       | ISI_BF(V2_RGB_CFG, 2));
		break;
	case ATMEL_ISI_PIXFMT_GBR16:
		cr = (ISI_BIT(V2_COL_SPACE) | ISI_BIT(V2_RGB_MODE)
		       | ISI_BF(V2_RGB_CFG, 3));
		break;
	case ATMEL_ISI_PIXFMT_RGB24_REV:
		cr = (ISI_BIT(V2_COL_SPACE) | ISI_BIT(V2_RGB_SWAP)
		       | ISI_BF(V2_RGB_CFG, 0));
		break;
	case ATMEL_ISI_PIXFMT_BGR24_REV:
		cr = (ISI_BIT(V2_COL_SPACE) | ISI_BIT(V2_RGB_SWAP)
		       | ISI_BF(V2_RGB_CFG, 1));
		break;
	case ATMEL_ISI_PIXFMT_RGB16_REV:
		cr = (ISI_BIT(V2_COL_SPACE) | ISI_BIT(V2_RGB_SWAP)
		       | ISI_BIT(V2_RGB_MODE) | ISI_BF(V2_RGB_CFG, 0));
		break;
	case ATMEL_ISI_PIXFMT_BGR16_REV:
		cr = (ISI_BIT(V2_COL_SPACE) | ISI_BIT(V2_RGB_SWAP)
		       | ISI_BIT(V2_RGB_MODE) | ISI_BF(V2_RGB_CFG, 1));
		break;
	case ATMEL_ISI_PIXFMT_GRB16_REV:
		cr = (ISI_BIT(V2_COL_SPACE) | ISI_BIT(V2_RGB_SWAP)
		       | ISI_BIT(V2_RGB_MODE) | ISI_BF(V2_RGB_CFG, 2));
		break;
	case ATMEL_ISI_PIXFMT_GBR16_REV:
		cr = (ISI_BIT(V2_COL_SPACE) | ISI_BIT(V2_RGB_SWAP)
		       | ISI_BIT(V2_RGB_MODE) | ISI_BF(V2_RGB_CFG, 3));
		break;
	default:
		return -EINVAL;
	}

	cfg1 = ISI_BF(V2_EMB_SYNC, (has_emb_sync))
		| ISI_BF(V2_HSYNC_POL, hsync_act_low)
		| ISI_BF(V2_VSYNC_POL, vsync_act_low)
		| ISI_BF(V2_PIXCLK_POL, pclk_act_falling)
		| ISI_BF(V2_FULL, isi_full_mode);

	ctrl = ISI_BIT(DIS);

	isi_writel(isi, V2_CFG1, cfg1);
	isi_writel(isi, V2_CTRL, ctrl);
	/* Check if module properly disable */
	while(isi_readl(isi, V2_STATUS) & ISI_BIT(V2_DIS_DONE));

#ifndef ISI_CODEC
	/* These values depend on the sensor output image size */
	isi_writel(isi, V2_PDECF, prev_decimation_factor);/* 1/16 * 16 = 1*/
	isi_writel(isi, V2_PSIZE , ISI_BF(V2_PREV_HSIZE,prev_hsize - 1)
		| ISI_BF(V2_PREV_VSIZE, prev_vsize - 1));
#endif
	cfg2 = isi_readl(isi, V2_CFG2);
	cfg2 |= cr;
	cfg2 = ISI_BFINS(V2_IM_VSIZE, isi->format.pix.height - 1, cfg2);
	cfg2 = ISI_BFINS(V2_IM_HSIZE, isi->format.pix.width - 1, cfg2);

	isi_writel(isi, V2_CFG2, cfg2);

	pr_debug("set_format:\n cfg1=0x%08x\n cfg2=0x%08x\n",
		 isi_readl(isi, V2_CFG1), isi_readl(isi, V2_CFG2));
	pr_debug("psize=0x%08x\n", isi_readl(isi, V2_PSIZE));

	return 0;
}
#else
static int atmel_isi_init_hardware(struct atmel_isi *isi)
{
	u32 cr2, cr1, cr;	
	cr = 0;
	switch (isi->format.input_format) {
	case ATMEL_ISI_PIXFMT_GREY:
		cr = ISI_BIT(GRAYSCALE);
		break;
	case ATMEL_ISI_PIXFMT_YCrYCb:
		cr = ISI_BF(YCC_SWAP, 0);
		break;
	case ATMEL_ISI_PIXFMT_YCbYCr:
		cr = ISI_BF(YCC_SWAP, 1);
		break;
	case ATMEL_ISI_PIXFMT_CrYCbY:
		cr = ISI_BF(YCC_SWAP, 2);
		break;
	case ATMEL_ISI_PIXFMT_CbYCrY:
		cr = ISI_BF(YCC_SWAP, 3);
		break;
	case ATMEL_ISI_PIXFMT_RGB24:
		cr = ISI_BIT(COL_SPACE) | ISI_BF(RGB_CFG, 0);
		break;
	case ATMEL_ISI_PIXFMT_BGR24:
		cr = ISI_BIT(COL_SPACE) | ISI_BF(RGB_CFG, 1);
		break;
	case ATMEL_ISI_PIXFMT_RGB16:
		cr = (ISI_BIT(COL_SPACE) | ISI_BIT(RGB_MODE)
		       | ISI_BF(RGB_CFG, 0));
		break;
	case ATMEL_ISI_PIXFMT_BGR16:
		cr = (ISI_BIT(COL_SPACE) | ISI_BIT(RGB_MODE)
		       | ISI_BF(RGB_CFG, 1));
		break;
	case ATMEL_ISI_PIXFMT_GRB16:
		cr = (ISI_BIT(COL_SPACE) | ISI_BIT(RGB_MODE)
		       | ISI_BF(RGB_CFG, 2));
		break;
	case ATMEL_ISI_PIXFMT_GBR16:
		cr = (ISI_BIT(COL_SPACE) | ISI_BIT(RGB_MODE)
		       | ISI_BF(RGB_CFG, 3));
		break;
	case ATMEL_ISI_PIXFMT_RGB24_REV:
		cr = (ISI_BIT(COL_SPACE) | ISI_BIT(RGB_SWAP)
		       | ISI_BF(RGB_CFG, 0));
		break;
	case ATMEL_ISI_PIXFMT_BGR24_REV:
		cr = (ISI_BIT(COL_SPACE) | ISI_BIT(RGB_SWAP)
		       | ISI_BF(RGB_CFG, 1));
		break;
	case ATMEL_ISI_PIXFMT_RGB16_REV:
		cr = (ISI_BIT(COL_SPACE) | ISI_BIT(RGB_SWAP)
		       | ISI_BIT(RGB_MODE) | ISI_BF(RGB_CFG, 0));
		break;
	case ATMEL_ISI_PIXFMT_BGR16_REV:
		cr = (ISI_BIT(COL_SPACE) | ISI_BIT(RGB_SWAP)
		       | ISI_BIT(RGB_MODE) | ISI_BF(RGB_CFG, 1));
		break;
	case ATMEL_ISI_PIXFMT_GRB16_REV:
		cr = (ISI_BIT(COL_SPACE) | ISI_BIT(RGB_SWAP)
		       | ISI_BIT(RGB_MODE) | ISI_BF(RGB_CFG, 2));
		break;
	case ATMEL_ISI_PIXFMT_GBR16_REV:
		cr = (ISI_BIT(COL_SPACE) | ISI_BIT(RGB_SWAP)
		       | ISI_BIT(RGB_MODE) | ISI_BF(RGB_CFG, 3));
		break;
	default:
		return -EINVAL;
	}

	cr1 = ISI_BF(EMB_SYNC, (has_emb_sync))
		| ISI_BF(HSYNC_POL, hsync_act_low)
		| ISI_BF(VSYNC_POL, vsync_act_low)
		| ISI_BF(PIXCLK_POL, pclk_act_falling)
		| ISI_BF(FULL, isi_full_mode)
		| ISI_BIT(DIS);
	isi_writel(isi, CR1, cr1);

#ifndef ISI_CODEC
	/* These values depend on the sensor output image size */
	isi_writel(isi, PDECF, prev_decimation_factor);/* 1/16 * 16 = 1*/
	isi_writel(isi,PSIZE , ISI_BF(PREV_HSIZE,prev_hsize - 1)
		| ISI_BF(PREV_VSIZE, prev_vsize - 1));
#endif
	cr2 = isi_readl(isi, CR2);
	cr2 |= cr;
	cr2 = ISI_BFINS(IM_VSIZE, isi->format.pix.height - 1, cr2);
	cr2 = ISI_BFINS(IM_HSIZE, isi->format.pix.width - 1, cr2);

	isi_writel(isi, CR2, cr2);

	pr_debug("set_format: cr1=0x%08x\n cr2=0x%08x\n",
		 isi_readl(isi, CR1), isi_readl(isi, CR2));
	pr_debug("psize=0x%08x\n", isi_readl(isi, PSIZE));

	return 0;
}
#endif	/* CONFIG_ARCH_AT91SAM9G45 */

static int atmel_isi_start_capture(struct atmel_isi *isi)
{
	u32 cr, sr=0;
	int ret;

	spin_lock_irq(&isi->lock);
	isi->state = STATE_IDLE;
	if(atmelisi_is_isi_v2()) {
		sr = isi_readl(isi, V2_STATUS); /* clear any pending SOF interrupt */
		isi_writel(isi, V2_INTEN, ISI_BIT(V2_VSYNC)); /* <=> SOF in previous ISI */
		isi_writel(isi, V2_CTRL, isi_readl(isi, V2_CTRL) | ISI_BIT(V2_EN));
		/* Check if module properly enable */
		while(isi_readl(isi, V2_STATUS) & ISI_BIT(V2_ENABLE));
	} else {
		isi_readl(isi, SR); /* clear any pending SOF interrupt */
		isi_writel(isi, IER, ISI_BIT(SOF));
		isi_writel(isi, CR1, isi_readl(isi, CR1) & ~ISI_BIT(DIS));
	}
	spin_unlock_irq(&isi->lock);

	pr_debug("isi: waiting for SOF\n");

	ret = wait_event_interruptible(isi->capture_wq,
				       isi->state != STATE_IDLE);
	if (ret)
		return ret;
	if (isi->state != STATE_CAPTURE_READY)
		return -EIO;

	/*
	 * Do a codec request. Next SOF indicates start of capture,
	 * the one after that indicates end of capture.
	 */
	pr_debug("isi: starting capture\n");
	if(atmelisi_is_isi_v2()) {
		/* Enable */
		isi_writel(isi, V2_DMA_CHER, ISI_BIT(V2_DMA_C_CH_EN));	
		isi_writel(isi, V2_DMA_C_ADDR, isi->capture_phys);
	} else {
		isi_writel(isi, CDBA, isi->capture_phys);
	}

	spin_lock_irq(&isi->lock);
	isi->state = STATE_CAPTURE_WAIT_SOF;
	if(atmelisi_is_isi_v2()) {
		/* Check if already in a frame */
		while(isi_readl(isi, V2_STATUS) & ISI_BIT(V2_CDC));
		cr = isi_readl(isi, V2_CTRL);
		cr |= ISI_BIT(V2_CDC);
		isi_writel(isi, V2_CTRL, cr);
		isi_writel(isi, V2_INTEN, ISI_V2_CAPTURE_MASK);
	} else {	
		cr = isi_readl(isi, CR1);
		cr |= ISI_BIT(CODEC_ON);
		isi_writel(isi, CR1, cr);
		isi_writel(isi, IER, ISI_CAPTURE_MASK);
	}
	spin_unlock_irq(&isi->lock);

	return 0;
}

static void atmel_isi_capture_done(struct atmel_isi *isi,
				   int state)
{
	u32 cr;

	if(atmelisi_is_isi_v2()) {
		cr = isi_readl(isi, V2_CTRL);	
		cr &= ~ISI_BIT(V2_CDC);
		isi_writel(isi, V2_CTRL, cr);
	} else {
		cr = isi_readl(isi, CR1);
		cr &= ~ISI_BIT(CODEC_ON);
		isi_writel(isi, CR1, cr);
	}

	isi->state = state;
	wake_up_interruptible(&isi->capture_wq);
	if(atmelisi_is_isi_v2()) {
		isi_writel(isi, V2_INTDIS, ISI_V2_CAPTURE_MASK);	
	} else {	
		isi_writel(isi, IDR, ISI_CAPTURE_MASK);
	}
}

#if defined(CONFIG_ARCH_AT91SAM9G45)	/* ISI_V2 */
static irqreturn_t atmel_isi_handle_streaming(struct atmel_isi *isi, 
							int sequence){
	
	int reqnr, putnr;
	struct frame_buffer *buffer;

	pr_debug("isi: isi_handle_streaming\n");

	if(kfifo_get(isi->grabq, (unsigned char *) &reqnr,
			sizeof(int)) != sizeof(int)){	
		/* as no new buffer is available we keep the
		* current one
		*/
		pr_debug("Not dequeud yet, so we use the same buffer\n");
	}
	else{
		if(sequence != 0) {
			if(reqnr == 0) {
				putnr = video_buffers-1;
			}
			else {
				putnr = reqnr-1;
			}
		}
		else {
			putnr = 0;
		}
		buffer = &(isi->video_buffer[putnr]);
		buffer->status = FRAME_DONE;
		kfifo_put(isi->doneq, (unsigned char *)
				&(putnr), sizeof(int));
	}
	return IRQ_HANDLED;
}

/* isi interrupt service routine */
static irqreturn_t isi_interrupt(int irq, void *dev_id)
{
	struct atmel_isi *isi = dev_id;
	u32 status, mask, pending;
	irqreturn_t ret = IRQ_NONE;
	/* TODO Should we set sequence to 0 upon each start sequence? */
	static int sequence = 0;

	spin_lock(&isi->lock);

	status = isi_readl(isi, V2_STATUS);
	mask = isi_readl(isi, V2_INTMASK);
	pending = status & mask;

	pr_debug("isi: interrupt:\n status 0x%08x\n pending 0x%08x\n"
			" mask=0x%08x\n",
			status, pending, mask);

	if(isi->streaming){
		if(likely(pending & (ISI_BIT(V2_CXFR_DONE) | ISI_BIT(V2_PXFR_DONE)))) {
			sequence++;
#ifdef ISI_CODEC
			/* if using the codec path we need to set a
			 * CDC request for each frame captured
			 */
			isi_writel(isi, V2_CTRL, 
					isi_readl(isi, V2_CTRL) | ISI_BIT(V2_CDC));
#endif
		 	ret = atmel_isi_handle_streaming(isi, sequence);
		}
	}
	else{
	while (pending) {
		if (pending & (ISI_BIT(V2_C_OVR) | ISI_BIT(V2_FR_OVR))) {
			atmel_isi_capture_done(isi, STATE_CAPTURE_ERROR);
			pr_debug("%s: FIFO overrun (status=0x%x)\n",
				 isi->vdev.name, status);
		} else if (pending & (ISI_BIT(V2_VSYNC) | ISI_BIT(V2_CDC))) {
			switch (isi->state) {
			case STATE_IDLE:
				isi->state = STATE_CAPTURE_READY;
				wake_up_interruptible(&isi->capture_wq);
				break;
			case STATE_CAPTURE_READY:
				break;
			case STATE_CAPTURE_WAIT_SOF:
				isi->state = STATE_CAPTURE_IN_PROGRESS;
				break;
			}
		}
		if (pending & (ISI_BIT(V2_CXFR_DONE) | ISI_BIT(V2_PXFR_DONE))){
			if( isi->state == STATE_CAPTURE_IN_PROGRESS)
				atmel_isi_capture_done(isi, STATE_CAPTURE_DONE);
		}

		if (pending & ISI_BIT(V2_SRST)) {
			complete(&isi->reset_complete);
			isi_writel(isi, V2_INTDIS, ISI_BIT(V2_SRST));
		}

		status = isi_readl(isi, V2_STATUS);
		mask = isi_readl(isi, V2_INTMASK);
		pending = status & mask;
		ret = IRQ_HANDLED;
	}
	}
	spin_unlock(&isi->lock);

	return ret;
}
#else
static irqreturn_t atmel_isi_handle_streaming(struct atmel_isi *isi, 
							int *sequence){
	
	int reqnr, putnr;
	if(!kfifo_get(isi->grabq, (unsigned char *) &reqnr,
			sizeof(int)) /*!= sizeof(int)*/){
			
		/* as no new buffer is available we keep the
		* current one
		*/
		pr_debug("Not dequeud yet, so we use the same buffer\n");
#ifdef ISI_CODEC
		/* Enable codec request */
		isi_writel(isi, CR1, ISI_BIT(CODEC_ON) | 
			isi_readl(isi, CR1));
#else
		/* TEST this has to be tested if it messes up the ISI
		 * streaming process */
		isi_writel(isi, PPFBD, (unsigned long) 
			&isi->video_buffer[isi->current_buffer->index]);
#endif
	}
	else{
		if(sequence != 0) {
			if(reqnr == 0) {
				putnr = video_buffers-1;
			}
			else {
				putnr = reqnr-1;
			}
		}
		else {
			putnr = 0;
		}

		*sequence = *sequence + 1;
		isi->current_buffer = &(isi->video_buffer[putnr]);

		isi->current_buffer->status = FRAME_DONE;
		isi->current_buffer->sequence = *sequence;

		kfifo_put(isi->doneq, (unsigned char *) 
			&(putnr), sizeof(int));

		isi->current_buffer = &(isi->video_buffer[reqnr]);
#ifdef ISI_CODEC
		isi_writel(isi, CDBA, 
			isi->current_buffer->fb_desc.fb_address);
		isi_writel(isi, CR1, ISI_BIT(CODEC_ON) | 
			isi_readl(isi, CR1));
#else
		/*TODO check if fbd corresponds to frame buffer */
#endif
	}
	return IRQ_HANDLED;
}

/* isi interrupt service routine */
static irqreturn_t isi_interrupt(int irq, void *dev_id)
{
	struct atmel_isi *isi = dev_id;
	u32 status, mask, pending;
	irqreturn_t ret = IRQ_NONE;
	/* TODO Should we set sequence to 0 upon each start sequence? */
	static int sequence = 0;

	spin_lock(&isi->lock);

	status = isi_readl(isi, SR);
	mask = isi_readl(isi, IMR);
	pending = status & mask;

	pr_debug("isi: interrupt status %x pending %x\n",
		 status, pending);
	if(isi->streaming){
		if(likely(pending & (ISI_BIT(FO_C_EMP) | ISI_BIT(FO_P_EMP)))){
		 	ret = atmel_isi_handle_streaming(isi, &sequence);
		}
	}
	else{
	while (pending) {
		if (pending & (ISI_BIT(FO_C_OVF) | ISI_BIT(FR_OVR))) {
			atmel_isi_capture_done(isi, STATE_CAPTURE_ERROR);
			pr_debug("%s: FIFO overrun (status=0x%x)\n",
				 isi->vdev.name, status);
		} else if (pending & ISI_BIT(SOF)) {
			switch (isi->state) {
			case STATE_IDLE:
				isi->state = STATE_CAPTURE_READY;
				wake_up_interruptible(&isi->capture_wq);
				break;
			case STATE_CAPTURE_READY:
				break;
			case STATE_CAPTURE_WAIT_SOF:
				isi->state = STATE_CAPTURE_IN_PROGRESS;
				break;
			}
		}
		if (pending & ISI_BIT(FO_C_EMP)){
			if( isi->state == STATE_CAPTURE_IN_PROGRESS)
				atmel_isi_capture_done(isi, STATE_CAPTURE_DONE);
		}

		if (pending & ISI_BIT(SOFTRST)) {
			complete(&isi->reset_complete);
			isi_writel(isi, IDR, ISI_BIT(SOFTRST));
		}

		status = isi_readl(isi, SR);
		mask = isi_readl(isi, IMR);
		pending = status & mask;
		ret = IRQ_HANDLED;
	}
	}
	spin_unlock(&isi->lock);

	return ret;
}
#endif	/* CONFIG_ARCH_AT91SAM9G45 */
/* ------------------------------------------------------------------------
 *  IOCTL videoc handling
 *  ----------------------------------------------------------------------*/

/* --------Capture ioctls ------------------------------------------------*/
/* Device capabilities callback function.
 */
static int atmel_isi_capture_querycap(struct file *file, void *priv,
			      struct v4l2_capability *cap)
{
	strcpy(cap->driver, "atmel-isi");
	strcpy(cap->card, "Atmel Image Sensor Interface");
	cap->version = ATMEL_ISI_VERSION;
	/* V4L2_CAP_VIDEO_CAPTURE -> This is a capture device
	 * V4L2_CAP_READWRITE -> read/write interface used
	 */
	cap->capabilities = (V4L2_CAP_VIDEO_CAPTURE 
			     | V4L2_CAP_READWRITE
			     );
	return 0;
}

/*  Input enumeration callback function. 
 *  Enumerates available input devices.
 *  This can be called many times from the V4L2-layer by 
 *  incrementing the index to get all avaliable input devices.
 */
static int atmel_isi_capture_enum_input(struct file *file, void *priv,
				struct v4l2_input *input)
{
	struct atmel_isi_fh *fh = priv;
	struct atmel_isi *isi = fh->isi;

	/* Just one input (ISI) is available */
	if (input->index != 0)
		return -EINVAL;

	/* Set input name as camera name */
	strlcpy(input->name, isi->camera->name, sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;
	
	/* Set to this value just because this should be set to a 
	 * defined value
	 */
	input->std = V4L2_STD_PAL;

	return 0;
}
/* Selects an input device.
 *  One input device (ISI) currently supported.
 */
static int atmel_isi_capture_s_input(struct file *file, void *priv,
			     unsigned int index)
{
	if (index != 0)
		return -EINVAL;
	return 0;
}

/* Gets current input device.
 */
static int atmel_isi_capture_g_input(struct file *file, void *priv,
			     unsigned int *index)
{
	*index = 0;
	return 0;
}

/* Format callback function 
 * Returns a v4l2_fmtdesc structure with according values to a
 * index.
 * This function is called from user space until it returns 
 * -EINVAL.
 */
static int atmel_isi_capture_enum_fmt_cap(struct file *file, void *priv,
				  struct v4l2_fmtdesc *fmt)
{
	if (fmt->index != 0)
		return -EINVAL;

	/* if we want to pretend another ISI output 
	 * this is usefull if we input an other input format from a camera
	 * than specified in the ISI -> makes it possible to swap bytes 
	 * in the ISI output format but messes up the preview path output
	 */ 
	if(capture_v4l2_fmt){
		fmt->pixelformat = capture_v4l2_fmt;
	}
	else {
		/* This is the format the ISI tries to output */
		strcpy(fmt->description, "YCbYCr (YUYV) 4:2:2");
		fmt->pixelformat = V4L2_PIX_FMT_YUYV;
	}

	return 0;
}

static int atmel_isi_capture_try_fmt_cap(struct file *file, void *priv,
			struct v4l2_format *vfmt)
{
	struct atmel_isi_fh *fh = priv;
	struct atmel_isi *isi = fh->isi;
	/* Just return the current format for now */
	memcpy(&vfmt->fmt.pix, &isi->format.pix,
		sizeof(struct v4l2_pix_format));

	return 0;
}

/* Gets current hardware configuration
 *  For capture devices the pixel format settings are 
 *  important.
 */
static int atmel_isi_capture_g_fmt_cap(struct file *file, void *priv,
			       struct v4l2_format *vfmt)
{
	struct atmel_isi_fh *fh = priv;
	struct atmel_isi *isi = fh->isi;

	/* Return current pixel format */
	memcpy(&vfmt->fmt.pix, &isi->format.pix,
	       sizeof(struct v4l2_pix_format));

	return 0;
}

static int atmel_isi_capture_s_fmt_cap(struct file *file, void *priv,
			       struct v4l2_format *vfmt)
{
	struct atmel_isi_fh *fh = priv;
	struct atmel_isi *isi = fh->isi;
	int ret = 0;
	
	/* We have a fixed format so just copy the current format 
	 * back
	 */
	memcpy(&vfmt->fmt.pix, &isi->format.pix,
		sizeof(struct v4l2_pix_format));

	return ret;
}

/* ------------ Preview path ioctls ------------------------------*/
/* Device capabilities callback function.
 */
static int atmel_isi_streaming_querycap(struct file *file, void *priv,
			      struct v4l2_capability *cap)
{
	strcpy(cap->driver, "atmel-isi");
	strcpy(cap->card, "Atmel Image Sensor Interface");
	cap->version = ATMEL_ISI_VERSION;
	/* V4L2_CAP_VIDEO_CAPTURE -> This is a capture device
	 * V4L2_CAP_READWRITE -> read/write interface used
	 * V4L2_CAP_STREAMING -> ioctl + mmap interface used
	 */
	cap->capabilities = (V4L2_CAP_VIDEO_CAPTURE 
			     | V4L2_CAP_READWRITE
			     | V4L2_CAP_STREAMING
			     );
	return 0;
}
/* Input enumeration callback function. 
 *  Enumerates available input devices.
 *  This can be called many times from the V4L2-layer by 
 *  incrementing the index to get all avaliable input devices.
 */
static int atmel_isi_streaming_enum_input(struct file *file, void *priv,
				struct v4l2_input *input)
{
	struct atmel_isi_fh *fh = priv;
	struct atmel_isi *isi = fh->isi;

	/* Just one input (ISI) is available */
	if (input->index != 0)
		return -EINVAL;

	/* Set input name as camera name */
	strlcpy(input->name, isi->camera->name, sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;
	
	/* Set to this value just because this should be set to a 
	 * defined value
	 */
	input->std = V4L2_STD_PAL;

	return 0;
}
/* Selects an input device.
 *  One input device (ISI) currently supported.
 */
static int atmel_isi_streaming_s_input(struct file *file, void *priv,
			     unsigned int index)
{
	if (index != 0)
		return -EINVAL;

	return 0;
}
/* Gets current input device.
 */
static int atmel_isi_streaming_g_input(struct file *file, void *priv,
			     unsigned int *index)
{
	*index = 0;
	return 0;
}
static int atmel_isi_streaming_g_std(struct file *file, void *priv, v4l2_std_id *norm)
{
	pr_debug("atmel isi: g_std\n");	
	*norm = V4L2_STD_UNKNOWN;
	return 0;
}
/* Format callback function 
 * Returns a v4l2_fmtdesc structure with according values to a
 * index.
 * This function is called from user space until it returns 
 * -EINVAL.
 */
static int atmel_isi_streaming_enum_fmt_cap(struct file *file, void *priv,
				  struct v4l2_fmtdesc *fmt)
{
	struct atmel_isi_fh *fh = priv;
	struct atmel_isi *isi = fh->isi;

	if (fmt->index != 0)
		return -EINVAL;

	/* TODO: Return all possible formats 
	* This depends on ISI and camera.
	* A enum_fmt function or a data structure should be 
	* added to the camera driver.
	* For now just one format supported 
	*/
	if(streaming_v4l2_fmt){
		strcpy(fmt->description, "Pretended format");
	}
	else{
		strcpy(fmt->description, "Normal format");
	}
	/* The pretended and normal format are already set earlier */
	fmt->pixelformat = isi->streaming_format.pix.pixelformat;

	return 0;
}
static int atmel_isi_streaming_try_fmt_cap(struct file *file, void *priv,
			struct v4l2_format *vfmt)
{
	struct atmel_isi_fh *fh = priv;
	struct atmel_isi *isi = fh->isi;

	/* FIXME For now we just return the current format*/
	memcpy(&vfmt->fmt.pix, &isi->streaming_format.pix,
		sizeof(struct v4l2_pix_format));
	return 0;
}
/* Gets current hardware configuration
 *  For capture devices the pixel format settings are 
 *  important.
 */
static int atmel_isi_streaming_g_fmt_cap(struct file *file, void *priv,
			       struct v4l2_format *vfmt)
{
	struct atmel_isi_fh *fh = priv;
	struct atmel_isi *isi = fh->isi;

	/*Copy current pixel format structure to user space*/
	memcpy(&vfmt->fmt.pix, &isi->streaming_format.pix,
	       sizeof(struct v4l2_pix_format));

	return 0;
}
static int atmel_isi_streaming_s_fmt_cap(struct file *file, void *priv,
			       struct v4l2_format *vfmt)
{
	struct atmel_isi_fh *fh = priv;
	struct atmel_isi *isi = fh->isi;
	int ret = 0;
	
	/* Just return the current format as we do not support
	* format switching */
	memcpy(&vfmt->fmt.pix, &isi->streaming_format.pix,
		sizeof(struct v4l2_pix_format));

	return ret;
}
/* Checks if control is supported in driver
 * No controls currently supported yet
 */
static int atmel_isi_streaming_queryctrl(struct file *file, void *priv,
			   struct v4l2_queryctrl *qc)
{
	switch(qc->id){
	case V4L2_CID_BRIGHTNESS:
		strcpy(qc->name, "Brightness");
		qc->minimum = 0;
		qc->maximum = 100;
		qc->step = 1;
		qc->default_value = 50;
		qc->flags = 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
static int atmel_isi_streaming_g_ctrl(struct file *file, void *priv,
			struct v4l2_control *ctrl)
{
	switch(ctrl->id){
	case V4L2_CID_BRIGHTNESS:
		ctrl->value = 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
static int atmel_isi_streaming_s_ctrl(struct file *file, void *priv,
			struct v4l2_control *ctrl)
{
	switch(ctrl->id){
	case V4L2_CID_BRIGHTNESS:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
static int atmel_isi_reqbufs(struct file *file, void *private_data, 
			struct v4l2_requestbuffers *req)
{
	/* Only memory mapped buffers supported*/
	if(req->memory != V4L2_MEMORY_MMAP){
		pr_debug("atmel_isi: buffer format not supported\n");
		return -EINVAL;
	}
	pr_debug("atmel_isi: Requested %d buffers. Using %d buffers\n",
		req->count, video_buffers);
	/* buffer number is fixed for now as it is difficult to get 
	 * that memory at runtime */	
	req->count = video_buffers;
	memset(&req->reserved, 0, sizeof(req->reserved));
	return 0;
}

static int atmel_isi_querybuf(struct file *file, void *private_data, 
			struct v4l2_buffer *buf)
{
	struct atmel_isi_fh *fh = private_data;
	struct atmel_isi *isi = fh->isi;
	struct frame_buffer *buffer; 

	if(unlikely(buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE))
		return -EINVAL;
	if(unlikely(buf->index >= video_buffers))
		return -EINVAL;

	buffer = &(isi->video_buffer[buf->index]);

	buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf->length = video_buffer_size;
	buf->memory = V4L2_MEMORY_MMAP;

	/* set index as mmap reference to the buffer */
	buf->m.offset = buf->index << PAGE_SHIFT;
	
	switch(buffer->status){
	case FRAME_UNUSED:
	case FRAME_ERROR:
	case FRAME_QUEUED:
		buf->flags |= V4L2_BUF_FLAG_QUEUED;
		buf->bytesused = buffer->bytes_used;
		break;
	case FRAME_DONE:
		buf->flags |= V4L2_BUF_FLAG_DONE;
		buf->bytesused = buffer->bytes_used;
		buf->sequence = buffer->sequence;
		buf->timestamp = buffer->timestamp;
		break;
	}

	buf->field = V4L2_FIELD_NONE; /* no interlacing stuff */

	if(buffer->mmap_count)
		buf->flags |= V4L2_BUF_FLAG_MAPPED;
	else 
		buf->flags &= ~V4L2_BUF_FLAG_MAPPED;

	pr_debug("atmel_isi: querybuf index:%d offset:%d\n",
		buf->index, buf->m.offset);

	return 0;
}

static int atmel_isi_qbuf(struct file *file, void *private_data,
			struct v4l2_buffer *buf)
{
	struct atmel_isi_fh *fh = private_data;
	struct atmel_isi *isi = fh->isi;
	struct frame_buffer *buffer;

	if(unlikely(buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE))
		return -EINVAL;
	if(unlikely(buf->index >= video_buffers || buf->index < 0)){
		pr_debug("Buffer index is not valid index=%d\n",buf->index);
		return -EINVAL;
	}
	if(unlikely(buf->memory != V4L2_MEMORY_MMAP)){
		pr_debug("Buffer is not of MEMORY_MMAP type\n"); 
		return -EINVAL;
	}

	buffer = &(isi->video_buffer[buf->index]);
	if(unlikely(buffer->status != FRAME_UNUSED)){
		pr_debug("Can't queue non unused frames\n");	
		return -EINVAL;
	}

	mutex_lock(&isi->mutex);
	buf->flags |= V4L2_BUF_FLAG_QUEUED;
	buf->flags &= ~V4L2_BUF_FLAG_DONE;
	buffer->status = FRAME_QUEUED;
	kfifo_put(isi->grabq, (unsigned char*) &buf->index, sizeof(int));
	mutex_unlock(&isi->mutex);

	return 0;
}

static int atmel_isi_dqbuf(struct file *file, void *private_data,
			struct v4l2_buffer *buf)
{
	struct atmel_isi_fh *fh = private_data;
	struct atmel_isi *isi = fh->isi;
	struct frame_buffer *buffer;
	int reqnr = 0;

	if(unlikely(buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE))
		return -EINVAL;
	if((kfifo_len(isi->doneq) == 0)){
		pr_debug("Done-queue is empty\n");
		return -EAGAIN;
	}
	if(!kfifo_get(isi->doneq, (unsigned char*) &reqnr, sizeof(int))){
		pr_debug("No new buffer ready\n");
		return -EBUSY;
	}
	buffer = &(isi->video_buffer[reqnr]);

	if(unlikely(buffer->status != FRAME_DONE)){
		if(isi->streaming == 0)
			return 0;
		pr_debug("isi: error, dequeued buffer not ready\n");
		return -EINVAL;
	}

	mutex_lock(&isi->mutex);
	buf->index = reqnr;
	buf->bytesused = buffer->bytes_used;
	buf->timestamp = buffer->timestamp;
	buf->sequence = buffer->sequence;
	buf->m.offset = reqnr << PAGE_SHIFT;
	buffer->status = FRAME_UNUSED;
	buf->flags = V4L2_BUF_FLAG_MAPPED | V4L2_BUF_FLAG_DONE;

	buf->length = video_buffer_size;
	buf->field = V4L2_FIELD_NONE;
	buf->memory = V4L2_MEMORY_MMAP;
	mutex_unlock(&isi->mutex);	
	return 0;
}
static int atmel_isi_streamon(struct file *file, void *private_data,
			enum v4l2_buf_type type)
{
	struct atmel_isi_fh *fh = private_data;
	struct atmel_isi *isi = fh->isi;
	int reqnr;
	struct frame_buffer *buffer;
	u32 cr1, cfg1, ctrl;

	if(unlikely(type != V4L2_BUF_TYPE_VIDEO_CAPTURE))
		return -EINVAL;
	
	if(!kfifo_get(isi->grabq, (unsigned char*) &reqnr, sizeof(int))){
		pr_debug("atmel_isi: No buffer in IN-Queue, start of streaming\
			aborted (one buffer is required in IN-Queue)\n"); 
		return -EINVAL;
	}
	buffer = &(isi->video_buffer[reqnr]);
	spin_lock_irq(isi->lock);
	isi->streaming = 1;
	if(atmelisi_is_isi_v2()) {
		ctrl = isi_readl(isi, V2_CTRL);
		cfg1 = isi_readl(isi, V2_CFG1);
		/* Disable irq: cxfr for the codec path, pxfr for the preview path */
		isi_writel(isi, V2_INTDIS, ISI_BIT(V2_CXFR_DONE) | ISI_BIT(V2_PXFR_DONE));
	} else {	
		isi->current_buffer = buffer;
		cr1 = isi_readl(isi, CR1);
	}
#ifdef ISI_CODEC
#if defined(CONFIG_ARCH_AT91SAM9G45)
		/* Enable codec path */
		ctrl |= ISI_BIT(V2_CDC);
		/* Check if already in a frame */
		while(isi_readl(isi, V2_STATUS) & ISI_BIT(V2_CDC));
		/* Write the address of the first frame buffer in the C_ADDR reg
	 	* write the address of the first descriptor(link list of buffer)
	 	* in the C_DSCR reg
	 	*/
		isi_writel(isi, V2_DMA_CHER, ISI_BIT(V2_DMA_C_CH_EN));	
		isi_writel(isi, V2_DMA_C_ADDR, buffer->fb_desc.fb_address);
		isi_writel(isi, V2_DMA_C_DSCR, isi->fbd_list_start);
		isi_writel(isi, V2_DMA_C_CTRL, buffer->fb_desc.dma_ctrl);
#else
		isi_writel(isi, CDBA, buffer->fb_desc.fb_address);
		/* Enable codec path */
		cr1 |= ISI_BIT(CODEC_ON) | ISI_BIT(DIS);
#endif
#else
#if defined(CONFIG_ARCH_AT91SAM9G45)
		/* Same as in the codec path, but for the preview path */
		isi_writel(isi, V2_DMA_CHER, ISI_BIT(V2_DMA_P_CH_EN));	
		isi_writel(isi, V2_DMA_P_ADDR, buffer->fb_desc.fb_address);
		isi_writel(isi, V2_DMA_P_DSCR, isi->fbd_list_start);
		isi_writel(isi, V2_DMA_P_CTRL, buffer->fb_desc.dma_ctrl);
#else
		isi_writel(isi, PPFBD, isi->fbd_list_start);
#endif
#endif
	if(atmelisi_is_isi_v2()) {
		/* Enable interrupts */
		isi_readl(isi, V2_STATUS);
		isi_writel(isi, V2_INTEN, ISI_BIT(V2_CXFR_DONE) | ISI_BIT(V2_PXFR_DONE));
		cfg1 |= ISI_BF(V2_FRATE, frame_rate_scaler);
		/* Enable ISI module*/
		ctrl |= ISI_BIT(V2_ENABLE);	
		isi_writel(isi, V2_CTRL, ctrl);
		isi_writel(isi, V2_CFG1, cfg1);	
		/* Dump registers */
		pr_debug("atmel_isi: Stream on\n");
		pr_debug("cfg1 register= 0x%x\n", isi_readl(isi, V2_CFG1));
		pr_debug("cfg2 register= 0x%x\n", isi_readl(isi, V2_CFG2));
		pr_debug("control register= 0x%x\n", isi_readl(isi, V2_CTRL));		
		pr_debug("status register=0x%x\n", isi_readl(isi, V2_STATUS));
		pr_debug("interrupt mask register= 0x%x\n", isi_readl(isi, V2_INTMASK));		
		pr_debug("DMA status register=0x%08x\n", isi_readl(isi, V2_DMA_CHSR));
	} else {	
		/* Enable interrupts */
		isi_readl(isi, SR);
		/* FIXME enable codec/preview path according to setup */
		isi_writel(isi, IER, ISI_BIT(FO_C_EMP) | ISI_BIT(FO_P_EMP));
		cr1 |= ISI_BF(FRATE, frame_rate_scaler);
		/* Enable ISI module*/
		cr1 &= ~ISI_BIT(DIS);
		isi_writel(isi, CR1, cr1);
		/* Dump registers */
		pr_debug("atmel_isi: Stream on\n");
		pr_debug("CR1 0x%x\n", isi_readl(isi, CR1));
		pr_debug("CR2 0x%x\n", isi_readl(isi, CR2));
		pr_debug("SR 0x%x", isi_readl(isi, SR));
		pr_debug("IMR 0x%x", isi_readl(isi, IMR));		
	}

	spin_unlock_irq(isi->lock);

	if(isi->camera)
		isi->camera->start_capture(isi->camera, &isi->format);

	return 0;
}

static int atmel_isi_streamoff(struct file *file, void *private_data,
			enum v4l2_buf_type type)
{
	struct atmel_isi_fh *fh = private_data;
	struct atmel_isi *isi = fh->isi;
	int reqnr;

	if(unlikely(type != V4L2_BUF_TYPE_VIDEO_CAPTURE))
		return -EINVAL;

	spin_lock_irq(isi->lock);
	isi->streaming = 0;
#ifdef ISI_CODEC
	if(atmelisi_is_isi_v2()){
		/* Disble codec path */
		isi_writel(isi, V2_CTRL, isi_readl(isi, V2_CTRL) & (~ISI_BIT(V2_CDC)));
	} else{	
		/* Disble codec path */
		isi_writel(isi, CR1, isi_readl(isi, CR1) & (~ISI_BIT(CODEC_ON)));
	}
#endif
	if(atmelisi_is_isi_v2()){
		/* Disable interrupts */
		isi_writel(isi, V2_INTDIS, ISI_BIT(V2_CXFR_DONE) | ISI_BIT(V2_PXFR_DONE));	
		/* Disable ISI module*/
		isi_writel(isi, V2_CTRL, isi_readl(isi, V2_CTRL) | ISI_BIT(V2_DIS));	
	} else{	
		/* Disable interrupts */
		isi_writel(isi, IDR, ISI_BIT(FO_C_EMP) | ISI_BIT(FO_P_EMP));
		/* Disable ISI module*/
		isi_writel(isi, CR1, isi_readl(isi, CR1) | ISI_BIT(DIS));
	}

	spin_unlock_irq(isi->lock);
	
	if(isi->camera)
		isi->camera->stop_capture(isi->camera);

	while(kfifo_len(isi->grabq)){
		kfifo_get(isi->grabq, (unsigned char *) &reqnr, sizeof(int));
		kfifo_put(isi->doneq, (unsigned char *) &reqnr, sizeof(int)); 
	}
	for(reqnr = 0;  reqnr < video_buffers; reqnr++){
		isi->video_buffer[reqnr].status = FRAME_UNUSED;
	}
	return 0;
}
static int atmel_isi_g_parm(struct file *file, void *f,
				struct v4l2_streamparm *parm)
{
	int err = 0;
	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	return err;
}
/*----------------------------------------------------------------------------*/

static int atmel_isi_init(struct atmel_isi *isi){
	unsigned long timeout;

	/*
	 * Reset the controller and wait for completion. 
	 * The reset will only succeed if we have a 
	 * pixel clock from the camera.
	 */
	init_completion(&isi->reset_complete);
	if(atmelisi_is_isi_v2()) {
		isi_writel(isi, V2_INTEN, ISI_BIT(V2_SRST));
		isi_writel(isi, V2_CTRL, ISI_BIT(V2_SRST));
	} else {	
		isi_writel(isi, IER, ISI_BIT(SOFTRST));
		isi_writel(isi, CR1, ISI_BIT(RST));
	}
	
	timeout = wait_for_completion_timeout(&isi->reset_complete,
		msecs_to_jiffies(100));
	if (timeout == 0) {
		return -ETIMEDOUT;
	}

	if(atmelisi_is_isi_v2()) {
		isi_writel(isi, V2_INTDIS, ~0UL);		
	} else {	
		isi_writel(isi, IDR, ~0UL);
	}
	
	atmel_isi_set_default_format(isi);

	/* If no camera is active try to find one*/
	if (!isi->camera) {
		isi->camera = atmel_isi_grab_camera(isi);
		
		/*If a camera was found and it offers a configuration
		 * interface we will use it.
		 */
		if (isi->camera && isi->camera->set_format)
		{
			atmel_isi_set_camera_input(isi);
			isi->camera->set_format(isi->camera, &isi->format);
			
		}
		else
		{
			printk(KERN_INFO "Pb no camera found!!!\n");
		}
	}
	atmel_isi_init_hardware(isi);
	
	return 0;
}

static int atmel_isi_capture_close (struct inode *inode, struct file *file)
{
	struct atmel_isi_fh *fh = file->private_data;
	struct atmel_isi *isi = fh->isi;
	u32 cr;

	mutex_lock(&isi->mutex);

	isi->capture_users--;
	kfree(fh);

	/* Stop camera and ISI  if driver has no users */
	if(!isi->stream_users) {
		if(isi->camera)
			isi->camera->stop_capture(isi->camera);

		spin_lock_irq(&isi->lock);
		if(atmelisi_is_isi_v2()) {
			cr = isi_readl(isi, V2_CTRL);
			cr |= ISI_BIT(V2_DIS);
			isi_writel(isi, V2_CTRL, cr);
		} else {
			cr = isi_readl(isi, CR1);
			cr |= ISI_BIT(DIS);
			isi_writel(isi, CR1, cr);
		}
		spin_unlock_irq(&isi->lock);
	}
	mutex_unlock(&isi->mutex);

	return 0;
}

static int atmel_isi_capture_open (struct inode *inode, struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct atmel_isi *isi = container_of(vdev, struct atmel_isi, cdev);
	struct atmel_isi_fh *fh;
	int ret = -EBUSY;

	pr_debug("%s: opened\n", vdev->name);

	mutex_lock(&isi->mutex);
	

	if (isi->capture_users) {
		pr_debug("%s: open(): device busy\n", vdev->name);
		goto out;
	}

	/* If the streaming interface has no users too we do a
	 * init of the hardware and software configuration.
	 */
	if(isi->stream_users == 0){
		ret = atmel_isi_init(isi);
		if(ret)
			goto out;
	}
	
	ret = -ENOMEM;
	fh = kzalloc(sizeof(struct atmel_isi_fh), GFP_KERNEL);
	if (!fh) {
		pr_debug("%s: open(): out of memory\n", vdev->name);
		goto out;
	}


	fh->isi = isi;
	file->private_data = fh;
	isi->capture_users++;

	ret = 0;

out:
	mutex_unlock(&isi->mutex);
	return ret;
}

static ssize_t atmel_isi_capture_read(struct file *file, char __user *data,
			      size_t count, loff_t *ppos)
{
	struct atmel_isi_fh *fh = file->private_data;
	struct atmel_isi *isi = fh->isi;
	int state;
	int ret;

	state = STATE_IDLE;

	pr_debug("isi: read %zu bytes read_off=%u state=%u sizeimage=%u\n",
		count, fh->read_off, state, isi->format.pix.sizeimage);

	if(isi->camera)
		isi->camera->start_capture(isi->camera, &isi->format);
	

	atmel_isi_start_capture(isi);

	ret = wait_event_interruptible( isi->capture_wq,
			(isi->state == STATE_CAPTURE_DONE)
			|| (isi->state == STATE_CAPTURE_ERROR));
	if (ret)
		return ret;
	if (isi->state == STATE_CAPTURE_ERROR) {
		isi->state = STATE_IDLE;
		return -EIO;
	}

	fh->read_off = 0;

	count = min(count, (size_t)isi->format.pix.sizeimage - fh->read_off);
	ret = copy_to_user(data, isi->capture_buf + fh->read_off, count);
	if (ret)
		return -EFAULT;

	fh->read_off += count;
	if (fh->read_off >= isi->format.pix.sizeimage)
		isi->state = STATE_IDLE;

	return count;
}

static void atmel_isi_capture_release (struct video_device *vdev)
{
	pr_debug("%s: release\n", vdev->name);
}
/* ----------------- Streaming interface -------------------------------------*/
static void atmel_isi_vm_open(struct vm_area_struct *vma){
	struct frame_buffer *buffer = 
		(struct frame_buffer *) vma->vm_private_data;
	buffer->mmap_count++;
	pr_debug("atmel_isi: vm_open count=%d\n",buffer->mmap_count);
}

static void atmel_isi_vm_close(struct vm_area_struct *vma){
	struct frame_buffer *buffer = 
		(struct frame_buffer *) vma->vm_private_data;
	pr_debug("atmel_isi: vm_close count=%d\n",buffer->mmap_count);	
	buffer->mmap_count--;
	if(buffer->mmap_count < 0)
		printk("atmel_isi: mmap_count went negative\n");
}


static struct vm_operations_struct atmel_isi_vm_ops = {
	.open = atmel_isi_vm_open,
	.close = atmel_isi_vm_close,
};

static int atmel_isi_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long pfn;
	int ret;
	struct atmel_isi_fh *fh = file->private_data;
	struct atmel_isi * isi = fh->isi;
	struct frame_buffer *buffer = &(isi->video_buffer[vma->vm_pgoff]);
	unsigned long size = vma->vm_end - vma->vm_start;

	pr_debug("atmel_isi: mmap called pgoff=%ld size=%ld \n", 
		vma->vm_pgoff, size);

	if(size > video_buffer_size){
		pr_debug("atmel_isi: mmap requested buffer is to large\n");
		return -EINVAL;
	}
	if(vma->vm_pgoff > video_buffers){
		pr_debug("atmel_isi: invalid mmap page offset\n");
		return -EINVAL;
	}
	pfn = isi->video_buffer[vma->vm_pgoff].fb_desc.fb_address >> PAGE_SHIFT;

	ret = remap_pfn_range(vma, vma->vm_start, pfn,
		vma->vm_end - vma->vm_start, vma->vm_page_prot);
	if(ret){
		return ret;
	}
	
	vma->vm_ops = &atmel_isi_vm_ops;
	vma->vm_flags |= VM_DONTEXPAND; /* fixed size */
	vma->vm_flags |= VM_RESERVED;/* do not swap out */
	vma->vm_flags |= VM_DONTCOPY;
	vma->vm_flags |= VM_SHARED;
	vma->vm_private_data = (void *) buffer;
	atmel_isi_vm_open(vma);

	pr_debug("atmel_isi: vma start=0x%08lx, size=%ld phys=%ld \n",
		(unsigned long) vma->vm_start,
		(unsigned long) vma->vm_end - (unsigned long) vma->vm_start,
		pfn << PAGE_SHIFT);
	return 0;
}

static unsigned int atmel_isi_poll(struct file *file, poll_table *wait)
{
	struct atmel_isi_fh *fh = file->private_data;
	struct atmel_isi *isi = fh->isi;
	unsigned int ret = 0;

	mutex_lock(&isi->mutex);
	poll_wait(file, &isi->capture_wq, wait);
	if(kfifo_len(isi->doneq))
		ret = POLLIN | POLLRDNORM;
	mutex_unlock(&isi->mutex);

	return ret;
}

static int atmel_isi_stream_close (struct inode *inode, struct file *file)
{
	struct atmel_isi_fh *fh = file->private_data;
	struct atmel_isi *isi = fh->isi;
	u32 cr;

	mutex_lock(&isi->mutex);

	isi->stream_users--;
	kfree(fh);

	/* Stop camera and ISI if driver has no users */
	if(!isi->capture_users) {
		if(isi->camera)
			isi->camera->stop_capture(isi->camera);

		spin_lock_irq(&isi->lock);
		if(atmelisi_is_isi_v2()) {
			cr = isi_readl(isi, V2_CTRL);
			cr |= ISI_BIT(V2_DIS);
			isi_writel(isi, V2_CTRL, cr);
		} else {		
			cr = isi_readl(isi, CR1);
			cr |= ISI_BIT(DIS);
			isi_writel(isi, CR1, cr);
		}
		spin_unlock_irq(&isi->lock);
	}

	mutex_unlock(&isi->mutex);

	return 0;
}

static int atmel_isi_stream_open (struct inode *inode, struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct atmel_isi *isi = to_atmel_isi(vdev);
	struct atmel_isi_fh *fh;
	int ret = -EBUSY;

	mutex_lock(&isi->mutex);
	/* Just one user is allowed for the streaming device*/
	if (isi->stream_users) {
		pr_debug("%s: open(): device busy\n", vdev->name);
		goto out;
	}

	/* If the capture interface is unused too we do a 
	 * init of hardware/software configuration
	 */
	if(isi->capture_users == 0){
		ret = atmel_isi_init(isi);
		if(ret)
			goto out;
	}

	ret = -ENOMEM;
	fh = kzalloc(sizeof(struct atmel_isi_fh), GFP_KERNEL);
	if (!fh) {
		pr_debug("%s: open(): out of memory\n", vdev->name);
		goto out;
	}
	kfifo_reset(isi->grabq);
	kfifo_reset(isi->doneq);
	fh->isi = isi;
	file->private_data = fh;
	isi->stream_users++;

	ret = 0;

out:
	mutex_unlock(&isi->mutex);
	return ret;
}

static void atmel_isi_stream_release (struct video_device *vdev)
{
	struct atmel_isi *isi = to_atmel_isi(vdev);
	pr_debug("%s: release\n", vdev->name);
	kfree(isi);
}
/* -----------------------------------------------------------------------*/
/* Streaming v4l2 device file operations */
static struct file_operations atmel_isi_streaming_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= video_ioctl2,
	.open		= atmel_isi_stream_open,
	.release	= atmel_isi_stream_close,
	.mmap		= atmel_isi_mmap,
	.poll		= atmel_isi_poll,
};
/* Capture v4l2 device file operations */
static struct file_operations atmel_isi_capture_fops = {
	.owner		= THIS_MODULE,
	.open		= atmel_isi_capture_open,
	.release	= atmel_isi_capture_close,
	.read		= atmel_isi_capture_read,
	.ioctl		= video_ioctl2,
};

static int __exit atmel_isi_remove(struct platform_device *pdev)
{
	struct atmel_isi *isi = platform_get_drvdata(pdev);
	int i;

	if (isi->camera)
		isi->camera->stop_capture(isi->camera);

	if (isi->camera)
		atmel_isi_release_camera(isi, isi->camera);
	video_unregister_device(&isi->cdev);
	video_unregister_device(&isi->vdev);

	platform_set_drvdata(pdev, NULL);

	/* release capture buffer */
	dma_free_coherent(&pdev->dev, capture_buffer_size,
			  isi->capture_buf, isi->capture_phys);

	/* release frame buffers */
	for(i = 0; i < video_buffers; i++){
		dma_free_coherent(&pdev->dev, 
			video_buffer_size,
			isi->video_buffer[i].frame_buffer,
			isi->video_buffer[i].fb_desc.fb_address);
	}

	kfifo_free(isi->doneq);
	kfifo_free(isi->grabq);
	
	free_irq(isi->irq, isi);
	iounmap(isi->regs);
	clk_disable(isi->hclk);
	clk_disable(isi->pclk);
	clk_put(isi->hclk);
	clk_put(isi->pclk);

	/*
	 * Don't free isi here -- it will be taken care of by the
	 * release() callback.
	 */

	return 0;
}


static const struct v4l2_ioctl_ops atmel_isi_capture_ioctl_ops = {
	.vidioc_querycap                = atmel_isi_capture_querycap,
	.vidioc_enum_fmt_vid_cap        = atmel_isi_capture_enum_fmt_cap,
	.vidioc_g_fmt_vid_cap           = atmel_isi_capture_g_fmt_cap,
	.vidioc_try_fmt_vid_cap         = atmel_isi_capture_try_fmt_cap,
	.vidioc_s_fmt_vid_cap           = atmel_isi_capture_s_fmt_cap, 
	.vidioc_reqbufs                 = atmel_isi_reqbufs,
	.vidioc_querybuf                = atmel_isi_querybuf,
	.vidioc_qbuf                    = atmel_isi_qbuf,
	.vidioc_dqbuf                   = atmel_isi_dqbuf,
	.vidioc_enum_input              = atmel_isi_capture_enum_input,
	.vidioc_g_input                 = atmel_isi_capture_g_input,
	.vidioc_s_input                 = atmel_isi_capture_s_input,
};

static struct video_device atmel_isi_capture_template = {
	.fops         = &atmel_isi_capture_fops,
	.minor        = -1,
	.ioctl_ops    = &atmel_isi_capture_ioctl_ops,
	.current_norm = V4L2_STD_PAL,
};
static const struct v4l2_ioctl_ops atmel_isi_streaming_ioctl_ops = {
	.vidioc_querycap                = atmel_isi_streaming_querycap,
	.vidioc_enum_fmt_vid_cap        = atmel_isi_streaming_enum_fmt_cap,
	.vidioc_g_fmt_vid_cap           = atmel_isi_streaming_g_fmt_cap,
	.vidioc_try_fmt_vid_cap         = atmel_isi_streaming_try_fmt_cap,
	.vidioc_s_fmt_vid_cap           = atmel_isi_streaming_s_fmt_cap, 
	.vidioc_reqbufs                 = atmel_isi_reqbufs,
	.vidioc_querybuf                = atmel_isi_querybuf,
	.vidioc_qbuf                    = atmel_isi_qbuf,
	.vidioc_dqbuf                   = atmel_isi_dqbuf,
	.vidioc_enum_input              = atmel_isi_streaming_enum_input,
	.vidioc_g_input                 = atmel_isi_streaming_g_input,
	.vidioc_s_input                 = atmel_isi_streaming_s_input,
	.vidioc_g_std		 	= atmel_isi_streaming_g_std,	
	.vidioc_queryctrl               = atmel_isi_streaming_queryctrl,
	.vidioc_g_ctrl                  = atmel_isi_streaming_g_ctrl,
	.vidioc_s_ctrl                  = atmel_isi_streaming_s_ctrl,
	.vidioc_streamon                = atmel_isi_streamon,
	.vidioc_streamoff               = atmel_isi_streamoff,
	.vidioc_g_parm                  = atmel_isi_g_parm,	
};

static struct video_device atmel_isi_streaming_template = {
	.fops         = &atmel_isi_streaming_fops,
	.minor        = -1,
	.ioctl_ops    = &atmel_isi_streaming_ioctl_ops,
	.current_norm = V4L2_STD_PAL,
};

static int __init atmel_isi_probe(struct platform_device *pdev)
{
	unsigned int irq;
	struct atmel_isi *isi;
	struct clk *pclk;
	struct resource *regs;
	int ret;
	int i;
	int video_bytes_used = video_buffer_size;
	struct device *dev = &pdev->dev;
	struct isi_platform_data* pdata;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!regs)
		return -ENXIO;

	pclk = clk_get(&pdev->dev, "isi_clk");
	if (IS_ERR(pclk))
		return PTR_ERR(pclk);

	clk_enable(pclk);

	isi = kzalloc(sizeof(struct atmel_isi), GFP_KERNEL);
	if(!isi){
		ret = -ENOMEM;
		dev_err(&pdev->dev, "can't allocate interface!\n");
		goto err_alloc_isi;
	}

	isi->pclk = pclk;

	spin_lock_init(&isi->lock);
	mutex_init(&isi->mutex);
	init_waitqueue_head(&isi->capture_wq);

	/* Initialize v4l2 capture device */
	isi->cdev = atmel_isi_capture_template;
	isi->cdev.release =atmel_isi_capture_release;
	strcpy(isi->cdev.name, "atmel_isi_capture");
#ifdef DEBUG
	isi->cdev.debug = V4L2_DEBUG_IOCTL | V4L2_DEBUG_IOCTL_ARG;
#endif
	/* Initialize v4l2 streaming device */
	isi->vdev = atmel_isi_streaming_template;
	isi->vdev.release = atmel_isi_stream_release;	
	strcpy(isi->vdev.name, "atmel_isi_streaming");
#ifdef DEBUG
	isi->vdev.debug = V4L2_DEBUG_IOCTL | V4L2_DEBUG_IOCTL_ARG;
#endif
	isi->regs = ioremap(regs->start, regs->end - regs->start + 1);
        if (!isi->regs) {
		ret = -ENOMEM;
		goto err_ioremap;
	}

	if(dev->platform_data){
		pdata = (struct isi_platform_data*) dev->platform_data;
		dev_info(&pdev->dev, "Reading configuration\n");
		image_hsize = pdata->image_hsize;
		image_vsize = pdata->image_vsize;

		if(pdata->prev_hsize)
			prev_hsize = pdata->prev_hsize;
		if(pdata->prev_vsize)
			prev_vsize = pdata->prev_vsize;
		gs_mode = pdata->gs_mode;
		if(pdata->pixfmt) {		
			input_format = pdata->pixfmt;
		}
		else {
			input_format = ATMEL_ISI_PIXFMT_YCbYCr;
		}
		frame_rate_scaler = pdata->frate;
		if(pdata->capture_v4l2_fmt)
			capture_v4l2_fmt = pdata->capture_v4l2_fmt;
		if(pdata->streaming_v4l2_fmt)
			streaming_v4l2_fmt = pdata->streaming_v4l2_fmt;
		if(pdata->cr1_flags & ISI_HSYNC_ACT_LOW)
			hsync_act_low = 1;
		if(pdata->cr1_flags & ISI_VSYNC_ACT_LOW)
			vsync_act_low = 1;
		if(pdata->cr1_flags & ISI_PXCLK_ACT_FALLING)
			pclk_act_falling = 1;
		if(pdata->cr1_flags & ISI_EMB_SYNC)
			has_emb_sync = 1;
		if(pdata->cr1_flags & ISI_CRC_SYNC)
			emb_crc_sync = 1;
		if(pdata->cr1_flags & ISI_FULL)
			isi_full_mode = 1;		
	}
	else{
		dev_info(&pdev->dev, "No config available using default values\n");
	}

	//capture_buffer_size = image_hsize * image_vsize * 2;
	/* Only grayscale mode with gs_mode=1 uses 4 bytes for one
	 * pixel. Oll other modes use 2 bytes per pixel.*/
	if(gs_mode)
		video_buffer_size = prev_hsize * prev_vsize * 4;
	else
		video_buffer_size = prev_hsize * prev_vsize * 2;

	//video_buffer_size = video_buffer_size *2;
	video_bytes_used = video_buffer_size;

	/* Round up buffer sizes to the next page if needed */
	video_buffer_size = PAGE_ALIGN(video_buffer_size);
	capture_buffer_size = PAGE_ALIGN(capture_buffer_size);

	if(atmelisi_is_isi_v2()) {
		isi_writel(isi, V2_CTRL, ISI_BIT(V2_DIS));
		/* Check if module disable */
		while(isi_readl(isi, V2_STATUS) & ISI_BIT(V2_DIS));
	} else {
		isi_writel(isi, CR1, ISI_BIT(DIS));
	}

	irq = platform_get_irq(pdev,0);
	ret = request_irq(irq, isi_interrupt, 0, "isi", isi);
	if (ret) {
		dev_err(&pdev->dev, "unable to request irq %d\n", irq);
		goto err_req_irq;
	}
	isi->irq = irq;

	/* Allocate ISI capture buffer */
	isi->capture_buf = dma_alloc_coherent(&pdev->dev,
					      capture_buffer_size,
					      &isi->capture_phys,
					      GFP_KERNEL);
	if (!isi->capture_buf) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "failed to allocate capture buffer\n");
		goto err_alloc_cbuf;
	}

	/* Allocate and initialize video buffers */
	for(i=0;i < video_buffers; i++){
		memset(&isi->video_buffer[i], 0, sizeof(struct frame_buffer));
		isi->video_buffer[i].frame_buffer = 
			dma_alloc_coherent(&pdev->dev,
				video_buffer_size,
				(dma_addr_t *)
				&(isi->video_buffer[i].fb_desc.fb_address),
				GFP_KERNEL);
		if(!isi->video_buffer[i].frame_buffer){
			ret = -ENOMEM;
			dev_err(&pdev->dev, 
				"failed to allocate video buffer\n");
			goto err_alloc_vbuf;
		}
		
		isi->video_buffer[i].bytes_used = video_bytes_used;
		isi->video_buffer[i].status = FRAME_UNUSED;
		isi->video_buffer[i].index = i;

#ifdef DEBUG
	/* Put some color into the buffers */
	/*	memset(isi->video_buffer[i].frame_buffer, (i*4)%0xFF, 
			video_buffer_size); 
	*/		
#endif
	}
	/* set up frame buffer descriptor list for ISI module*/
	/* FIXME: The fbd list should be allocated with dma_alloc
	 * This is currently not the case.
	isi->fbd_list_start = dma_map_single(&pdev->dev,
		&isi->video_buffer[0].fb_desc,
		sizeof(struct fbd),
		DMA_NONE);
		*/
	isi->fbd_list_start = __pa(&isi->video_buffer[0].fb_desc);
	for(i=0; i < (video_buffers - 1); i++){
		isi->video_buffer[i].fb_desc.next_fbd_address =
		/*
			dma_map_single(&pdev->dev, 
				&isi->video_buffer[i+1].fb_desc,
				sizeof(struct fbd),
				DMA_NONE);*/
			__pa(&isi->video_buffer[i+1].fb_desc);
#if defined(ISI_CODEC)
#if defined(CONFIG_ARCH_AT91SAM9G45)
			isi->video_buffer[i].fb_desc.dma_ctrl = ISI_BIT(V2_DMA_FETCH) | ISI_BIT(V2_DMA_WB);

#endif
#endif
	}
	
	/* FIXME
	 * isi->video_buffer[i].fb_desc.next_fbd_address =
	 * 	isi->fbd_list_start;
	 */
	isi->video_buffer[i].fb_desc.next_fbd_address = 
		__pa(&isi->video_buffer[0].fb_desc);
#if defined(ISI_CODEC)
#if defined(CONFIG_ARCH_AT91SAM9G45) 
		isi->video_buffer[i].fb_desc.dma_ctrl = ISI_BIT(V2_DMA_FETCH) | ISI_BIT(V2_DMA_WB);
#endif
#endif
	for(i=0;i < video_buffers; i++){
		dev_info(&pdev->dev,"video buffer: %d bytes at %p (phys %08lx)\n",
		video_buffer_size, 
		isi->video_buffer[i].frame_buffer,
		(unsigned long) isi->video_buffer[i].fb_desc.fb_address);
	}

	dev_info(&pdev->dev,
		 "capture buffer: %d bytes at %p (phys 0x%08x)\n",
		 capture_buffer_size, isi->capture_buf,
		 isi->capture_phys);
	
	spin_lock_init(&isi->grabq_lock);
	isi->grabq = kfifo_alloc(sizeof(int) * video_buffers, GFP_KERNEL,
		&isi->grabq_lock);
	if(IS_ERR(isi->grabq)){
		dev_err(&pdev->dev, "fifo allocation failed\n");
		goto err_fifo_alloc1;
	}
	spin_lock_init(&isi->doneq_lock);
	isi->doneq = kfifo_alloc(sizeof(int) * video_buffers, GFP_KERNEL,
		&isi->doneq_lock);
	if(IS_ERR(isi->doneq)){
		dev_err(&pdev->dev, "fifo allocation failed\n");
		printk(KERN_INFO "pb allocating the queue\n");
		goto err_fifo_alloc2;
	}

	ret = video_register_device(&isi->cdev, VFL_TYPE_GRABBER, video_nr);
	if(ret){
		dev_err(&pdev->dev, "Registering capturing device failed\n");
		video_device_release(&isi->cdev);
		kfree(dev);
		printk(KERN_INFO "failed to register capture\n");
		goto err_register1;
	}

	ret = video_register_device(&isi->vdev, VFL_TYPE_GRABBER, video_nr);
	if (ret){
		dev_err(&pdev->dev, "Registering streaming device failed\n");
		video_device_release(&isi->vdev);
		kfree(dev);		
		printk(KERN_INFO "failed to register streaming\n");		
		goto err_register2;
	}
	platform_set_drvdata(pdev, isi);

	dev_info(&pdev->dev, "Atmel ISI V4L2 device at 0x%08lx\n",
		 (unsigned long)regs->start);

	return 0;

err_register2:
	video_unregister_device(&isi->cdev);
err_register1:
	kfifo_free(isi->doneq);
err_fifo_alloc2:
	kfifo_free(isi->grabq);
err_fifo_alloc1:
err_alloc_vbuf:
	while(i--)
		dma_free_coherent(&pdev->dev, video_buffer_size,
				isi->video_buffer[i].frame_buffer,
				isi->video_buffer[i].fb_desc.fb_address);
	dma_free_coherent(&pdev->dev, capture_buffer_size,
				isi->capture_buf,
				isi->capture_phys);
err_alloc_cbuf:
        free_irq(isi->irq, isi);
err_req_irq:
	iounmap(isi->regs);
err_ioremap:
	kfree(isi);
err_alloc_isi:
	clk_disable(pclk);

	return ret;

}

static struct platform_driver atmel_isi_driver = {
	.probe		= atmel_isi_probe,
	.remove		= __exit_p(atmel_isi_remove),
	.driver		= {
		.name = "atmel_isi",
		.owner = THIS_MODULE,
	},
};

static int __init atmel_isi_init_module(void)
{
	return  platform_driver_probe(&atmel_isi_driver, &atmel_isi_probe);
}


static void __exit atmel_isi_exit(void)
{
	platform_driver_unregister(&atmel_isi_driver);
}


module_init(atmel_isi_init_module);
module_exit(atmel_isi_exit);

MODULE_AUTHOR("Lars Häring <lharing@atmel.com>");
MODULE_DESCRIPTION("The V4L2 driver for atmel Linux");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("video");
